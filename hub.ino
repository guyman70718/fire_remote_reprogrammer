// Fire TV remote hub v2: ESP32-S3 (USB keyboard) or ESP32-C3 and other chips without USB-OTG (BLE keyboard).
// NimBLE-Arduino 2.x, arduino-esp32 core.
//
// Output to the PC:
//   - Chips with native USB-OTG (S3): USB keyboard (default). Arduino IDE: USB Mode = "USB-OTG (TinyUSB)", CDC On Boot = Disabled.
//   - Chips without it (C3, ...): BLE keyboard, picked automatically. Pair "Fire Remote Hub" with the PC like any Bluetooth
//     keyboard. Arduino IDE (C3): USB CDC On Boot = Enabled (the serial/GUI link goes over the board's USB port).
//   - Uncomment the next line to force BLE keyboard output on an S3 too.
// #define OUTPUT_BLE
// The BLE keyboard has no PC sleep/wake/shutdown (those need USB), and the GUI hides them.
//
// Arduino IDE: USB Mode = "USB-OTG (TinyUSB)", USB CDC On Boot = Disabled.
// PC -> S3 native USB port = keyboard. UART port (COM25 for you) = serial log AND the GUI link (115200).
// Do NOT enable "Erase All Flash Before Sketch Upload": the remote's bond and the layer list live in flash.
//
// Behavior
//   - Connects to the remote and turns key presses it sends into USB keyboard / media keys using rules from the GUI,
//     per layer. A configurable remote key (default Prime Video) cycles layers.
//   - Accepts table uploads from remote-binder.html over the UART (line protocol below).
//
// Serial protocol (one command per line, '\n' terminated). Every command gets exactly one reply line:
//   HELLO                       -> @HELLO hub2 conn=<0|1> layer=<i> n=<layers> out=<usb|ble>
//   LIST                        -> @LIST <hex of the remote's CONTROL read> | @ERR ...
//   ERASE                       -> @OK | @ERR ...        (deletes ALL tables on the remote)
//   TBL <id:32hex> <len>        -> @OK                    (start receiving a table incl. its SHA-256 trailer)
//   DAT <hex>                   -> @OK <bytes so far>
//   WRITE                       -> @OK wrote | @ERR ...   (begin/chunks/verify on the remote)
//   LAYERS <n> <id32> <fwd> ... -> @OK                    (layer order; saved to flash; first = default)
//   SWITCH <i>                  -> @OK | @ERR ...         (activate layer i on the remote)
//   RCLEAR                      -> @OK                    (forget all keyboard rules, in RAM)
//   RULE <layer> <sig> <k|c|g> <mods hex> <code hex> -> @OK (when the remote sends <sig> on that layer: press a
//                                USB key (k, HID usage), a media key (c, consumer usage), or go to layer <code> (g))
//   RSAVE                       -> @OK                    (save the rules to flash)
//   SWKEY <sig>|-               -> @OK                    (which remote key cycles layers, or - for none; saved to flash)
//   WIFI <ssid hex> <pass hex>  -> @OK                    (string arguments are hex-encoded ASCII; "-" = empty)
//   MQTT <host hex> <port> <user hex> <pass hex> -> @OK
//   NCLEAR / NACT <i> m <retain> <topic hex> <payload hex> / NSAVE -> @OK   (MQTT actions; RULE ... n 0 <i> fires action i)
//   NTEST m <retain> <topic hex> <payload hex> -> @OK     (publish one message now)     NSTATUS -> @NET wifi=.. ip=.. mqtt=..
//   WATCH [0|1]                 -> @OK                    (every key press is reported as !KEY <sig> and nothing else happens)
//   LEARN [0|1]                 -> @OK                    (next remote key press is reported as !SIG <sig>, not acted on)
// A <sig> is "<report id>:<hex of the report bytes, trailing zeros trimmed>", e.g. 1:52 (Up) or 239:A1 (Prime Video).
// Unsolicited lines start with '!':  !CONN <0|1>   !LAYER <i>   !SIG <sig>
//
// Not compiled or run: treat as a first draft.
#include <NimBLEDevice.h>
#include <vector>
#include <WiFi.h>
#include <PubSubClient.h>   // Arduino Library Manager: "PubSubClient" by Nick O'Leary
#include <Preferences.h>
#include <esp_system.h>
#if __has_include(<esp_mac.h>)
#include <esp_mac.h>
#endif
#include "soc/soc_caps.h"
#if !defined(OUTPUT_BLE) && !SOC_USB_OTG_SUPPORTED
#define OUTPUT_BLE
#endif
#ifdef OUTPUT_BLE
#include <NimBLEHIDDevice.h>   // not pulled in by NimBLEDevice.h
#else
#include "USB.h"
#include "USBHIDKeyboard.h"
#include "USBHIDConsumerControl.h"
#include "USBHIDSystemControl.h"
#if __has_include("tusb.h")
#include "tusb.h"
#endif
#endif

// Types used in function signatures come first: the Arduino IDE inserts function prototypes above the first function.
struct NetAct { uint8_t kind; uint8_t flag; char s1[80]; char s2[200]; };   // kind 'm': s1 topic, s2 payload, flag&1 retain
struct Evt { uint8_t id; uint8_t len; uint8_t d[4]; };
struct RepInfo { uint16_t handle; uint8_t id; };

Preferences prefs;
void pollSerial();

static NimBLEUUID SVC_KEYMAP("fe151500-5e8d-11e6-8b77-86f30ca893d3");
static NimBLEUUID MAP_UUID("fe151501-5e8d-11e6-8b77-86f30ca893d3");
static NimBLEUUID CTRL_UUID("fe151502-5e8d-11e6-8b77-86f30ca893d3");
static NimBLEUUID BLAST_UUID("fe151503-5e8d-11e6-8b77-86f30ca893d3");

static const uint8_t PC_ID[16] = {0x7c,0x3e,0x5a,0x10,0x2b,0x4d,0x4f,0x6a,0x9e,0x81,0x5d,0x0c,0x4a,0x1b,0x2f,0x01};
static const uint8_t TV_ID[16] = {0x7c,0x3e,0x5a,0x10,0x2b,0x4d,0x4f,0x6a,0x9e,0x81,0x5d,0x0c,0x4a,0x1b,0x2f,0x02};

struct Rule { uint8_t layer, rid, slen, sig[4], type, mods; uint16_t code; };  // type 0 = USB key, 1 = media key
static Rule g_rules[128];
static int g_nrules = 0;
static uint8_t g_swRid = 239, g_swLen = 1, g_swSig[4] = {0xA1, 0, 0, 0};
static bool g_swDown = false, g_held = false;
static uint32_t g_pressAt = 0;
static volatile bool g_learn = false;
static volatile bool g_watch = false;   // GUI scan-ID finder: report every key press as !KEY and do nothing else

struct Layer { uint8_t id[16]; uint8_t fwd; };   // fwd is stored for compatibility; keyboard output is rule-driven now
static Layer g_layers[8];
static int g_nlayers = 0;
static int g_layer = 0;

static NimBLEAddress g_addr;
static int g_fail = 0, g_secFail = 0;
static bool g_lastSwitchAcked = true;
static volatile bool g_nullSeen = false;
static NimBLEAddress g_cacheAddr;      // remote whose GATT table we already discovered (it never changes)
static bool g_cacheValid = false;
static volatile bool g_found = false;
static volatile bool g_connected = false;
static NimBLEClient* g_client = nullptr;
static NimBLERemoteCharacteristic* g_ctrl = nullptr;
// Fast link (7.5-15 ms) for setup and uploads; relaxed afterwards so the remote can still save power and go to sleep.
static void setLinkSpeed(bool fast) {
  if (!g_client || !g_client->isConnected()) return;
  if (fast) g_client->updateConnParams(6, 12, 0, 200);
  else g_client->updateConnParams(24, 48, 4, 400);
}
static NimBLERemoteCharacteristic* g_map = nullptr;

static RepInfo g_reps[12];
static int g_nreps = 0;

static QueueHandle_t g_q;

// upload buffer
static uint8_t* g_up = nullptr;
static size_t g_upLen = 0, g_upGot = 0;
static uint8_t g_upId[16];

// ---------- helpers ----------
static int hexv(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
static int hexDecode(const char* s, uint8_t* out, int maxOut) {
  int n = 0;
  while (s[0] && s[1]) {
    int a = hexv(s[0]), b = hexv(s[1]);
    if (a < 0 || b < 0 || n >= maxOut) return -1;
    out[n++] = (a << 4) | b;
    s += 2;
  }
  return *s ? -1 : n;
}

// A new Bluetooth identity for the hub. A remote that still remembers the hub's old identity (but whose keys the hub
// no longer has) refuses to pair; presenting as a "new device" gets around that. Takes effect after a restart.
static void newIdentityAndRestart() {
  uint8_t mac[6];
  esp_fill_random(mac, 6);
  mac[0] = (mac[0] & 0xFC) | 0x02;   // unicast, locally administered
  prefs.putBytes("btmac", mac, 6);
  Serial.println("new Bluetooth identity saved, restarting");
  delay(300);
  ESP.restart();
}
static void printBonds(const char* why) {
  int n = NimBLEDevice::getNumBonds();
  Serial.printf("%s: %d\n", why, n);
  for (int i = 0; i < n; i++) Serial.printf("  bond %d: %s\n", i, NimBLEDevice::getBondedAddress(i).toString().c_str());
}
static void defaultLayers() {
  g_nlayers = 2;
  memcpy(g_layers[0].id, PC_ID, 16); g_layers[0].fwd = 1;
  memcpy(g_layers[1].id, TV_ID, 16); g_layers[1].fwd = 0;
}
static void loadLayers() {
  defaultLayers();
  uint8_t buf[1 + 8 * 17];
  size_t n = prefs.getBytes("layers", buf, sizeof(buf));
  if (n >= 1) {
    int c = buf[0];
    if (c >= 1 && c <= 8 && n == (size_t)(1 + c * 17)) {
      g_nlayers = c;
      for (int i = 0; i < c; i++) {
        memcpy(g_layers[i].id, buf + 1 + i * 17, 16);
        g_layers[i].fwd = buf[1 + i * 17 + 16];
      }
    }
  }
}
static void saveLayers() {
  uint8_t buf[1 + 8 * 17];
  buf[0] = g_nlayers;
  for (int i = 0; i < g_nlayers; i++) {
    memcpy(buf + 1 + i * 17, g_layers[i].id, 16);
    buf[1 + i * 17 + 16] = g_layers[i].fwd;
  }
  prefs.putBytes("layers", buf, 1 + g_nlayers * 17);
}

static void addRule(uint8_t layer, uint8_t rid, uint8_t sigByte, uint8_t type, uint8_t mods, uint16_t code) {
  if (g_nrules >= 128) return;
  Rule& r = g_rules[g_nrules++];
  memset(&r, 0, sizeof(r));
  r.layer = layer; r.rid = rid; r.slen = 1; r.sig[0] = sigByte; r.type = type; r.mods = mods; r.code = code;
}
static void defaultRules() {      // what the old hub did: arrows + Select -> USB keys on layer 0
  g_nrules = 0;
  addRule(0, 1, 0x52, 0, 0, 0x52); addRule(0, 1, 0x51, 0, 0, 0x51);
  addRule(0, 1, 0x50, 0, 0, 0x50); addRule(0, 1, 0x4F, 0, 0, 0x4F);
  addRule(0, 1, 0x58, 0, 0, 0x28);
}
static void loadRules() {
  defaultRules();
  if (prefs.isKey("nrules")) {
    int n = prefs.getUChar("nrules", 0);
    if (n <= 128 && prefs.getBytesLength("rules") == (size_t)n * sizeof(Rule)) {
      if (n) prefs.getBytes("rules", g_rules, n * sizeof(Rule));
      g_nrules = n;
    }
  }
  uint8_t sw[6];
  if (prefs.getBytes("swkey", sw, 6) == 6 && sw[1] <= 4) {
    g_swRid = sw[0]; g_swLen = sw[1]; memcpy(g_swSig, sw + 2, 4);
  }
}
static void saveRules() {
  prefs.putUChar("nrules", g_nrules);
  if (g_nrules) prefs.putBytes("rules", g_rules, g_nrules * sizeof(Rule));
}
// ---------- network actions (MQTT publish) ----------
static NetAct g_net[24];
static int g_nnet = 0;
static Preferences netPrefs;
static char g_ssid[64], g_wpass[80], g_mhost[64], g_muser[48], g_mpass[64];
static uint16_t g_mport = 1883;
static volatile bool g_netDirty = false;
static QueueHandle_t g_nq;
static WiFiClient g_wc;
static PubSubClient g_mqtt(g_wc);

static void decodeStr(const char* hex, char* out, size_t max) {
  out[0] = 0;
  if (!hex || !strcmp(hex, "-")) return;
  int n = hexDecode(hex, (uint8_t*)out, (int)max - 1);
  if (n < 0) n = 0;
  out[n] = 0;
}
static void loadNet() {
  netPrefs.begin("hubnet", false);
  netPrefs.getString("ssid", "").toCharArray(g_ssid, sizeof(g_ssid));
  netPrefs.getString("wpass", "").toCharArray(g_wpass, sizeof(g_wpass));
  netPrefs.getString("mhost", "").toCharArray(g_mhost, sizeof(g_mhost));
  netPrefs.getString("muser", "").toCharArray(g_muser, sizeof(g_muser));
  netPrefs.getString("mpass", "").toCharArray(g_mpass, sizeof(g_mpass));
  g_mport = netPrefs.getUShort("mport", 1883);
  int n = netPrefs.getUChar("n", 0);
  g_nnet = 0;
  for (int i = 0; i < n && i < 24; i++) {
    char k[8]; snprintf(k, sizeof(k), "a%d", i);
    if (netPrefs.getBytes(k, &g_net[g_nnet], sizeof(NetAct)) == sizeof(NetAct)) g_nnet++;
  }
}
static void saveNetActions() {
  netPrefs.putUChar("n", g_nnet);
  for (int i = 0; i < g_nnet; i++) { char k[8]; snprintf(k, sizeof(k), "a%d", i); netPrefs.putBytes(k, &g_net[i], sizeof(NetAct)); }
}
static void doNet(const NetAct& a) {
  if (WiFi.status() != WL_CONNECTED) { Serial.println("!NET fail no wifi"); return; }
  if (a.kind == 'm') {
    if (!g_mhost[0]) { Serial.println("!NET fail no mqtt broker set"); return; }
    if (!g_mqtt.connected()) {
      String cid = "remote-hub-" + String((uint32_t)ESP.getEfuseMac(), HEX);
      g_mqtt.setServer(g_mhost, g_mport);
      g_mqtt.setBufferSize(512);
      if (g_muser[0]) g_mqtt.connect(cid.c_str(), g_muser, g_mpass); else g_mqtt.connect(cid.c_str());
    }
    bool ok = g_mqtt.connected() && g_mqtt.publish(a.s1, a.s2, a.flag & 1);
    Serial.printf("!NET mqtt publish %s\n", ok ? "ok" : "fail");
  }
}
// WiFi/MQTT run in their own task so a slow network never delays key handling.
static void netTask(void*) {
  uint32_t lastWifi = 0, lastMqtt = 0;
  bool wasUp = false;
  for (;;) {
    if (g_netDirty) { g_netDirty = false; WiFi.disconnect(); g_mqtt.disconnect(); lastWifi = 0; lastMqtt = 0; wasUp = false; }
    if (g_ssid[0] && WiFi.status() != WL_CONNECTED && (lastWifi == 0 || millis() - lastWifi > 10000)) {
      lastWifi = millis() | 1;
      WiFi.mode(WIFI_STA);
      WiFi.begin(g_ssid, g_wpass);
    }
    bool up = WiFi.status() == WL_CONNECTED;
    if (up != wasUp) { wasUp = up; Serial.printf("!NET wifi %d %s\n", up, up ? WiFi.localIP().toString().c_str() : ""); }
    if (up && g_mhost[0]) {
      if (!g_mqtt.connected() && (lastMqtt == 0 || millis() - lastMqtt > 5000)) {
        lastMqtt = millis() | 1;
        String cid = "remote-hub-" + String((uint32_t)ESP.getEfuseMac(), HEX);
        g_mqtt.setServer(g_mhost, g_mport);
        g_mqtt.setBufferSize(512);
        bool ok = g_muser[0] ? g_mqtt.connect(cid.c_str(), g_muser, g_mpass) : g_mqtt.connect(cid.c_str());
        Serial.printf("!NET mqtt %d\n", ok);
      }
      g_mqtt.loop();
    }
    NetAct a;
    if (xQueueReceive(g_nq, &a, pdMS_TO_TICKS(50)) == pdTRUE) doNet(a);
  }
}

static bool parseSig(const char* s, uint8_t* rid, uint8_t* sig, uint8_t* slen) {
  if (!s) return false;
  const char* colon = strchr(s, ':');
  if (!colon) return false;
  int r = atoi(s);
  if (r < 0 || r > 255) return false;
  uint8_t tmp[4] = {0};
  int n = hexDecode(colon + 1, tmp, 4);
  if (n < 1) return false;
  *rid = r; *slen = n; memcpy(sig, tmp, 4);
  return true;
}
// ---------- output to the PC (USB or BLE keyboard) ----------
#ifndef OUTPUT_BLE
USBHIDKeyboard Keyboard;
USBHIDConsumerControl Consumer;
USBHIDSystemControl SystemControl;
static const char* OUT_NAME = "usb";
static void outBegin() {
  Keyboard.begin();
  Consumer.begin();
  SystemControl.begin();
  // By default the Arduino core does not advertise USB remote wakeup, so Windows offers no "wake the computer" option
  // for the device and the hub could never wake a sleeping PC. Advertise it.
  USB.usbAttributes(TUSB_DESC_CONFIG_ATT_SELF_POWERED | TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP);
  USB.begin();
}
static void outKeyDown(uint8_t usage) { Keyboard.pressRaw(usage); }      // 0xE0-0xE7 are the modifiers
static void outConsumerDown(uint16_t usage) { Consumer.press(usage); }
static void outSystemDown(uint8_t v) {                                    // 1 power off, 2 sleep, 3 wake the PC
  // A sleeping PC ignores key reports; the device has to signal USB remote wakeup first (the PC must allow it).
  if (v == 3 && tud_suspended()) { tud_remote_wakeup(); delay(150); }
  SystemControl.press(v);
}
static void outReleaseAll() { Keyboard.releaseAll(); Consumer.release(); SystemControl.release(); }
static void outKeepAdvertising() {}
#else
static const char* OUT_NAME = "ble";
static NimBLEServer* g_srv = nullptr;
static NimBLECharacteristic* g_inKbd = nullptr;
static NimBLECharacteristic* g_inCons = nullptr;
static uint8_t g_kMods = 0, g_kKeys[6] = {0};
static const uint8_t HID_MAP[] = {
  0x05,0x01, 0x09,0x06, 0xA1,0x01, 0x85,0x01,                       // keyboard, report id 1
  0x05,0x07, 0x19,0xE0, 0x29,0xE7, 0x15,0x00, 0x25,0x01, 0x75,0x01, 0x95,0x08, 0x81,0x02,   // modifiers
  0x95,0x01, 0x75,0x08, 0x81,0x01,                                  // reserved
  0x95,0x06, 0x75,0x08, 0x15,0x00, 0x25,0x65, 0x05,0x07, 0x19,0x00, 0x29,0x65, 0x81,0x00,   // 6 keys
  0xC0,
  0x05,0x0C, 0x09,0x01, 0xA1,0x01, 0x85,0x02,                       // consumer control, report id 2
  0x15,0x00, 0x26,0xFF,0x03, 0x19,0x00, 0x2A,0xFF,0x03, 0x75,0x10, 0x95,0x01, 0x81,0x00,   // one 16-bit usage
  0xC0
};
static void outSendKbd() {
  if (!g_inKbd || !g_srv || g_srv->getConnectedCount() == 0) return;
  uint8_t rep[8] = {g_kMods, 0, g_kKeys[0], g_kKeys[1], g_kKeys[2], g_kKeys[3], g_kKeys[4], g_kKeys[5]};
  g_inKbd->setValue(rep, 8);
  g_inKbd->notify();
}
static void outBegin() {
  g_srv = NimBLEDevice::createServer();
  g_srv->advertiseOnDisconnect(true);
  NimBLEHIDDevice* hid = new NimBLEHIDDevice(g_srv);
  g_inKbd = hid->getInputReport(1);
  g_inCons = hid->getInputReport(2);
  hid->setManufacturer("Remote Binder");
  hid->setPnp(0x02, 0xe502, 0xa111, 0x0210);
  hid->setHidInfo(0x00, 0x01);
  hid->setReportMap((uint8_t*)HID_MAP, sizeof(HID_MAP));
  hid->setBatteryLevel(100);
  g_srv->start();
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setAppearance(0x03C1);                                          // keyboard
  adv->addServiceUUID(hid->getHidService()->getUUID());
  // Put the name in the scan response so the PC/phone list shows it, and flags + HID service + appearance in the advert.
  NimBLEAdvertisementData sr;
  sr.setName("Fire Remote Hub");
  adv->setScanResponseData(sr);
  bool ok = adv->start();
  Serial.printf("BLE keyboard advertising as 'Fire Remote Hub': %s\n", ok ? "started" : "FAILED");
}
// Connecting to the remote (central role) can stop advertising; make sure the PC can always find us.
static void outKeepAdvertising() {
  static uint32_t last = 0;
  if (millis() - last < 1000) return;
  last = millis();
  if (g_srv && g_srv->getConnectedCount() == 0) {
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    if (!adv->isAdvertising() && adv->start()) Serial.println("BLE keyboard: advertising restarted");
  }
}
static void outKeyDown(uint8_t usage) {
  if (usage >= 0xE0 && usage <= 0xE7) g_kMods |= 1 << (usage - 0xE0);
  else {
    for (int i = 0; i < 6; i++) if (g_kKeys[i] == usage) { outSendKbd(); return; }
    for (int i = 0; i < 6; i++) if (!g_kKeys[i]) { g_kKeys[i] = usage; break; }
  }
  outSendKbd();
}
static void outConsumerDown(uint16_t usage) {
  if (!g_inCons || !g_srv || g_srv->getConnectedCount() == 0) return;
  uint8_t rep[2] = {(uint8_t)(usage & 0xFF), (uint8_t)(usage >> 8)};
  g_inCons->setValue(rep, 2);
  g_inCons->notify();
}
static void outSystemDown(uint8_t) {}                                  // not available over BLE
static void outReleaseAll() {
  bool had = g_kMods || g_kKeys[0];
  g_kMods = 0; memset(g_kKeys, 0, sizeof(g_kKeys));
  if (had) outSendKbd();
  outConsumerDown(0);
}
#endif
// The PC's pairing (BLE output) lives in the same bond store as the remote's: only ever delete the remote's.
static void dropRemoteBond(const NimBLEAddress& a) { NimBLEDevice::deleteBond(a); }
static void dropCorruptBond() {
#ifdef OUTPUT_BLE
  NimBLEDevice::deleteBond(NimBLEAddress("00:00:00:00:00:00", 0));
#else
  NimBLEDevice::deleteAllBonds();
#endif
}
static void releaseKeys() { outReleaseAll(); g_held = false; }
static void setLinkSpeed(bool fast);
static bool switchLayerTable(int idx);
static void applyRule(const Rule& r) {
  if (r.type == 2) {                               // go to layer <code>
    if (r.code < g_nlayers && r.code != g_layer) {
      g_layer = r.code;
      releaseKeys();
      bool ok = switchLayerTable(g_layer);
      Serial.printf("layer -> %d (%s)\n!LAYER %d\n", g_layer, !ok ? "not sent" : g_lastSwitchAcked ? "ok" : "sent, no ack", g_layer);
    }
    return;
  }
  if (r.type == 4) {                               // network action: code = index into g_net
    if (r.code < g_nnet) xQueueSend(g_nq, &g_net[r.code], 0);
    return;
  }
  if (r.type == 3) outSystemDown(r.code);          // system control: 1 power off, 2 sleep, 3 wake the PC (USB only)
  else if (r.type == 1) outConsumerDown(r.code);
  else {
    for (int b = 0; b < 8; b++) if (r.mods & (1 << b)) outKeyDown(0xE0 + b);
    if (r.code) outKeyDown((uint8_t)r.code);
  }
  g_held = true;
  g_pressAt = millis();
}

// ---------- BLE ----------
class ScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* d) override {
    if (g_found) return;
    {   // diagnostics: list each strong nearby device once, so we can see what a remote in pairing mode advertises
      static uint32_t seen[16];
      static int ns = 0;
      uint32_t key = 0;
      std::string as = d->getAddress().toString();
      for (char ch : as) key = key * 131 + (uint8_t)ch;
      bool dup = false;
      for (int i = 0; i < ns; i++) if (seen[i] == key) dup = true;
      if (!dup && d->getRSSI() > -75 && ns < 16) {
        seen[ns++] = key;
        Serial.printf("nearby: %s rssi %d name '%s' hid=%d\n", as.c_str(), d->getRSSI(), d->getName().c_str(), d->isAdvertisingService(NimBLEUUID((uint16_t)0x1812)));
      }
    }
    std::string nm = d->getName();
    bool match = NimBLEDevice::isBonded(d->getAddress()) || d->getAddress().toString() == "a8:42:a7:45:10:6f" || nm == "AR" || nm.find("Alexa") != std::string::npos || nm.find("Fire") != std::string::npos;
    if (!match) return;
    if (d->getAddress().isNull()) { g_nullSeen = true; return; }   // a remote reported with an all-zero address: corrupt bond
    Serial.printf("found remote %s type %d rssi %d name '%s'\n", d->getAddress().toString().c_str(), (int)d->getAddress().getType(), d->getRSSI(), d->getName().c_str());
    g_addr = d->getAddress();
    g_found = true;
    NimBLEDevice::getScan()->stop();
  }
};

class ClientCB : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient*, int reason) override {
    g_connected = false;
    g_ctrl = nullptr;
    g_map = nullptr;
    Serial.printf("remote disconnected (%d)\n!CONN 0\n", reason);
  }
};

static void onReport(NimBLERemoteCharacteristic* c, uint8_t* d, size_t n, bool) {
  if (!g_connected) Serial.printf("[%lu] early report (arrived before setup finished)\n", (unsigned long)millis());
  Evt e = {0xFF, (uint8_t)min(n, (size_t)4), {0, 0, 0, 0}};
  for (int i = 0; i < g_nreps; i++) if (g_reps[i].handle == c->getHandle()) e.id = g_reps[i].id;
  memcpy(e.d, d, e.len);
  xQueueSend(g_q, &e, 0);
}
static void onQuiet(NimBLERemoteCharacteristic*, uint8_t*, size_t, bool) {}

static bool writeCtrl(const uint8_t* d, size_t n, int attempts = 1) {
  if (!g_connected || !g_ctrl) return false;
  for (int a = 0; a < attempts; a++) {
    if (a) delay(500);
    if (g_ctrl->writeValue(d, n, true)) return true;
  }
  return false;
}
// Switching tables: the gen3 remote often does not acknowledge the write even though it takes effect, so a missing
// ack is reported as such instead of as a failure.

static bool switchLayerTable(int idx) {
  if (idx < 0 || idx >= g_nlayers) return false;
  uint8_t cmd[18] = {0x01};
  memcpy(cmd + 1, g_layers[idx].id, 16);
  cmd[17] = 0x00;
  g_lastSwitchAcked = writeCtrl(cmd, 18, 1);
  if (!g_lastSwitchAcked && g_connected && g_ctrl) g_ctrl->writeValue(cmd, 18, false);   // also send without waiting for an ack
  return g_connected;
}

static const char* writeTableToRemote() {
  if (!g_connected || !g_ctrl || !g_map) return "notconnected";
  uint8_t begin[22] = {0x02, 0x01};
  memcpy(begin + 2, g_upId, 16);
  begin[18] = 0; begin[19] = 0;
  begin[20] = g_upLen & 0xFF;
  begin[21] = g_upLen >> 8;
  if (!writeCtrl(begin, sizeof(begin), 2)) return "begin";
  for (size_t off = 0; off < g_upLen; off += 200) {
    uint8_t chunk[200] = {0};
    memcpy(chunk, g_up + off, min((size_t)200, g_upLen - off));
    if (!g_map->writeValue(chunk, 200, g_map->canWrite())) return "chunk";
  }
  NimBLEAttValue res = g_map->readValue();
  if (res.size() < 1 || res.data()[0] != 0x02) return "verify";
  return nullptr;
}

static bool connectRemote() {
  g_found = false;
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->clearResults();
  scan->start(0);
  uint32_t t0 = millis();
  while (!g_found && !g_nullSeen && millis() - t0 < 3000) { pollSerial(); outKeepAdvertising(); delay(30); }
  if (g_nullSeen) {
    g_nullSeen = false;
    scan->stop();
    dropCorruptBond();
    Serial.println("remote showed up with address 00:00:00:00:00:00 (a corrupt stored bond). Deleted all bonds: keep holding Home on the remote.");
    delay(500);
    return false;
  }
  if (!g_found) { scan->stop(); return false; }
  uint32_t tFound = millis();

  if (!g_client) {
    g_client = NimBLEDevice::createClient();
    static ClientCB ccb;
    g_client->setClientCallbacks(&ccb, false);
  }
  g_client->setConnectionParams(6, 12, 0, 200);   // ask for a fast link (7.5-15 ms, no slave latency): every setup step is a round trip
  bool reuse = g_cacheValid && g_cacheAddr == g_addr;   // keep the discovered services: skips slow rediscovery on every wake
  // No MTU exchange yet: it costs round trips before encryption can start, and the remote's queued press may go out as soon as the link is encrypted.
  if (!g_client->connect(g_addr, !reuse, false, false)) {
    g_cacheValid = false;
    // A remote that lost its bond (batteries pulled, re-paired elsewhere) refuses us while we still hold the old
    // keys. Drop the stale bond and use a fresh client; then hold Home on the remote to pair again.
    g_fail++;
    Serial.printf("connect failed (%d) for %s\n", g_fail, g_addr.toString().c_str());
    NimBLEDevice::deleteClient(g_client);
    g_client = nullptr;
    // Every failed connect clears the bond for that remote (a remote that lost its bond refuses a hub that still has keys).
    if (g_addr.isNull()) dropCorruptBond();
    else dropRemoteBond(g_addr);
    Serial.println("cleared stored bond(s): hold Home on the remote until it pairs");
    uint32_t t = millis();
    while (millis() - t < 1500) { pollSerial(); delay(30); }   // back off instead of spamming
    return false;
  }
  g_fail = 0;
  if (!g_client->secureConnection()) {
    // Without encryption the remote sends no key presses and hangs up (reason 531), so don't pretend we are connected.
    g_secFail++;
    Serial.printf("pairing failed (%d): hold Home on the remote\n", g_secFail);
    g_client->disconnect();
    dropRemoteBond(g_addr);
    uint32_t t = millis();
    while (millis() - t < 1500) { pollSerial(); delay(30); }
    return false;
  }
  g_secFail = 0;
  printBonds("stored bonds after pairing");

  uint32_t tSecure = millis();
  // Subscribe to the key reports FIRST: a key press that woke the remote is only delivered if we are subscribed in time.
  NimBLERemoteService* hid = g_client->getService(NimBLEUUID((uint16_t)0x1812));
  if (!hid) { Serial.println("no HID service"); g_cacheValid = false; return false; }
  bool haveReps = reuse && g_nreps > 0;
  if (!haveReps) g_nreps = 0;
  std::vector<NimBLERemoteCharacteristic*> reps;
  for (auto* ch : hid->getCharacteristics(true)) {
    if (!(ch->getUUID() == NimBLEUUID((uint16_t)0x2A4D))) continue;
    reps.push_back(ch);
    if (!haveReps) {
      auto* rr = ch->getDescriptor(NimBLEUUID((uint16_t)0x2908));
      if (rr && g_nreps < 12) {
        NimBLEAttValue v = rr->readValue();
        if (v.size() >= 2) g_reps[g_nreps++] = {ch->getHandle(), v.data()[0]};
      }
    }
  }
  // Each subscription is a round trip to the remote, so do the reports that carry key presses (1 keyboard, 2 consumer,
  // 239 vendor) first and everything else afterwards.
  for (int pass = 0; pass < 2; pass++) {
    for (auto* ch : reps) {
      int id = -1;
      for (int k = 0; k < g_nreps; k++) if (g_reps[k].handle == ch->getHandle()) id = g_reps[k].id;
      bool key = (id == 1 || id == 2 || id == 239);
      if ((pass == 0) != key || !ch->canNotify()) continue;
      ch->subscribe(true, onReport);
    }
    if (pass == 0) { g_connected = true; Serial.printf("[%lu] key reports subscribed\n", (unsigned long)millis()); }
  }
  uint32_t tSub = millis();

  NimBLERemoteService* km = g_client->getService(SVC_KEYMAP);
  g_ctrl = km ? km->getCharacteristic(CTRL_UUID) : nullptr;
  g_map = km ? km->getCharacteristic(MAP_UUID) : nullptr;
  if (km) {
    auto* blast = km->getCharacteristic(BLAST_UUID);
    if (blast && blast->canNotify()) blast->subscribe(true, onQuiet);
    if (g_map && g_map->canNotify()) g_map->subscribe(true, onQuiet);
  }
  g_cacheValid = true; g_cacheAddr = g_addr;
  Serial.printf("[%lu] timing: found->encrypted %lu ms, encrypted->reports subscribed %lu ms (cache %s)\n",
                (unsigned long)millis(), (unsigned long)(tSecure - tFound), (unsigned long)(tSub - tSecure), reuse ? "used" : "empty");

  // Keep the layer that was active before the remote went to sleep (g_layer starts at 0 only at boot).
  if (g_layer < 0 || g_layer >= g_nlayers) g_layer = 0;
  bool sw0 = switchLayerTable(g_layer);
  g_client->exchangeMTU();   // needed for 200-byte table writes during uploads
  setLinkSpeed(false);
  Serial.printf("connected; layer %d: %s\n!CONN 1\n!LAYER %d\n", g_layer, !sw0 ? "not sent" : g_lastSwitchAcked ? "ok" : "sent (no ack)", g_layer);
  return true;
}

// ---------- serial protocol ----------
static void handleLine(char* line) {
  char* cmd = strtok(line, " ");
  if (!cmd) return;

  if (!strcmp(cmd, "HELLO")) {
    Serial.printf("@HELLO hub2 conn=%d layer=%d n=%d out=%s\n", g_connected ? 1 : 0, g_layer, g_nlayers, OUT_NAME);
    return;
  }
  if (!strcmp(cmd, "LAYERS")) {
    char* ns = strtok(NULL, " ");
    int n = ns ? atoi(ns) : 0;
    if (n < 1 || n > 8) { Serial.println("@ERR n"); return; }
    Layer tmp[8];
    for (int i = 0; i < n; i++) {
      char* id = strtok(NULL, " ");
      char* f = strtok(NULL, " ");
      if (!id || !f || strlen(id) != 32 || hexDecode(id, tmp[i].id, 16) != 16) { Serial.println("@ERR layer"); return; }
      tmp[i].fwd = atoi(f) ? 1 : 0;
    }
    for (int i = 0; i < n; i++) g_layers[i] = tmp[i];
    g_nlayers = n;
    if (g_layer >= n) g_layer = 0;
    saveLayers();
    Serial.println("@OK");
    return;
  }
  if (!strcmp(cmd, "NEWID")) { Serial.println("@OK"); newIdentityAndRestart(); return; }
  if (!strcmp(cmd, "UNBOND")) { NimBLEDevice::deleteAllBonds(); Serial.println("@OK"); return; }
  if (!strcmp(cmd, "RCLEAR")) { g_nrules = 0; releaseKeys(); Serial.println("@OK"); return; }
  if (!strcmp(cmd, "RULE")) {
    char* ls = strtok(NULL, " "); char* sg = strtok(NULL, " "); char* ty = strtok(NULL, " ");
    char* ms = strtok(NULL, " "); char* cs = strtok(NULL, " ");
    uint8_t rid, sig[4], slen;
    if (!ls || !ty || !ms || !cs || !parseSig(sg, &rid, sig, &slen) || g_nrules >= 128) { Serial.println("@ERR rule"); return; }
    Rule& r = g_rules[g_nrules];
    memset(&r, 0, sizeof(r));
    r.layer = atoi(ls); r.rid = rid; r.slen = slen; memcpy(r.sig, sig, 4);
    r.type = (ty[0] == 'c') ? 1 : (ty[0] == 'g') ? 2 : (ty[0] == 's') ? 3 : (ty[0] == 'n') ? 4 : 0;   // 0 USB key, 1 media key, 2 go to layer, 3 system (sleep/wake)
    r.mods = strtol(ms, NULL, 16); r.code = strtol(cs, NULL, 16);
    g_nrules++;
    Serial.println("@OK");
    return;
  }
  if (!strcmp(cmd, "RSAVE")) { saveRules(); Serial.println("@OK"); return; }
  if (!strcmp(cmd, "SWKEY")) {
    uint8_t rid, sig[4], slen;
    char* arg = strtok(NULL, " ");
    if (arg && !strcmp(arg, "-")) { rid = 0; slen = 0; memset(sig, 0, 4); }       // "-" = no layer switch key (use go-to-layer actions only)
    else if (!parseSig(arg, &rid, sig, &slen)) { Serial.println("@ERR sig"); return; }
    g_swRid = rid; g_swLen = slen; memcpy(g_swSig, sig, 4);
    uint8_t sw[6] = {rid, slen, sig[0], sig[1], sig[2], sig[3]};
    prefs.putBytes("swkey", sw, 6);
    Serial.println("@OK");
    return;
  }
  if (!strcmp(cmd, "WIFI")) {
    decodeStr(strtok(NULL, " "), g_ssid, sizeof(g_ssid)); decodeStr(strtok(NULL, " "), g_wpass, sizeof(g_wpass));
    netPrefs.putString("ssid", g_ssid); netPrefs.putString("wpass", g_wpass); g_netDirty = true;
    Serial.println("@OK"); return;
  }
  if (!strcmp(cmd, "MQTT")) {
    decodeStr(strtok(NULL, " "), g_mhost, sizeof(g_mhost)); char* pt = strtok(NULL, " "); g_mport = pt ? atoi(pt) : 1883;
    decodeStr(strtok(NULL, " "), g_muser, sizeof(g_muser)); decodeStr(strtok(NULL, " "), g_mpass, sizeof(g_mpass));
    netPrefs.putString("mhost", g_mhost); netPrefs.putUShort("mport", g_mport); netPrefs.putString("muser", g_muser); netPrefs.putString("mpass", g_mpass);
    g_netDirty = true;
    Serial.println("@OK"); return;
  }
  if (!strcmp(cmd, "NCLEAR")) { g_nnet = 0; Serial.println("@OK"); return; }
  if (!strcmp(cmd, "NACT") || !strcmp(cmd, "NTEST")) {
    bool test = cmd[1] == 'T';
    char* idx = test ? NULL : strtok(NULL, " ");
    char* kd = strtok(NULL, " "); char* fl = strtok(NULL, " "); char* h1 = strtok(NULL, " "); char* h2 = strtok(NULL, " ");
    if (!kd || !fl || !h1 || (!test && !idx)) { Serial.println("@ERR net"); return; }
    NetAct a; memset(&a, 0, sizeof(a));
    a.kind = kd[0]; a.flag = atoi(fl);
    decodeStr(h1, a.s1, sizeof(a.s1)); decodeStr(h2, a.s2, sizeof(a.s2));
    if (test) { xQueueSend(g_nq, &a, 0); Serial.println("@OK"); return; }
    int i = atoi(idx);
    if (i < 0 || i >= 24) { Serial.println("@ERR net"); return; }
    g_net[i] = a; if (i >= g_nnet) g_nnet = i + 1;
    Serial.println("@OK"); return;
  }
  if (!strcmp(cmd, "NSAVE")) { saveNetActions(); Serial.println("@OK"); return; }
  if (!strcmp(cmd, "NSTATUS")) {
    Serial.printf("@NET wifi=%d ip=%s mqtt=%d ssid=%d actions=%d\n", WiFi.status() == WL_CONNECTED, WiFi.localIP().toString().c_str(),
                  g_mqtt.connected(), g_ssid[0] != 0, g_nnet);
    return;
  }
  if (!strcmp(cmd, "WATCH")) {
    char* a = strtok(NULL, " ");
    g_watch = !(a && a[0] == '0');
    Serial.println("@OK");
    return;
  }
  if (!strcmp(cmd, "LEARN")) {
    char* a = strtok(NULL, " ");
    g_learn = !(a && a[0] == '0');
    Serial.println("@OK");
    return;
  }
  if (!strcmp(cmd, "TBL")) {
    char* id = strtok(NULL, " ");
    char* ls = strtok(NULL, " ");
    long len = ls ? atol(ls) : 0;
    if (!id || strlen(id) != 32 || hexDecode(id, g_upId, 16) != 16 || len < 33 || len > 60000) { Serial.println("@ERR args"); return; }
    if (g_up) { free(g_up); g_up = nullptr; }
    g_up = (uint8_t*)malloc(len);
    if (!g_up) { Serial.println("@ERR memory"); return; }
    g_upLen = len;
    g_upGot = 0;
    Serial.println("@OK");
    return;
  }
  if (!strcmp(cmd, "DAT")) {
    char* hex = strtok(NULL, " ");
    if (!g_up || !hex) { Serial.println("@ERR state"); return; }
    int n = hexDecode(hex, g_up + g_upGot, (int)(g_upLen - g_upGot));
    if (n < 0) { Serial.println("@ERR hex"); return; }
    g_upGot += n;
    Serial.printf("@OK %u\n", (unsigned)g_upGot);
    return;
  }

  // everything below needs the remote
  if (!g_connected || !g_ctrl) { Serial.println("@ERR notconnected"); return; }

  if (!strcmp(cmd, "LIST")) {
    NimBLEAttValue v = g_ctrl->readValue();
    Serial.print("@LIST ");
    for (size_t i = 0; i < v.size(); i++) Serial.printf("%02X", v.data()[i]);
    Serial.println();
    return;
  }
  if (!strcmp(cmd, "ERASE")) {
    setLinkSpeed(true);
    uint8_t e = 0x10;
    if (writeCtrl(&e, 1, 2)) { delay(300); Serial.println("@OK"); }
    else Serial.println("@ERR erase");
    return;
  }
  if (!strcmp(cmd, "WRITE")) {
    if (!g_up || g_upGot != g_upLen) { Serial.println("@ERR incomplete"); return; }
    const char* err = writeTableToRemote();
    if (err) Serial.printf("@ERR %s\n", err);
    else Serial.println("@OK wrote");
    return;
  }
  if (!strcmp(cmd, "SWITCH")) {
    char* is = strtok(NULL, " ");
    int i = is ? atoi(is) : -1;
    if (switchLayerTable(i)) { g_layer = i; setLinkSpeed(false); Serial.println("@OK"); }
    else Serial.println("@ERR switch");
    return;
  }
  Serial.println("@ERR unknown");
}

void pollSerial() {
  static char line[1600];
  static int n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[n] = 0;
      if (n > 0) handleLine(line);
      n = 0;
    } else if (n < (int)sizeof(line) - 1) {
      line[n++] = c;
    } else {
      n = 0;  // overlong line: drop it
    }
  }
}

// ---------- remote input ----------
static void handleEvt(const Evt& e) {
  Serial.printf("[%lu] report id=%u:", (unsigned long)millis(), e.id);
  for (int i = 0; i < e.len; i++) Serial.printf(" %02X", e.d[i]);
  Serial.println();

  uint8_t tl = e.len;
  while (tl && e.d[tl - 1] == 0) tl--;
  if (tl == 0) {                                   // all zero = release
    if (e.id == g_swRid) g_swDown = false;
    if (g_held) {
      // After a wake the remote delivers its queued press and release almost together; hold the key long enough for the PC to see it.
      uint32_t dt = millis() - g_pressAt;
      if (dt < 40) delay(40 - dt);
      releaseKeys();
    }
    return;
  }
  if (g_watch) {                                   // scan-ID finder: just report the press
    Serial.printf("!KEY %u:", e.id);
    for (int i = 0; i < tl; i++) Serial.printf("%02X", e.d[i]);
    Serial.println();
    return;
  }
  if (g_learn) {                                   // GUI is waiting to learn which key this is
    g_learn = false;
    Serial.printf("!SIG %u:", e.id);
    for (int i = 0; i < tl; i++) Serial.printf("%02X", e.d[i]);
    Serial.println();
    return;
  }
  if (e.id == g_swRid && tl == g_swLen && !memcmp(e.d, g_swSig, tl)) {   // layer switch key
    if (!g_swDown) {
      g_swDown = true;
      g_layer = (g_layer + 1) % g_nlayers;
      releaseKeys();
      bool ok = switchLayerTable(g_layer);
      Serial.printf("layer -> %d (%s)\n!LAYER %d\n", g_layer, !ok ? "not sent" : g_lastSwitchAcked ? "ok" : "sent, no ack", g_layer);
    }
    return;
  }
  // Every matching rule fires (so one key can send e.g. PC sleep and switch layer); layer changes go last.
  int layerRule = -1;
  for (int i = 0; i < g_nrules; i++) {
    const Rule& r = g_rules[i];
    if (r.layer == g_layer && r.rid == e.id && r.slen == tl && !memcmp(r.sig, e.d, tl)) {
      if (r.type == 2) layerRule = i; else applyRule(r);
    }
  }
  if (layerRule >= 0) applyRule(g_rules[layerRule]);
}

void setup() {
  Serial.setRxBufferSize(4096);
  Serial.begin(115200);
  g_q = xQueueCreate(32, sizeof(Evt));
  prefs.begin("hub", false);
  { uint8_t mac[6]; if (prefs.getBytes("btmac", mac, 6) == 6) esp_base_mac_addr_set(mac); }
  loadLayers();
  loadRules();
  loadNet();
  g_nq = xQueueCreate(8, sizeof(NetAct));
  xTaskCreate(netTask, "net", 10240, NULL, 1, NULL);
  NimBLEDevice::init(OUT_NAME[0] == 'b' ? "Fire Remote Hub" : "esp32-hub");
  NimBLEDevice::setSecurityAuth(true, false, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  // Only exchange the encryption key, not the remote's identity key (IRK). With the IRK the stored bond ended up with
  // an all-zero peer address for this remote, so after a restart the hub could no longer reconnect to it.
  NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC);
  NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC);
  NimBLEDevice::setMTU(247);
  printBonds("stored bonds at start");
  outBegin();     // USB keyboard, or the BLE keyboard server (needs the BLE stack initialised above)
  Serial.printf("PC output: %s keyboard\n", OUT_NAME);
  static ScanCB cb;
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&cb, true);
  scan->setActiveScan(true);
  scan->setInterval(100);   // listen continuously so the remote's first advertisement is caught
  scan->setWindow(100);
  Serial.println("hub2 starting. First time: hold Home on the remote until it pairs.");
}

void loop() {
  pollSerial();
  outKeepAdvertising();
  if (!g_connected) {
    releaseKeys();
    if (!connectRemote()) delay(100);
    return;
  }
  Evt e;
  while (xQueueReceive(g_q, &e, pdMS_TO_TICKS(20)) == pdTRUE) handleEvt(e);
}
