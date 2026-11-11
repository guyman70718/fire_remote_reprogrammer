# fire_remote_reprogrammer
### [!AI! !AI! !AI!](#ai-notice)
Reprogram Fire TV remote to use with alternate setups (targeted at HTPC)
## Features
 - Runs on ESP32 (Tested on S3 and C3)
 - Works with most Bluetooth Fire remotes ([Tested](#tested-remotes))
 - Change action of all remote keys (including Prime video, Netflix, etc)
 - IR Output
   - NEC Format
   - Multiple IR codes per button
   - Repeat/Delay
 - Keyboard Buttons
   - USB (ESP32-S3 Only) and BLE
   - Media Keys
   - Sleep/Wake
 - MQTT Actions
 - Multi Layers
   - Up to 8 Layers
   - Switch on the fly
   - Base layer can be used to pass unbound buttons
  
## Install
### WARNING! USE THIS SOFTWARE AT YOUR OWN RISK
#### Any time you write data to the remote you risk breaking something.
#### Do not use this on critical infrastructure like a medical Fire Remote, or the only TV when your kid has friends over.

## Requirements
- ESP32 Board (S3 Dual USB Port Preferred)
- Arduino IDE
  - Board Manager > "esp32" package
  - Library Manager > "NimBLE-Arduino" and "PubSubClient"
- Fire Remote
- WebSerial Supported Browser
  - Chrome/Chromium
  - Firefox with [WebSerial Extension](https://addons.mozilla.org/en-US/firefox/addon/webserial-for-firefox/)

## Steps
1. Download the hub.ino project
3. Compile and upload the project to your ESP32. Refer to board manufacturer for more information. Most generic ESP32S3 boards can use the "ESP32S3 Dev Module" preset.
    - If you are using a C3 or S3 with a single USB port, uncomment "#define OUTPUT_BLE" (remove //).
5. Open remote-binder.html on the same computer. Ensure the Serial Monitor in Arduino IDE is closed before connecting.
6. To pair the remote with the hub, hold the home button for 10s.
    - If this fails, you can reset the remote to factory settings by holding Back, Menu (3 Lines) and DPad Left for 10s.
  

## Tested Remotes
Feel free to open a pull request if you test one of your own.
- Alexa Remote Gen 3 (FCC ID: 2AYT9-7426)
- Toshiba Fire TV (FCC ID: PUW-2K19-YKF470)
#### Not Working
If there's interest in fixing these I'll take a look.
- Alexa Remote Gen 2 (FCC ID: 2AN7U-5463)
    - Fails to upload configuration
 
## AI Notice
This was entirely vibe coded. Was a simple experiment to see how much claude would be able to do without a human, but this is a project I've been wanting to take a look at for awhile. I didn't write a single line manually outside this README. Therefore, it should not be trusted for anything critical.
