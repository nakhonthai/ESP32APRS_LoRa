# ESP32APRS LoRa Simple Project

ESP32IAPRS LoRa is a Internet Gateway(IGate)/Dital Repeater(DiGi)/Tracker/Weather(WX)/Telemetry(TLM) with LoRa RF network in that is implemented for Espressif ESP32,ESP32-S3,ESP32C3 processor support.
 

## Feature
* Supported APRS on AX.25 Protocol
* Supported hardware: TTGO_LoRa32,TTGO T-Beam V1.2,LILYGO T-Beam Supreme,Heltec HTIT-Tracker,HT-CT62,D.I.Y Mod GPIO
* Support APRS internet gateway (IGATE)
* Support APRS digital repeater (DIGI)
* Support APRS tracker (TRACKER)
* Support GNSS External mod select UART0-2 and TCP Client
* Support TNC External mod select UART0-2 and Yaesu packet
* Support APRS IGATE/DIGI/WX with fix position for move position from GNSS
* Using ESP-Arduino development on Visual studio code + Platform IO
* Support LoRa Chip SX1231,SX1233,SX1261,SX1262,SX1268,SX12732,SX1273,SX1276,SX1278,SX1279,SX1280,SX1281,SX1282
* Support Frequncy by LoRa Chip 137Mhz-1020Mhz
* Support monitor display information and statistices
* Support Wi-Fi multi station or WiFi Access point
* support Web Service config and control system
* support filter packet rx/tx on igate,digi,display
* support VPN wireguard
* support global time zone
* support web service auth login
* support Telemetry on IGate,Digi,Tracker
* display received and transmit packet on the LED and display OLED

## Hardware screen short
![LILYGO_T_BEAM_SUPREME](doc/LILYGO_T_BEAM_SUPREME.jpg) ![HTIT_Tracker](doc/APRS_LoRa_Heltec_HTIT_Tracker_Quaterwave.jpg) ![APRS_LoRa_VHF](doc/APRS_LoRa_VHF_DIY.jpg)

## Web service screen short
![Map_LoRa_VHF](doc/TEST_APRS_LoRa_VHF.jpg)
![screen_dashboard](image/ESP32APRS_Screen_Dashboard.png) ![screen_radio](image/ESP32APRS_Screen_Radio.png) \
![screen_mod](image/ESP32APRS_Screen_Mod.png)

## WIKI How to install firmware
* [How to install firmware for TTGO LoRa32 V1.0](https://github.com/nakhonthai/ESP32APRS_LoRa/wiki/How-to-install-firmware-for-TTGO-LoRa32-V1.0)
* [How to install firmware for LILYGO T‐Beam SUPREME (M)](https://github.com/nakhonthai/ESP32APRS_LoRa/wiki/How-to-install-firmware-for-LILYGO-T%E2%80%90Beam-SUPREME-(M))
* [How to install firmware for HELTEC HTIT‐Tracker](https://github.com/nakhonthai/ESP32APRS_LoRa/wiki/How-to-install-firmware-for-HELTEC-HTIT%E2%80%90Tracker)
* [How to install firmware for ESP32‐DevKit,DI](https://github.com/nakhonthai/ESP32APRS_LoRa/wiki/How-to-install-firmware-for-ESP32%E2%80%90DevKit,DIY)

## WIKI How to configuration
* [How to config WX:Weather Station Report](https://github.com/nakhonthai/ESP32APRS_LoRa/wiki/How-to-config-WX:Weather-Station-Report)
* [How to config Telemetry on IGate Digi Tracker](https://github.com/nakhonthai/ESP32APRS_LoRa/wiki/How-to-config-Telemetry-on-IGate-Digi-Tracker)


## PlatformIO Quick Start

1. Install [Visual Studio Code](https://code.visualstudio.com/) and [Python](https://www.python.org/)
2. Search for the `PlatformIO` plugin in the `VisualStudioCode` extension and install it.
3. After the installation is complete, you need to restart `VisualStudioCode`
4. After restarting `VisualStudioCode`, select `File` in the upper left corner of `VisualStudioCode` -> `Open Folder` -> select the `ESP32APRS_LoRa` directory
5. Click on the `platformio.ini` file, and in the `platformio` column, cancel the sample line that needs to be used, please make sure that only one line is valid and change ESP type HT-CT62 Module as `default_envs = ht-ct62` or TTGO LoRa32 V1 as `default_envs = ttgo-lora32-v1`
6. Click the (✔) symbol in the lower left corner to compile
7. Connect the board to the computer USB
8. Click (→) to upload firmware and reboot again
9. After reboot display monitor and reconfig

## APRS Server service

- APRS SERVER of T2THAI at [aprs.dprns.com:14580](http://aprs.dprns.com:14501), CBAPRS at [aprs.dprns.com:24580](http://aprs.dprns.com:24501)
- APRS SERVER of T2THAI ampr host at [aprs.hs5tqa.ampr.org:14580](http://aprs.hs5tqa.ampr.org:14501)
- APRS MAP SERVICE [http://aprs.dprns.com](http://aprs.dprns.com)

## Developer/Support Information

- Author:	Mr.Somkiat Nakhonthai
- Callsign:	HS5TQA,Atten,Nakhonthai
- Country:	Bangkok,Thailand
- Github:	[https://github.com/nakhonthai](https://github.com/nakhonthai)
- Youtube:	[https://www.youtube.com/@HS5TQA](https://www.youtube.com/@HS5TQA)
- TikTok:   [https://www.tiktok.com/@hs5tqa](https://www.tiktok.com/@hs5tqa)
- Facebook:	[https://www.facebook.com/atten](https://www.facebook.com/atten)
- LineID: nakhonline
- WeChat: HS5TQA

## Donate

To support the development of ESP32APRS you can make us a donation using [github sponsors](https://github.com/sponsors/nakhonthai). \
If you want to donate some hardware to facilitate APRS porting and development, [contact us](https://www.facebook.com/atten). \
<a href="https://www.paypal.me/0hs5tqa0"><img src="https://github.com/nakhonthai/ESP32IGate/raw/master/blue.svg" height="40"></a> 

## ESP32 Flash Download Tools
https://www.espressif.com/en/support/download/other-tools

## Credits & Reference

- APRS Library by markqvist [LibAPRS](https://github.com/markqvist/LibAPRS)

## HITH
This project implement by APRS text (TNC2 Raw) only,It not support null string(0x00) in the package.
