# AT Command Reference - ESP32 APRS LoRa

AT Command Reference Guide for ESP32 APRS LoRa Firmware

---

## Table of Contents

- [Basic Commands](#basic-commands)
- [WiFi Commands](#wifi-commands)
- [Time Commands](#time-commands)
- [Bluetooth Commands](#bluetooth-commands)
- [RF/LoRa Commands](#rflora-commands)
- [IGATE Commands](#igate-commands)
- [DIGI Commands](#digi-commands)
- [TRACKER Commands](#tracker-commands)
- [WEATHER Commands](#weather-commands)
- [TELEMETRY Commands](#telemetry-commands)
- [OLED/Display Commands](#oleddisplay-commands)
- [Network/Server Commands](#networkserver-commands)
- [WireGuard Commands](#wireguard-commands)
- [GNSS/GPS Commands](#gnssgps-commands)
- [I2C Commands](#i2c-commands)
- [UART Commands](#uart-commands)
- [Power Management Commands](#power-management-commands)
- [MQTT Commands](#mqtt-commands)
- [AT Command Interface Commands](#at-command-interface-commands)

---

## Basic Commands

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT | `AT` | None | Test command, check connection | `AT` | `OK` |
| AT+RESET | `AT+RESET` or `AT+RESTART` | None | Reboot the system | `AT+RESET` | (device restarts) |
| AT+CHIPID? | `AT+CHIPID?` | None | Get Chip ID (MAC Address) | `AT+CHIPID?` | `240AC4123456` |
| AT+SAVECONFIG | `AT+SAVECONFIG` | None | Save configuration to file | `AT+SAVECONFIG` | `Configuration Saved` |
| LOADCONFIG | `LOADCONFIG` | None | Load configuration from file | `LOADCONFIG` | `Configuration Loaded` |

---

## WiFi Commands

### WiFi Connection

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+WIFI_DISCONNECT | `AT+WIFI_DISCONNECT` | None | Disconnect WiFi | `AT+WIFI_DISCONNECT` | `WiFi Disconnected` |
| AT+WIFI_CONNECT | `AT+WIFI_CONNECT` | None | Reconnect WiFi | `AT+WIFI_CONNECT` | `WiFi Reconnecting` |
| AT+WIFI_STATUS? | `AT+WIFI_STATUS?` | None | Check WiFi status | `AT+WIFI_STATUS?` | `WiFi Status: Connected` |
| AT+WIFI_SCAN | `AT+WIFI_SCAN` | None | Scan for WiFi networks | `AT+WIFI_SCAN` | Network list |
| AT+WIFI? | `AT+WIFI?` | None | Get current WiFi info | `AT+WIFI?` | `Mode:STA,SSID:MyNetwork,RSSI:-65dBm` |

### WiFi Settings

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+WIFI_MODE? | `AT+WIFI_MODE?` | None | Get WiFi mode | `AT+WIFI_MODE?` | `1` (0=OFF, 1=STA, 2=AP, 3=AP+STA) |
| AT+WIFI_MODE= | `AT+WIFI_MODE=<0-3>` | 0-3: WiFi mode | Set WiFi mode | `AT+WIFI_MODE=1` | `OK` |
| AT+WIFI_POWER? | `AT+WIFI_POWER?` | None | Get WiFi power | `AT+WIFI_POWER?` | `20` (dBm) |
| AT+WIFI_POWER= | `AT+WIFI_POWER=<value>` | value: power dBm | Set WiFi power | `AT+WIFI_POWER=20` | `OK` |

### WiFi AP Settings

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+WIFI_AP_CH? | `AT+WIFI_AP_CH?` | None | Get AP channel | `AT+WIFI_AP_CH?` | `1` |
| AT+WIFI_AP_CH= | `AT+WIFI_AP_CH=<1-13>` | 1-13: channel | Set AP channel | `AT+WIFI_AP_CH=6` | `OK` |
| AT+WIFI_AP_SSID? | `AT+WIFI_AP_SSID?` | None | Get AP SSID | `AT+WIFI_AP_SSID?` | `ESP32APRS` |
| AT+WIFI_AP_SSID= | `AT+WIFI_AP_SSID="<ssid>"` | ssid: string | Set AP SSID | `AT+WIFI_AP_SSID="MyAP"` | `OK` |
| AT+WIFI_AP_PASS? | `AT+WIFI_AP_PASS?` | None | Get AP password | `AT+WIFI_AP_PASS?` | `12345678` |
| AT+WIFI_AP_PASS= | `AT+WIFI_AP_PASS="<pass>"` | pass: password | Set AP password | `AT+WIFI_AP_PASS="mypass"` | `OK` |

### WiFi STA Profiles (5 profiles: 0-4)

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+WIFI[n]EN? | `AT+WIFI[n]EN?` | n: 0-4 | Get profile status | `AT+WIFI0EN?` | `1` or `0` |
| AT+WIFI[n]EN= | `AT+WIFI[n]EN=<0\|1>` | 0/1: disable/enable | Enable/disable profile | `AT+WIFI0EN=1` | `OK` |
| AT+WIFI[n]SSID? | `AT+WIFI[n]SSID?` | n: 0-4 | Get profile SSID | `AT+WIFI0SSID?` | `MyNetwork` |
| AT+WIFI[n]SSID= | `AT+WIFI[n]SSID="<ssid>"` | n: 0-4, ssid: string | Set profile SSID | `AT+WIFI0SSID="Home"` | `OK` |
| AT+WIFI[n]PASS? | `AT+WIFI[n]PASS?` | n: 0-4 | Get profile password | `AT+WIFI0PASS?` | `secret123` |
| AT+WIFI[n]PASS= | `AT+WIFI[n]PASS="<pass>"` | n: 0-4, pass: string | Set profile password | `AT+WIFI0PASS="mypass"` | `OK` |

> **Note:** Replace `[n]` with 0, 1, 2, 3, or 4 for each WiFi STA profile

---

## Time Commands

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+TIME? | `AT+TIME?` | None | Get current time | `AT+TIME?` | `2024-02-23 14:30:00` |
| AT+TIME= | `AT+TIME="YYYY-MM-DD HH:MM:SS"` | datetime string | Set time | `AT+TIME="2024-02-23 14:30:00"` | `Set Time: 2024-02-23 14:30:00` |
| AT+TIMEZONE? | `AT+TIMEZONE?` | None | Get timezone | `AT+TIMEZONE?` | `7.000000` |
| AT+TIMEZONE= | `AT+TIMEZONE=<value>` | value: hours (float) | Set timezone | `AT+TIMEZONE=7.0` | `OK` |
| AT+SYNCTIME? | `AT+SYNCTIME?` | None | Get NTP sync status | `AT+SYNCTIME?` | `1` or `0` |
| AT+SYNCTIME= | `AT+SYNCTIME=<0\|1>` | 0/1: disable/enable | Enable/disable NTP | `AT+SYNCTIME=1` | `OK` |

---

## Bluetooth Commands

> **Note:** These commands are available only when BLUETOOTH feature is enabled

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+BT_SLAVE? | `AT+BT_SLAVE?` | None | Get Slave status | `AT+BT_SLAVE?` | `1` or `0` |
| AT+BT_SLAVE= | `AT+BT_SLAVE=<0\|1>` | 0/1: disable/enable | Enable/disable Slave | `AT+BT_SLAVE=1` | `OK` |
| AT+BT_MASTER? | `AT+BT_MASTER?` | None | Get Master status | `AT+BT_MASTER?` | `1` or `0` |
| AT+BT_MASTER= | `AT+BT_MASTER=<0\|1>` | 0/1: disable/enable | Enable/disable Master | `AT+BT_MASTER=1` | `OK` |
| AT+BT_MODE? | `AT+BT_MODE?` | None | Get Bluetooth mode | `AT+BT_MODE?` | `0` |
| AT+BT_MODE= | `AT+BT_MODE=<value>` | value: mode | Set Bluetooth mode | `AT+BT_MODE=0` | `OK` |
| AT+BT_NAME? | `AT+BT_NAME?` | None | Get device name | `AT+BT_NAME?` | `ESP32APRS` |
| AT+BT_NAME= | `AT+BT_NAME="<name>"` | name: string | Set device name | `AT+BT_NAME="MyAPRS"` | `OK` |
| AT+BT_PIN? | `AT+BT_PIN?` | None | Get PIN | `AT+BT_PIN?` | `1234` |
| AT+BT_PIN= | `AT+BT_PIN=<value>` | value: PIN | Set PIN | `AT+BT_PIN=1234` | `OK` |
| AT+BT_POWER? | `AT+BT_POWER?` | None | Get power | `AT+BT_POWER?` | `0` |
| AT+BT_POWER= | `AT+BT_POWER=<value>` | value: power | Set power | `AT+BT_POWER=0` | `OK` |
| AT+BT_UUID? | `AT+BT_UUID?` | None | Get UUID (ESP32-S3/C3) | `AT+BT_UUID?` | `00001801-...` |
| AT+BT_UUID= | `AT+BT_UUID="<uuid>"` | uuid: string | Set UUID | `AT+BT_UUID="00001801..."` | `OK` |
| AT+BT_UUID_RX? | `AT+BT_UUID_RX?` | None | Get RX UUID | `AT+BT_UUID_RX?` | (UUID) |
| AT+BT_UUID_RX= | `AT+BT_UUID_RX="<uuid>"` | uuid: string | Set RX UUID | `AT+BT_UUID_RX="..."` | `OK` |
| AT+BT_UUID_TX? | `AT+BT_UUID_TX?` | None | Get TX UUID | `AT+BT_UUID_TX?` | (UUID) |
| AT+BT_UUID_TX= | `AT+BT_UUID_TX="<uuid>"` | uuid: string | Set TX UUID | `AT+BT_UUID_TX="..."` | `OK` |

---

## RF/LoRa Commands

### Main RF Settings

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+RF_EN? | `AT+RF_EN?` | None | Get RF status | `AT+RF_EN?` | `1` or `0` |
| AT+RF_EN= | `AT+RF_EN=<0\|1>` | 0/1: disable/enable | Enable/disable RF | `AT+RF_EN=1` | `OK` |
| AT+RF_TYPE? | `AT+RF_TYPE?` | None | Get RF type | `AT+RF_TYPE?` | `0` |
| AT+RF_TYPE= | `AT+RF_TYPE=<value>` | value: type | Set RF type | `AT+RF_TYPE=0` | `OK` |
| AT+RF_MODE? | `AT+RF_MODE?` | None | Get RF mode | `AT+RF_MODE?` | `0` |
| AT+RF_MODE= | `AT+RF_MODE=<value>` | value: mode | Set RF mode | `AT+RF_MODE=0` | `OK` |
| AT+RF_FREQ? | `AT+RF_FREQ?` | None | Get frequency (MHz) | `AT+RF_FREQ?` | `144.39000` |
| AT+RF_FREQ= | `AT+RF_FREQ=<value>` | value: MHz | Set frequency | `AT+RF_FREQ=144.390` | `OK` |
| AT+RF_FREQ_OFFSET? | `AT+RF_FREQ_OFFSET?` | None | Get frequency offset | `AT+RF_FREQ_OFFSET?` | `0` |
| AT+RF_FREQ_OFFSET= | `AT+RF_FREQ_OFFSET=<value>` | value: Hz | Set frequency offset | `AT+RF_FREQ_OFFSET=0` | `OK` |
| AT+RF_BW? | `AT+RF_BW?` | None | Get Bandwidth (kHz) | `AT+RF_BW?` | `125.00` |
| AT+RF_BW= | `AT+RF_BW=<value>` | value: kHz | Set Bandwidth | `AT+RF_BW=125.0` | `OK` |
| AT+RF_BR? | `AT+RF_BR?` | None | Get Bit Rate | `AT+RF_BR?` | `1200.00` |
| AT+RF_BR= | `AT+RF_BR=<value>` | value: bit rate | Set Bit Rate | `AT+RF_BR=1200.0` | `OK` |
| AT+RF_SF? | `AT+RF_SF?` | None | Get Spreading Factor | `AT+RF_SF?` | `7` |
| AT+RF_SF= | `AT+RF_SF=<7-12>` | 7-12: SF | Set SF | `AT+RF_SF=7` | `OK` |
| AT+RF_CR? | `AT+RF_CR?` | None | Get Coding Rate | `AT+RF_CR?` | `5` |
| AT+RF_CR= | `AT+RF_CR=<value>` | value: CR | Set Coding Rate | `AT+RF_CR=5` | `OK` |
| AT+RF_SYNC? | `AT+RF_SYNC?` | None | Get Sync Word | `AT+RF_SYNC?` | `0x12` |
| AT+RF_SYNC= | `AT+RF_SYNC=<value>` | value: sync word | Set Sync Word | `AT+RF_SYNC=0x12` | `OK` |
| AT+RF_POWER? | `AT+RF_POWER?` | None | Get power | `AT+RF_POWER?` | `17` |
| AT+RF_POWER= | `AT+RF_POWER=<value>` | value: dBm | Set power | `AT+RF_POWER=17` | `OK` |
| AT+RF_PREAMABLE? | `AT+RF_PREAMABLE?` | None | Get Preamble Length | `AT+RF_PREAMABLE?` | `8` |
| AT+RF_PREAMABLE= | `AT+RF_PREAMABLE=<value>` | value: length | Set Preamble | `AT+RF_PREAMABLE=8` | `OK` |
| AT+RF_LNA? | `AT+RF_LNA?` | None | Get LNA Gain | `AT+RF_LNA?` | `0` |
| AT+RF_LNA= | `AT+RF_LNA=<value>` | value: gain | Set LNA Gain | `AT+RF_LNA=0` | `OK` |
| AT+RF_AX25? | `AT+RF_AX25?` | None | Get AX.25 mode | `AT+RF_AX25?` | `1` or `0` |
| AT+RF_AX25= | `AT+RF_AX25=<0\|1>` | 0/1: disable/enable | Enable/disable AX.25 | `AT+RF_AX25=1` | `OK` |
| AT+RF_SHAPING? | `AT+RF_SHAPING?` | None | Get Shaping | `AT+RF_SHAPING?` | `0` |
| AT+RF_SHAPING= | `AT+RF_SHAPING=<value>` | value: type | Set Shaping | `AT+RF_SHAPING=0` | `OK` |
| AT+RF_ENCODING? | `AT+RF_ENCODING?` | None | Get Encoding | `AT+RF_ENCODING?` | `0` |
| AT+RF_ENCODING= | `AT+RF_ENCODING=<value>` | value: type | Set Encoding | `AT+RF_ENCODING=0` | `OK` |
| AT+RF_RX_BOOST? | `AT+RF_RX_BOOST?` | None | Get RX Boost | `AT+RF_RX_BOOST?` | `1` or `0` |
| AT+RF_RX_BOOST= | `AT+RF_RX_BOOST=<0\|1>` | 0/1: disable/enable | Enable/disable RX Boost | `AT+RF_RX_BOOST=1` | `OK` |

### RF GPIO Pins

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+RF_TX_GPIO? | `AT+RF_TX_GPIO?` | None | Get TX GPIO | `AT+RF_TX_GPIO?` | `4` |
| AT+RF_TX_GPIO= | `AT+RF_TX_GPIO=<value>` | value: GPIO | Set TX GPIO | `AT+RF_TX_GPIO=4` | `OK` |
| AT+RF_RX_GPIO? | `AT+RF_RX_GPIO?` | None | Get RX GPIO | `AT+RF_RX_GPIO?` | `5` |
| AT+RF_RX_GPIO= | `AT+RF_RX_GPIO=<value>` | value: GPIO | Set RX GPIO | `AT+RF_RX_GPIO=5` | `OK` |
| AT+RF_DIO1_GPIO? | `AT+RF_DIO1_GPIO?` | None | Get DIO1 GPIO | `AT+RF_DIO1_GPIO?` | `33` |
| AT+RF_DIO1_GPIO= | `AT+RF_DIO1_GPIO=<value>` | value: GPIO | Set DIO1 GPIO | `AT+RF_DIO1_GPIO=33` | `OK` |
| AT+RF_RESET_GPIO? | `AT+RF_RESET_GPIO?` | None | Get Reset GPIO | `AT+RF_RESET_GPIO?` | `14` |
| AT+RF_RESET_GPIO= | `AT+RF_RESET_GPIO=<value>` | value: GPIO | Set Reset GPIO | `AT+RF_RESET_GPIO=14` | `OK` |
| AT+RF_DIO0_GPIO? | `AT+RF_DIO0_GPIO?` | None | Get DIO0 GPIO | `AT+RF_DIO0_GPIO?` | `2` |
| AT+RF_DIO0_GPIO= | `AT+RF_DIO0_GPIO=<value>` | value: GPIO | Set DIO0 GPIO | `AT+RF_DIO0_GPIO=2` | `OK` |
| AT+RF_DIO2_GPIO? | `AT+RF_DIO2_GPIO?` | None | Get DIO2 GPIO | `AT+RF_DIO2_GPIO?` | `0` |
| AT+RF_DIO2_GPIO= | `AT+RF_DIO2_GPIO=<value>` | value: GPIO | Set DIO2 GPIO | `AT+RF_DIO2_GPIO=0` | `OK` |
| AT+RF_NSS_GPIO? | `AT+RF_NSS_GPIO?` | None | Get NSS GPIO | `AT+RF_NSS_GPIO?` | `5` |
| AT+RF_NSS_GPIO= | `AT+RF_NSS_GPIO=<value>` | value: GPIO | Set NSS GPIO | `AT+RF_NSS_GPIO=5` | `OK` |
| AT+RF_SCLK_GPIO? | `AT+RF_SCLK_GPIO?` | None | Get SCLK GPIO | `AT+RF_SCLK_GPIO?` | `18` |
| AT+RF_SCLK_GPIO= | `AT+RF_SCLK_GPIO=<value>` | value: GPIO | Set SCLK GPIO | `AT+RF_SCLK_GPIO=18` | `OK` |
| AT+RF_MISO_GPIO? | `AT+RF_MISO_GPIO?` | None | Get MISO GPIO | `AT+RF_MISO_GPIO?` | `19` |
| AT+RF_MISO_GPIO= | `AT+RF_MISO_GPIO=<value>` | value: GPIO | Set MISO GPIO | `AT+RF_MISO_GPIO=19` | `OK` |
| AT+RF_MOSI_GPIO? | `AT+RF_MOSI_GPIO?` | None | Get MOSI GPIO | `AT+RF_MOSI_GPIO?` | `23` |
| AT+RF_MOSI_GPIO= | `AT+RF_MOSI_GPIO=<value>` | value: GPIO | Set MOSI GPIO | `AT+RF_MOSI_GPIO=23` | `OK` |
| AT+RF_TX_ACTIVE? | `AT+RF_TX_ACTIVE?` | None | Get TX Active State | `AT+RF_TX_ACTIVE?` | `1` or `0` |
| AT+RF_TX_ACTIVE= | `AT+RF_TX_ACTIVE=<0\|1>` | 0/1: Low/High | Set TX Active | `AT+RF_TX_ACTIVE=1` | `OK` |
| AT+RF_RX_ACTIVE? | `AT+RF_RX_ACTIVE?` | None | Get RX Active State | `AT+RF_RX_ACTIVE?` | `1` or `0` |
| AT+RF_RX_ACTIVE= | `AT+RF_RX_ACTIVE=<0\|1>` | 0/1: Low/High | Set RX Active | `AT+RF_RX_ACTIVE=1` | `OK` |
| AT+RF_RESET_ACTIVE? | `AT+RF_RESET_ACTIVE?` | None | Get Reset Active | `AT+RF_RESET_ACTIVE?` | `1` or `0` |
| AT+RF_RESET_ACTIVE= | `AT+RF_RESET_ACTIVE=<0\|1>` | 0/1: Low/High | Set Reset Active | `AT+RF_RESET_ACTIVE=1` | `OK` |
| AT+RF_NSS_ACTIVE? | `AT+RF_NSS_ACTIVE?` | None | Get NSS Active | `AT+RF_NSS_ACTIVE?` | `1` or `0` |
| AT+RF_NSS_ACTIVE= | `AT+RF_NSS_ACTIVE=<0\|1>` | 0/1: Low/High | Set NSS Active | `AT+RF_NSS_ACTIVE=1` | `OK` |

> **Note:** RF1 commands (e.g., `AT+RF1_FREQ`, `AT+RF1_GPIO`) are for RF channel 2 (if available)

---

## IGATE Commands

### Basic IGATE Settings

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+IGATE_EN? | `AT+IGATE_EN?` | None | Get IGATE status | `AT+IGATE_EN?` | `1` or `0` |
| AT+IGATE_EN= | `AT+IGATE_EN=<0\|1>` | 0/1: disable/enable | Enable/disable IGATE | `AT+IGATE_EN=1` | `OK` |
| AT+RF2INET? | `AT+RF2INET?` | None | Get RF→INET | `AT+RF2INET?` | `1` or `0` |
| AT+RF2INET= | `AT+RF2INET=<0\|1>` | 0/1: disable/enable | Enable/disable RF→INET | `AT+RF2INET=1` | `OK` |
| AT+INET2RF? | `AT+INET2RF?` | None | Get INET→RF | `AT+INET2RF?` | `1` or `0` |
| AT+INET2RF= | `AT+INET2RF=<0\|1>` | 0/1: disable/enable | Enable/disable INET→RF | `AT+INET2RF=1` | `OK` |
| AT+IGATE_LOC2RF? | `AT+IGATE_LOC2RF?` | None | Get Location→RF | `AT+IGATE_LOC2RF?` | `1` or `0` |
| AT+IGATE_LOC2RF= | `AT+IGATE_LOC2RF=<0\|1>` | 0/1: disable/enable | Enable/disable Loc→RF | `AT+IGATE_LOC2RF=1` | `OK` |
| AT+IGATE_LOC2INET? | `AT+IGATE_LOC2INET?` | None | Get Location→INET | `AT+IGATE_LOC2INET?` | `1` or `0` |
| AT+IGATE_LOC2INET= | `AT+IGATE_LOC2INET=<0\|1>` | 0/1: disable/enable | Enable/disable Loc→INET | `AT+IGATE_LOC2INET=1` | `OK` |

### Server Settings

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+IGATE_MYCALL? | `AT+IGATE_MYCALL?` | None | Get Callsign | `AT+IGATE_MYCALL?` | `MYCALL` |
| AT+IGATE_MYCALL= | `AT+IGATE_MYCALL="<call>"` | call: Callsign | Set Callsign | `AT+IGATE_MYCALL="HS1ABC"` | `OK` |
| AT+IGATE_HOST? | `AT+IGATE_HOST?` | None | Get Server Host | `AT+IGATE_HOST?` | `rotate.aprs2.net` |
| AT+IGATE_HOST= | `AT+IGATE_HOST="<host>"` | host: address | Set Server | `AT+IGATE_HOST="noam.aprs2.net"` | `OK` |
| AT+APRS_PORT? | `AT+APRS_PORT?` | None | Get Port | `AT+APRS_PORT?` | `14580` |
| AT+APRS_PORT= | `AT+APRS_PORT=<value>` | value: Port | Set Port | `AT+APRS_PORT=14580` | `OK` |
| AT+IGATE_SSID? | `AT+IGATE_SSID?` | None | Get SSID | `AT+IGATE_SSID?` | `1` |
| AT+IGATE_SSID= | `AT+IGATE_SSID=<0-15>` | 0-15: SSID | Set SSID | `AT+IGATE_SSID=1` | `OK` |

### Beacon Settings

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+IGATE_LAT? | `AT+IGATE_LAT?` | None | Get Latitude | `AT+IGATE_LAT?` | `13.756331` |
| AT+IGATE_LAT= | `AT+IGATE_LAT=<value>` | value: latitude | Set Latitude | `AT+IGATE_LAT=13.756331` | `OK` |
| AT+IGATE_LON? | `AT+IGATE_LON?` | None | Get Longitude | `AT+IGATE_LON?` | `100.501765` |
| AT+IGATE_LON= | `AT+IGATE_LON=<value>` | value: longitude | Set Longitude | `AT+IGATE_LON=100.501765` | `OK` |
| AT+IGATE_ALT? | `AT+IGATE_ALT?` | None | Get Altitude | `AT+IGATE_ALT?` | `10.5` |
| AT+IGATE_ALT= | `AT+IGATE_ALT=<value>` | value: meters | Set Altitude | `AT+IGATE_ALT=10.5` | `OK` |
| AT+IGATE_INTERVAL? | `AT+IGATE_INTERVAL?` | None | Get Beacon Interval | `AT+IGATE_INTERVAL?` | `600` (seconds) |
| AT+IGATE_INTERVAL= | `AT+IGATE_INTERVAL=<value>` | value: seconds | Set Beacon Interval | `AT+IGATE_INTERVAL=600` | `OK` |
| AT+IGATE_SYMBOL? | `AT+IGATE_SYMBOL?` | None | Get APRS Symbol | `AT+IGATE_SYMBOL?` | `/I` |
| AT+IGATE_SYMBOL= | `AT+IGATE_SYMBOL="<sym>"` | sym: symbol | Set Symbol | `AT+IGATE_SYMBOL="/I"` | `OK` |
| AT+IGATE_COMMENT? | `AT+IGATE_COMMENT?` | None | Get Comment | `AT+IGATE_COMMENT?` | `ESP32 APRS` |
| AT+IGATE_COMMENT= | `AT+IGATE_COMMENT="<text>"` | text: comment | Set Comment | `AT+IGATE_COMMENT="ESP32 APRS"` | `OK` |

### Filter and Others

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+IGATE_FILTER? | `AT+IGATE_FILTER?` | None | Get Filter | `AT+IGATE_FILTER?` | `m/100` |
| AT+IGATE_FILTER= | `AT+IGATE_FILTER="<filter>"` | filter: string | Set Filter | `AT+IGATE_FILTER="m/100"` | `OK` |
| AT+IGATE_MONICALL? | `AT+IGATE_MONICALL?` | None | Get Monitor Callsign | `AT+IGATE_MONICALL?` | `*` |
| AT+IGATE_MONICALL= | `AT+IGATE_MONICALL="<call>"` | call: Callsign | Set Monitor | `AT+IGATE_MONICALL="*"` | `OK` |
| AT+IGATE_BCN? | `AT+IGATE_BCN?` | None | Get Beacon | `AT+IGATE_BCN?` | `1` or `0` |
| AT+IGATE_BCN= | `AT+IGATE_BCN=<0\|1>` | 0/1: disable/enable | Enable/disable Beacon | `AT+IGATE_BCN=1` | `OK` |
| AT+IGATE_GPS? | `AT+IGATE_GPS?` | None | Get GPS | `AT+IGATE_GPS?` | `1` or `0` |
| AT+IGATE_GPS= | `AT+IGATE_GPS=<0\|1>` | 0/1: disable/enable | Enable/disable GPS | `AT+IGATE_GPS=1` | `OK` |
| AT+IGATE_TIMESTAMP? | `AT+IGATE_TIMESTAMP?` | None | Get Timestamp | `AT+IGATE_TIMESTAMP?` | `1` or `0` |
| AT+IGATE_TIMESTAMP= | `AT+IGATE_TIMESTAMP=<0\|1>` | 0/1: disable/enable | Enable/disable Timestamp | `AT+IGATE_TIMESTAMP=1` | `OK` |

---

## DIGI Commands

### Basic DIGI Settings

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+DIGI_EN? | `AT+DIGI_EN?` | None | Get DIGI status | `AT+DIGI_EN?` | `1` or `0` |
| AT+DIGI_EN= | `AT+DIGI_EN=<0\|1>` | 0/1: disable/enable | Enable/disable DIGI | `AT+DIGI_EN=1` | `OK` |
| AT+DIGI_AUTO? | `AT+DIGI_AUTO?` | None | Get Auto Mode | `AT+DIGI_AUTO?` | `1` or `0` |
| AT+DIGI_AUTO= | `AT+DIGI_AUTO=<0\|1>` | 0/1: disable/enable | Enable/disable Auto | `AT+DIGI_AUTO=1` | `OK` |
| AT+DIGI_LOC2RF? | `AT+DIGI_LOC2RF?` | None | Get Location→RF | `AT+DIGI_LOC2RF?` | `1` or `0` |
| AT+DIGI_LOC2RF= | `AT+DIGI_LOC2RF=<0\|1>` | 0/1: disable/enable | Enable/disable Loc→RF | `AT+DIGI_LOC2RF=1` | `OK` |
| AT+DIGI_LOC2INET? | `AT+DIGI_LOC2INET?` | None | Get Location→INET | `AT+DIGI_LOC2INET?` | `1` or `0` |
| AT+DIGI_LOC2INET= | `AT+DIGI_LOC2INET=<0\|1>` | 0/1: disable/enable | Enable/disable Loc→INET | `AT+DIGI_LOC2INET=1` | `OK` |

### DIGI Beacon Settings

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+DIGI_MYCALL? | `AT+DIGI_MYCALL?` | None | Get Callsign | `AT+DIGI_MYCALL?` | `MYCALL` |
| AT+DIGI_MYCALL= | `AT+DIGI_MYCALL="<call>"` | call: Callsign | Set Callsign | `AT+DIGI_MYCALL="HS1ABC"` | `OK` |
| AT+DIGI_SSID? | `AT+DIGI_SSID?` | None | Get SSID | `AT+DIGI_SSID?` | `1` |
| AT+DIGI_SSID= | `AT+DIGI_SSID=<0-15>` | 0-15: SSID | Set SSID | `AT+DIGI_SSID=1` | `OK` |
| AT+DIGI_PATH? | `AT+DIGI_PATH?` | None | Get Path | `AT+DIGI_PATH?` | `0` |
| AT+DIGI_PATH= | `AT+DIGI_PATH=<value>` | value: type | Set Path | `AT+DIGI_PATH=0` | `OK` |
| AT+DIGI_DELAY? | `AT+DIGI_DELAY?` | None | Get Delay | `AT+DIGI_DELAY?` | `0` (ms) |
| AT+DIGI_DELAY= | `AT+DIGI_DELAY=<value>` | value: ms | Set Delay | `AT+DIGI_DELAY=0` | `OK` |
| AT+DIGI_INTERVAL? | `AT+DIGI_INTERVAL?` | None | Get Beacon Interval | `AT+DIGI_INTERVAL?` | `600` (seconds) |
| AT+DIGI_INTERVAL= | `AT+DIGI_INTERVAL=<value>` | value: seconds | Set Beacon Interval | `AT+DIGI_INTERVAL=600` | `OK` |
| AT+DIGI_SYMBOL? | `AT+DIGI_SYMBOL?` | None | Get Symbol | `AT+DIGI_SYMBOL?` | `/D` |
| AT+DIGI_SYMBOL= | `AT+DIGI_SYMBOL="<sym>"` | sym: symbol | Set Symbol | `AT+DIGI_SYMBOL="/D"` | `OK` |
| AT+DIGI_COMMENT? | `AT+DIGI_COMMENT?` | None | Get Comment | `AT+DIGI_COMMENT?` | `DIGI` |
| AT+DIGI_COMMENT= | `AT+DIGI_COMMENT="<text>"` | text: comment | Set Comment | `AT+DIGI_COMMENT="DIGI"` | `OK` |

### Filter and Others

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+DIGIFILTER? | `AT+DIGIFILTER?` | None | Get Filter | `AT+DIGIFILTER?` | `0` |
| AT+DIGIFILTER= | `AT+DIGIFILTER=<value>` | value: type | Set Filter | `AT+DIGIFILTER=0` | `OK` |
| AT+DIGI_BCN? | `AT+DIGI_BCN?` | None | Get Beacon | `AT+DIGI_BCN?` | `1` or `0` |
| AT+DIGI_BCN= | `AT+DIGI_BCN=<0\|1>` | 0/1: disable/enable | Enable/disable Beacon | `AT+DIGI_BCN=1` | `OK` |
| AT+DIGI_GPS? | `AT+DIGI_GPS?` | None | Get GPS | `AT+DIGI_GPS?` | `1` or `0` |
| AT+DIGI_GPS= | `AT+DIGI_GPS=<0\|1>` | 0/1: disable/enable | Enable/disable GPS | `AT+DIGI_GPS=1` | `OK` |
| AT+DIGI_TIMESTAMP? | `AT+DIGI_TIMESTAMP?` | None | Get Timestamp | `AT+DIGI_TIMESTAMP?` | `1` or `0` |
| AT+DIGI_TIMESTAMP= | `AT+DIGI_TIMESTAMP=<0\|1>` | 0/1: disable/enable | Enable/disable Timestamp | `AT+DIGI_TIMESTAMP=1` | `OK` |

---

## TRACKER Commands

### Basic Tracker Settings

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+TRK_EN? | `AT+TRK_EN?` | None | Get Tracker status | `AT+TRK_EN?` | `1` or `0` |
| AT+TRK_EN= | `AT+TRK_EN=<0\|1>` | 0/1: disable/enable | Enable/disable Tracker | `AT+TRK_EN=1` | `OK` |
| AT+TRK_LOC2RF? | `AT+TRK_LOC2RF?` | None | Get Location→RF | `AT+TRK_LOC2RF?` | `1` or `0` |
| AT+TRK_LOC2RF= | `AT+TRK_LOC2RF=<0\|1>` | 0/1: disable/enable | Enable/disable Loc→RF | `AT+TRK_LOC2RF=1` | `OK` |
| AT+TRK_LOC2INET? | `AT+TRK_LOC2INET?` | None | Get Location→INET | `AT+TRK_LOC2INET?` | `1` or `0` |
| AT+TRK_LOC2INET= | `AT+TRK_LOC2INET=<0\|1>` | 0/1: disable/enable | Enable/disable Loc→INET | `AT+TRK_LOC2INET=1` | `OK` |

### Tracker Beacon Settings

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+TRK_MYCALL? | `AT+TRK_MYCALL?` | None | Get Callsign | `AT+TRK_MYCALL?` | `MYCALL` |
| AT+TRK_MYCALL= | `AT+TRK_MYCALL="<call>"` | call: Callsign | Set Callsign | `AT+TRK_MYCALL="HS1ABC"` | `OK` |
| AT+TRK_SSID? | `AT+TRK_SSID?` | None | Get SSID | `AT+TRK_SSID?` | `9` |
| AT+TRK_SSID= | `AT+TRK_SSID=<0-15>` | 0-15: SSID | Set SSID | `AT+TRK_SSID=9` | `OK` |
| AT+TRK_PATH? | `AT+TRK_PATH?` | None | Get Path | `AT+TRK_PATH?` | `0` |
| AT+TRK_PATH= | `AT+TRK_PATH=<value>` | value: type | Set Path | `AT+TRK_PATH=0` | `OK` |
| AT+TRK_INTERVAL? | `AT+TRK_INTERVAL?` | None | Get Beacon Interval | `AT+TRK_INTERVAL?` | `60` (seconds) |
| AT+TRK_INTERVAL= | `AT+TRK_INTERVAL=<value>` | value: seconds | Set Beacon Interval | `AT+TRK_INTERVAL=60` | `OK` |
| AT+TRK_SYMBOL? | `AT+TRK_SYMBOL?` | None | Get Symbol | `AT+TRK_SYMBOL?` | `/>` |
| AT+TRK_SYMBOL= | `AT+TRK_SYMBOL="<sym>"` | sym: symbol | Set Symbol | `AT+TRK_SYMBOL="/>"` | `OK` |
| AT+TRK_COMMENT? | `AT+TRK_COMMENT?` | None | Get Comment | `AT+TRK_COMMENT?` | `Mobile` |
| AT+TRK_COMMENT= | `AT+TRK_COMMENT="<text>"` | text: comment | Set Comment | `AT+TRK_COMMENT="Mobile"` | `OK` |

### SmartBeacon

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+TRK_SMARTBEACON? | `AT+TRK_SMARTBEACON?` | None | Get SmartBeacon | `AT+TRK_SMARTBEACON?` | `1` or `0` |
| AT+TRK_SMARTBEACON= | `AT+TRK_SMARTBEACON=<0\|1>` | 0/1: disable/enable | Enable/disable SmartBeacon | `AT+TRK_SMARTBEACON=1` | `OK` |
| AT+TRK_HSPEED? | `AT+TRK_HSPEED?` | None | Get High Speed | `AT+TRK_HSPEED?` | `90` (km/h) |
| AT+TRK_HSPEED= | `AT+TRK_HSPEED=<value>` | value: km/h | Set High Speed | `AT+TRK_HSPEED=90` | `OK` |
| AT+TRK_LSPEED? | `AT+TRK_LSPEED?` | None | Get Low Speed | `AT+TRK_LSPEED?` | `5` (km/h) |
| AT+TRK_LSPEED= | `AT+TRK_LSPEED=<value>` | value: km/h | Set Low Speed | `AT+TRK_LSPEED=5` | `OK` |
| AT+TRK_MAXINTERVAL? | `AT+TRK_MAXINTERVAL?` | None | Get Max Interval | `AT+TRK_MAXINTERVAL?` | `180` (seconds) |
| AT+TRK_MAXINTERVAL= | `AT+TRK_MAXINTERVAL=<value>` | value: seconds | Set Max Interval | `AT+TRK_MAXINTERVAL=180` | `OK` |
| AT+TRK_MININTERVAL? | `AT+TRK_MININTERVAL?` | None | Get Min Interval | `AT+TRK_MININTERVAL?` | `10` (seconds) |
| AT+TRK_MININTERVAL= | `AT+TRK_MININTERVAL=<value>` | value: seconds | Set Min Interval | `AT+TRK_MININTERVAL=10` | `OK` |
| AT+TRK_MINANGLE? | `AT+TRK_MINANGLE?` | None | Get Min Angle | `AT+TRK_MINANGLE?` | `30` (degrees) |
| AT+TRK_MINANGLE= | `AT+TRK_MINANGLE=<value>` | value: degrees | Set Min Angle | `AT+TRK_MINANGLE=30` | `OK` |
| AT+TRK_SLOWINTERVAL? | `AT+TRK_SLOWINTERVAL?` | None | Get Slow Interval | `AT+TRK_SLOWINTERVAL?` | `120` (seconds) |
| AT+TRK_SLOWINTERVAL= | `AT+TRK_SLOWINTERVAL=<value>` | value: seconds | Set Slow Interval | `AT+TRK_SLOWINTERVAL=120` | `OK` |

### Additional Options

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+TRK_COMPRESS? | `AT+TRK_COMPRESS?` | None | Get Compression | `AT+TRK_COMPRESS?` | `1` or `0` |
| AT+TRK_COMPRESS= | `AT+TRK_COMPRESS=<0\|1>` | 0/1: disable/enable | Enable/disable Compression | `AT+TRK_COMPRESS=1` | `OK` |
| AT+TRK_ALTITUDE? | `AT+TRK_ALTITUDE?` | None | Get Altitude Report | `AT+TRK_ALTITUDE?` | `1` or `0` |
| AT+TRK_ALTITUDE= | `AT+TRK_ALTITUDE=<0\|1>` | 0/1: disable/enable | Enable/disable Altitude | `AT+TRK_ALTITUDE=1` | `OK` |
| AT+TRK_LOG? | `AT+TRK_LOG?` | None | Get Logging | `AT+TRK_LOG?` | `1` or `0` |
| AT+TRK_LOG= | `AT+TRK_LOG=<0\|1>` | 0/1: disable/enable | Enable/disable Logging | `AT+TRK_LOG=1` | `OK` |
| AT+TRK_RSSI? | `AT+TRK_RSSI?` | None | Get RSSI Report | `AT+TRK_RSSI?` | `1` or `0` |
| AT+TRK_RSSI= | `AT+TRK_RSSI=<0\|1>` | 0/1: disable/enable | Enable/disable RSSI | `AT+TRK_RSSI=1` | `OK` |
| AT+TRK_SAT? | `AT+TRK_SAT?` | None | Get Satellite Report | `AT+TRK_SAT?` | `1` or `0` |
| AT+TRK_SAT= | `AT+TRK_SAT=<0\|1>` | 0/1: disable/enable | Enable/disable Satellite | `AT+TRK_SAT=1` | `OK` |
| AT+TRK_DX? | `AT+TRK_DX?` | None | Get DX Report | `AT+TRK_DX?` | `1` or `0` |
| AT+TRK_DX= | `AT+TRK_DX=<0\|1>` | 0/1: disable/enable | Enable/disable DX | `AT+TRK_DX=1` | `OK` |
| AT+TRK_GPS? | `AT+TRK_GPS?` | None | Get GPS | `AT+TRK_GPS?` | `1` or `0` |
| AT+TRK_GPS= | `AT+TRK_GPS=<0\|1>` | 0/1: disable/enable | Enable/disable GPS | `AT+TRK_GPS=1` | `OK` |
| AT+TRK_TIMESTAMP? | `AT+TRK_TIMESTAMP?` | None | Get Timestamp | `AT+TRK_TIMESTAMP?` | `1` or `0` |
| AT+TRK_TIMESTAMP= | `AT+TRK_TIMESTAMP=<0\|1>` | 0/1: disable/enable | Enable/disable Timestamp | `AT+TRK_TIMESTAMP=1` | `OK` |

---

## WEATHER Commands

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+WX_EN? | `AT+WX_EN?` | None | Get Weather status | `AT+WX_EN?` | `1` or `0` |
| AT+WX_EN= | `AT+WX_EN=<0\|1>` | 0/1: disable/enable | Enable/disable Weather | `AT+WX_EN=1` | `OK` |
| AT+WX_2RF? | `AT+WX_2RF?` | None | Get WX→RF | `AT+WX_2RF?` | `1` or `0` |
| AT+WX_2RF= | `AT+WX_2RF=<0\|1>` | 0/1: disable/enable | Enable/disable WX→RF | `AT+WX_2RF=1` | `OK` |
| AT+WX_2INET? | `AT+WX_2INET?` | None | Get WX→INET | `AT+WX_2INET?` | `1` or `0` |
| AT+WX_2INET= | `AT+WX_2INET=<0\|1>` | 0/1: disable/enable | Enable/disable WX→INET | `AT+WX_2INET=1` | `OK` |
| AT+WX_MYCALL? | `AT+WX_MYCALL?` | None | Get Callsign | `AT+WX_MYCALL?` | `MYCALL` |
| AT+WX_MYCALL= | `AT+WX_MYCALL="<call>"` | call: Callsign | Set Callsign | `AT+WX_MYCALL="HS1ABC"` | `OK` |
| AT+WX_SSID? | `AT+WX_SSID?` | None | Get SSID | `AT+WX_SSID?` | `0` |
| AT+WX_SSID= | `AT+WX_SSID=<0-15>` | 0-15: SSID | Set SSID | `AT+WX_SSID=0` | `OK` |
| AT+WX_PATH? | `AT+WX_PATH?` | None | Get Path | `AT+WX_PATH?` | `0` |
| AT+WX_PATH= | `AT+WX_PATH=<value>` | value: type | Set Path | `AT+WX_PATH=0` | `OK` |
| AT+WX_INTERVAL? | `AT+WX_INTERVAL?` | None | Get Beacon Interval | `AT+WX_INTERVAL?` | `300` (seconds) |
| AT+WX_INTERVAL= | `AT+WX_INTERVAL=<value>` | value: seconds | Set Beacon Interval | `AT+WX_INTERVAL=300` | `OK` |
| AT+WX_OBJECT? | `AT+WX_OBJECT?` | None | Get Object Name | `AT+WX_OBJECT?` | `WX` |
| AT+WX_OBJECT= | `AT+WX_OBJECT="<name>"` | name: string | Set Object | `AT+WX_OBJECT="WX"` | `OK` |
| AT+WX_COMMENT? | `AT+WX_COMMENT?` | None | Get Comment | `AT+WX_COMMENT?` | `WX Station` |
| AT+WX_COMMENT= | `AT+WX_COMMENT="<text>"` | text: comment | Set Comment | `AT+WX_COMMENT="WX Station"` | `OK` |
| AT+WX_GPS? | `AT+WX_GPS?` | None | Get GPS | `AT+WX_GPS?` | `1` or `0` |
| AT+WX_GPS= | `AT+WX_GPS=<0\|1>` | 0/1: disable/enable | Enable/disable GPS | `AT+WX_GPS=1` | `OK` |
| AT+WX_LAT? | `AT+WX_LAT?` | None | Get Latitude | `AT+WX_LAT?` | `13.756331` |
| AT+WX_LAT= | `AT+WX_LAT=<value>` | value: latitude | Set Latitude | `AT+WX_LAT=13.756331` | `OK` |
| AT+WX_LON? | `AT+WX_LON?` | None | Get Longitude | `AT+WX_LON?` | `100.501765` |
| AT+WX_LON= | `AT+WX_LON=<value>` | value: longitude | Set Longitude | `AT+WX_LON=100.501765` | `OK` |
| AT+WX_ALT? | `AT+WX_ALT?` | None | Get Altitude | `AT+WX_ALT?` | `10.5` |
| AT+WX_ALT= | `AT+WX_ALT=<value>` | value: meters | Set Altitude | `AT+WX_ALT=10.5` | `OK` |
| AT+WX_TIMESTAMP? | `AT+WX_TIMESTAMP?` | None | Get Timestamp | `AT+WX_TIMESTAMP?` | `1` or `0` |
| AT+WX_TIMESTAMP= | `AT+WX_TIMESTAMP=<0\|1>` | 0/1: disable/enable | Enable/disable Timestamp | `AT+WX_TIMESTAMP=1` | `OK` |

---

## TELEMETRY Commands

### Telemetry 0 (TLM0)

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+TLM0_EN? | `AT+TLM0_EN?` | None | Get TLM0 status | `AT+TLM0_EN?` | `1` or `0` |
| AT+TLM0_EN= | `AT+TLM0_EN=<0\|1>` | 0/1: disable/enable | Enable/disable TLM0 | `AT+TLM0_EN=1` | `OK` |
| AT+TLM0_2RF? | `AT+TLM0_2RF?` | None | Get TLM0→RF | `AT+TLM0_2RF?` | `1` or `0` |
| AT+TLM0_2RF= | `AT+TLM0_2RF=<0\|1>` | 0/1: disable/enable | Enable/disable TLM0→RF | `AT+TLM0_2RF=1` | `OK` |
| AT+TLM0_2INET? | `AT+TLM0_2INET?` | None | Get TLM0→INET | `AT+TLM0_2INET?` | `1` or `0` |
| AT+TLM0_2INET= | `AT+TLM0_2INET=<0\|1>` | 0/1: disable/enable | Enable/disable TLM0→INET | `AT+TLM0_2INET=1` | `OK` |
| AT+TLM0_MYCALL? | `AT+TLM0_MYCALL?` | None | Get Callsign | `AT+TLM0_MYCALL?` | `MYCALL` |
| AT+TLM0_MYCALL= | `AT+TLM0_MYCALL="<call>"` | call: Callsign | Set Callsign | `AT+TLM0_MYCALL="HS1ABC"` | `OK` |
| AT+TLM0_SSID? | `AT+TLM0_SSID?` | None | Get SSID | `AT+TLM0_SSID?` | `0` |
| AT+TLM0_SSID= | `AT+TLM0_SSID=<0-15>` | 0-15: SSID | Set SSID | `AT+TLM0_SSID=0` | `OK` |
| AT+TLM0_PATH? | `AT+TLM0_PATH?` | None | Get Path | `AT+TLM0_PATH?` | `0` |
| AT+TLM0_PATH= | `AT+TLM0_PATH=<value>` | value: type | Set Path | `AT+TLM0_PATH=0` | `OK` |
| AT+TLM0_DATA_INTERVAL? | `AT+TLM0_DATA_INTERVAL?` | None | Get Data Interval | `AT+TLM0_DATA_INTERVAL?` | `60` (seconds) |
| AT+TLM0_DATA_INTERVAL= | `AT+TLM0_DATA_INTERVAL=<value>` | value: seconds | Set Data Interval | `AT+TLM0_DATA_INTERVAL=60` | `OK` |
| AT+TLM0_INFO_INTERVAL? | `AT+TLM0_INFO_INTERVAL?` | None | Get Info Interval | `AT+TLM0_INFO_INTERVAL?` | `300` (seconds) |
| AT+TLM0_INFO_INTERVAL= | `AT+TLM0_INFO_INTERVAL=<value>` | value: seconds | Set Info Interval | `AT+TLM0_INFO_INTERVAL=300` | `OK` |
| AT+TLM0_BITS_ACTIVE? | `AT+TLM0_BITS_ACTIVE?` | None | Get Bits Active | `AT+TLM0_BITS_ACTIVE?` | `0` |
| AT+TLM0_BITS_ACTIVE= | `AT+TLM0_BITS_ACTIVE=<value>` | value: bits | Set Bits | `AT+TLM0_BITS_ACTIVE=0` | `OK` |
| AT+TLM0_COMMENT? | `AT+TLM0_COMMENT?` | None | Get Comment | `AT+TLM0_COMMENT?` | `Telemetry` |
| AT+TLM0_COMMENT= | `AT+TLM0_COMMENT="<text>"` | text: comment | Set Comment | `AT+TLM0_COMMENT="Telemetry"` | `OK` |

---

## OLED/Display Commands

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+OLED_ENABLE? | `AT+OLED_ENABLE?` | None | Get OLED status | `AT+OLED_ENABLE?` | `1` or `0` |
| AT+OLED_ENABLE= | `AT+OLED_ENABLE=<0\|1>` | 0/1: disable/enable | Enable/disable OLED | `AT+OLED_ENABLE=1` | `OK` |
| AT+OLED_TIMEOUT? | `AT+OLED_TIMEOUT?` | None | Get Timeout | `AT+OLED_TIMEOUT?` | `60` (seconds) |
| AT+OLED_TIMEOUT= | `AT+OLED_TIMEOUT=<value>` | value: seconds | Set Timeout | `AT+OLED_TIMEOUT=60` | `OK` |
| AT+DIM? | `AT+DIM?` | None | Get Dimming Level | `AT+DIM?` | `0` |
| AT+DIM= | `AT+DIM=<value>` | value: level | Set Dimming | `AT+DIM=0` | `OK` |
| AT+CONTRAST? | `AT+CONTRAST?` | None | Get Contrast | `AT+CONTRAST?` | `0` |
| AT+CONTRAST= | `AT+CONTRAST=<value>` | value: level | Set Contrast | `AT+CONTRAST=0` | `OK` |
| AT+STARTUP? | `AT+STARTUP?` | None | Get Startup Screen | `AT+STARTUP?` | `0` |
| AT+STARTUP= | `AT+STARTUP=<value>` | value: type | Set Startup Screen | `AT+STARTUP=0` | `OK` |
| AT+H_UP? | `AT+H_UP?` | None | Get H-up Display | `AT+H_UP?` | `1` or `0` |
| AT+H_UP= | `AT+H_UP=<0\|1>` | 0/1: disable/enable | Enable/disable H-up | `AT+H_UP=1` | `OK` |
| AT+TX_DISPLAY? | `AT+TX_DISPLAY?` | None | Get TX Display | `AT+TX_DISPLAY?` | `1` or `0` |
| AT+TX_DISPLAY= | `AT+TX_DISPLAY=<0\|1>` | 0/1: disable/enable | Enable/disable TX Display | `AT+TX_DISPLAY=1` | `OK` |
| AT+RX_DISPLAY? | `AT+RX_DISPLAY?` | None | Get RX Display | `AT+RX_DISPLAY?` | `1` or `0` |
| AT+RX_DISPLAY= | `AT+RX_DISPLAY=<0\|1>` | 0/1: disable/enable | Enable/disable RX Display | `AT+RX_DISPLAY=1` | `OK` |
| AT+DISPFILTER? | `AT+DISPFILTER?` | None | Get Display Filter | `AT+DISPFILTER?` | `0` |
| AT+DISPFILTER= | `AT+DISPFILTER=<value>` | value: type | Set Filter | `AT+DISPFILTER=0` | `OK` |
| AT+DISPRF? | `AT+DISPRF?` | None | Get RF Display | `AT+DISPRF?` | `1` or `0` |
| AT+DISPRF= | `AT+DISPRF=<0\|1>` | 0/1: disable/enable | Enable/disable RF Display | `AT+DISPRF=1` | `OK` |
| AT+DISPINET? | `AT+DISPINET?` | None | Get INET Display | `AT+DISPINET?` | `1` or `0` |
| AT+DISPINET= | `AT+DISPINET=<0\|1>` | 0/1: disable/enable | Enable/disable INET Display | `AT+DISPINET=1` | `OK` |
| AT+DISP_FLIP? | `AT+DISP_FLIP?` | None | Get Display Flip | `AT+DISP_FLIP?` | `1` or `0` |
| AT+DISP_FLIP= | `AT+DISP_FLIP=<0\|1>` | 0/1: disable/enable | Enable/disable Flip | `AT+DISP_FLIP=1` | `OK` |
| AT+DISP_BRIGHTNESS? | `AT+DISP_BRIGHTNESS?` | None | Get Brightness | `AT+DISP_BRIGHTNESS?` | `100` |
| AT+DISP_BRIGHTNESS= | `AT+DISP_BRIGHTNESS=<value>` | value: % | Set Brightness | `AT+DISP_BRIGHTNESS=100` | `OK` |

---

## Network/Server Commands

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+TX_TIMESLOT? | `AT+TX_TIMESLOT?` | None | Get TX Timeslot | `AT+TX_TIMESLOT?` | `0` |
| AT+TX_TIMESLOT= | `AT+TX_TIMESLOT=<value>` | value: timeslot | Set TX Timeslot | `AT+TX_TIMESLOT=0` | `OK` |
| AT+NTP_HOST? | `AT+NTP_HOST?` | None | Get NTP Server | `AT+NTP_HOST?` | `pool.ntp.org` |
| AT+NTP_HOST= | `AT+NTP_HOST="<host>"` | host: NTP | Set NTP Server | `AT+NTP_HOST="time.google.com"` | `OK` |
| AT+VPN? | `AT+VPN?` | None | Get VPN | `AT+VPN?` | `1` or `0` |
| AT+VPN= | `AT+VPN=<0\|1>` | 0/1: disable/enable | Enable/disable VPN | `AT+VPN=1` | `OK` |
| AT+MODEM? | `AT+MODEM?` | None | Get Modem | `AT+MODEM?` | `1` or `0` |
| AT+MODEM= | `AT+MODEM=<0\|1>` | 0/1: disable/enable | Enable/disable Modem | `AT+MODEM=1` | `OK` |
| AT+HOST_NAME? | `AT+HOST_NAME?` | None | Get Hostname | `AT+HOST_NAME?` | `esp32aprs` |
| AT+HOST_NAME= | `AT+HOST_NAME="<name>"` | name: hostname | Set Hostname | `AT+HOST_NAME="myaprs"` | `OK` |
| AT+HTTP_USERNAME? | `AT+HTTP_USERNAME?` | None | Get HTTP User | `AT+HTTP_USERNAME?` | `admin` |
| AT+HTTP_USERNAME= | `AT+HTTP_USERNAME="<user>"` | user: username | Set HTTP User | `AT+HTTP_USERNAME="admin"` | `OK` |
| AT+HTTP_PASSWORD? | `AT+HTTP_PASSWORD?` | None | Get HTTP Password | `AT+HTTP_PASSWORD?` | `password` |
| AT+HTTP_PASSWORD= | `AT+HTTP_PASSWORD="<pass>"` | pass: password | Set HTTP Password | `AT+HTTP_PASSWORD="secret"` | `OK` |

---

## WireGuard Commands

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+WG_PORT? | `AT+WG_PORT?` | None | Get WireGuard Port | `AT+WG_PORT?` | `51820` |
| AT+WG_PORT= | `AT+WG_PORT=<value>` | value: port | Set WG Port | `AT+WG_PORT=51820` | `OK` |
| AT+WG_PEER_ADDRESS? | `AT+WG_PEER_ADDRESS?` | None | Get Peer Address | `AT+WG_PEER_ADDRESS?` | `1.2.3.4` |
| AT+WG_PEER_ADDRESS= | `AT+WG_PEER_ADDRESS="<ip>"` | ip: Peer IP | Set Peer Address | `AT+WG_PEER_ADDRESS="1.2.3.4"` | `OK` |
| AT+WG_LOCAL_ADDRESS? | `AT+WG_LOCAL_ADDRESS?` | None | Get Local Address | `AT+WG_LOCAL_ADDRESS?` | `10.0.0.2` |
| AT+WG_LOCAL_ADDRESS= | `AT+WG_LOCAL_ADDRESS="<ip>"` | ip: Local IP | Set Local Address | `AT+WG_LOCAL_ADDRESS="10.0.0.2"` | `OK` |
| AT+WG_NETMASK_ADDRESS? | `AT+WG_NETMASK_ADDRESS?` | None | Get Netmask | `AT+WG_NETMASK_ADDRESS?` | `255.255.255.0` |
| AT+WG_NETMASK_ADDRESS= | `AT+WG_NETMASK_ADDRESS="<mask>"` | mask: netmask | Set Netmask | `AT+WG_NETMASK_ADDRESS="255.255.255.0"` | `OK` |
| AT+WG_GW_ADDRESS? | `AT+WG_GW_ADDRESS?` | None | Get Gateway | `AT+WG_GW_ADDRESS?` | `10.0.0.1` |
| AT+WG_GW_ADDRESS= | `AT+WG_GW_ADDRESS="<ip>"` | ip: Gateway IP | Set Gateway | `AT+WG_GW_ADDRESS="10.0.0.1"` | `OK` |
| AT+WG_PUBLIC_KEY? | `AT+WG_PUBLIC_KEY?` | None | Get Public Key | `AT+WG_PUBLIC_KEY?` | (base64) |
| AT+WG_PUBLIC_KEY= | `AT+WG_PUBLIC_KEY="<key>"` | key: base64 | Set Public Key | `AT+WG_PUBLIC_KEY="abc..."` | `OK` |
| AT+WG_PRIVATE_KEY? | `AT+WG_PRIVATE_KEY?` | None | Get Private Key | `AT+WG_PRIVATE_KEY?` | (base64) |
| AT+WG_PRIVATE_KEY= | `AT+WG_PRIVATE_KEY="<key>"` | key: base64 | Set Private Key | `AT+WG_PRIVATE_KEY="xyz..."` | `OK` |

---

## GNSS/GPS Commands

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+GNSS_ENABLE? | `AT+GNSS_ENABLE?` | None | Get GNSS status | `AT+GNSS_ENABLE?` | `1` or `0` |
| AT+GNSS_ENABLE= | `AT+GNSS_ENABLE=<0\|1>` | 0/1: disable/enable | Enable/disable GNSS | `AT+GNSS_ENABLE=1` | `OK` |
| AT+GNSS_PPS_GPIO? | `AT+GNSS_PPS_GPIO?` | None | Get PPS GPIO | `AT+GNSS_PPS_GPIO?` | `0` |
| AT+GNSS_PPS_GPIO= | `AT+GNSS_PPS_GPIO=<value>` | value: GPIO | Set PPS GPIO | `AT+GNSS_PPS_GPIO=0` | `OK` |
| AT+GNSS_CHANNEL? | `AT+GNSS_CHANNEL?` | None | Get Channel | `AT+GNSS_CHANNEL?` | `0` |
| AT+GNSS_CHANNEL= | `AT+GNSS_CHANNEL=<value>` | value: channel | Set Channel | `AT+GNSS_CHANNEL=0` | `OK` |
| AT+GNSS_TCP_PORT? | `AT+GNSS_TCP_PORT?` | None | Get TCP Port | `AT+GNSS_TCP_PORT?` | `0` |
| AT+GNSS_TCP_PORT= | `AT+GNSS_TCP_PORT=<value>` | value: port | Set TCP Port | `AT+GNSS_TCP_PORT=0` | `OK` |
| AT+GNSS_TCP_HOST? | `AT+GNSS_TCP_HOST?` | None | Get TCP Host | `AT+GNSS_TCP_HOST?` | `gps.server.com` |
| AT+GNSS_TCP_HOST= | `AT+GNSS_TCP_HOST="<host>"` | host: server | Set TCP Host | `AT+GNSS_TCP_HOST="gps.server.com"` | `OK` |
| AT+GNSS_AT_COMMAND? | `AT+GNSS_AT_COMMAND?` | None | Get AT Command | `AT+GNSS_AT_COMMAND?` | `PMTK...` |
| AT+GNSS_AT_COMMAND= | `AT+GNSS_AT_COMMAND="<cmd>"` | cmd: AT command | Set AT Command | `AT+GNSS_AT_COMMAND="PMTK..."` | `OK` |

---

## I2C Commands

### I2C Channel 0

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+I2C_ENABLE? | `AT+I2C_ENABLE?` | None | Get I2C status | `AT+I2C_ENABLE?` | `1` or `0` |
| AT+I2C_ENABLE= | `AT+I2C_ENABLE=<0\|1>` | 0/1: disable/enable | Enable/disable I2C | `AT+I2C_ENABLE=1` | `OK` |
| AT+I2C_SDA_PIN? | `AT+I2C_SDA_PIN?` | None | Get SDA Pin | `AT+I2C_SDA_PIN?` | `21` |
| AT+I2C_SDA_PIN= | `AT+I2C_SDA_PIN=<value>` | value: GPIO | Set SDA Pin | `AT+I2C_SDA_PIN=21` | `OK` |
| AT+I2C_SCK_PIN? | `AT+I2C_SCK_PIN?` | None | Get SCK Pin | `AT+I2C_SCK_PIN?` | `22` |
| AT+I2C_SCK_PIN= | `AT+I2C_SCK_PIN=<value>` | value: GPIO | Set SCK Pin | `AT+I2C_SCK_PIN=22` | `OK` |
| AT+I2C_RST_PIN? | `AT+I2C_RST_PIN?` | None | Get Reset Pin | `AT+I2C_RST_PIN?` | `0` |
| AT+I2C_RST_PIN= | `AT+I2C_RST_PIN=<value>` | value: GPIO | Set Reset Pin | `AT+I2C_RST_PIN=0` | `OK` |
| AT+I2C_FREQ? | `AT+I2C_FREQ?` | None | Get Frequency | `AT+I2C_FREQ?` | `400000` |
| AT+I2C_FREQ= | `AT+I2C_FREQ=<value>` | value: Hz | Set Frequency | `AT+I2C_FREQ=400000` | `OK` |

### I2C Channel 1

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+I2C1_ENABLE? | `AT+I2C1_ENABLE?` | None | Get I2C1 status | `AT+I2C1_ENABLE?` | `1` or `0` |
| AT+I2C1_ENABLE= | `AT+I2C1_ENABLE=<0\|1>` | 0/1: disable/enable | Enable/disable I2C1 | `AT+I2C1_ENABLE=1` | `OK` |
| AT+I2C1_SDA_PIN? | `AT+I2C1_SDA_PIN?` | None | Get SDA Pin | `AT+I2C1_SDA_PIN?` | `0` |
| AT+I2C1_SDA_PIN= | `AT+I2C1_SDA_PIN=<value>` | value: GPIO | Set SDA Pin | `AT+I2C1_SDA_PIN=0` | `OK` |
| AT+I2C1_SCK_PIN? | `AT+I2C1_SCK_PIN?` | None | Get SCK Pin | `AT+I2C1_SCK_PIN?` | `0` |
| AT+I2C1_SCK_PIN= | `AT+I2C1_SCK_PIN=<value>` | value: GPIO | Set SCK Pin | `AT+I2C1_SCK_PIN=0` | `OK` |
| AT+I2C1_FREQ? | `AT+I2C1_FREQ?` | None | Get Frequency | `AT+I2C1_FREQ?` | `400000` |
| AT+I2C1_FREQ= | `AT+I2C1_FREQ=<value>` | value: Hz | Set Frequency | `AT+I2C1_FREQ=400000` | `OK` |

---

## UART Commands

### UART0

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+UART0_ENABLE? | `AT+UART0_ENABLE?` | None | Get UART0 status | `AT+UART0_ENABLE?` | `1` or `0` |
| AT+UART0_ENABLE= | `AT+UART0_ENABLE=<0\|1>` | 0/1: disable/enable | Enable/disable UART0 | `AT+UART0_ENABLE=1` | `OK` |
| AT+UART0_TX_GPIO? | `AT+UART0_TX_GPIO?` | None | Get TX GPIO | `AT+UART0_TX_GPIO?` | `1` |
| AT+UART0_TX_GPIO= | `AT+UART0_TX_GPIO=<value>` | value: GPIO | Set TX GPIO | `AT+UART0_TX_GPIO=1` | `OK` |
| AT+UART0_RX_GPIO? | `AT+UART0_RX_GPIO?` | None | Get RX GPIO | `AT+UART0_RX_GPIO?` | `3` |
| AT+UART0_RX_GPIO= | `AT+UART0_RX_GPIO=<value>` | value: GPIO | Set RX GPIO | `AT+UART0_RX_GPIO=3` | `OK` |
| AT+UART0_RTS_GPIO? | `AT+UART0_RTS_GPIO?` | None | Get RTS GPIO | `AT+UART0_RTS_GPIO?` | `0` |
| AT+UART0_RTS_GPIO= | `AT+UART0_RTS_GPIO=<value>` | value: GPIO | Set RTS GPIO | `AT+UART0_RTS_GPIO=0` | `OK` |

### UART1

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+UART1_ENABLE? | `AT+UART1_ENABLE?` | None | Get UART1 status | `AT+UART1_ENABLE?` | `1` or `0` |
| AT+UART1_ENABLE= | `AT+UART1_ENABLE=<0\|1>` | 0/1: disable/enable | Enable/disable UART1 | `AT+UART1_ENABLE=1` | `OK` |
| AT+UART1_TX_GPIO? | `AT+UART1_TX_GPIO?` | None | Get TX GPIO | `AT+UART1_TX_GPIO?` | `0` |
| AT+UART1_TX_GPIO= | `AT+UART1_TX_GPIO=<value>` | value: GPIO | Set TX GPIO | `AT+UART1_TX_GPIO=0` | `OK` |
| AT+UART1_RX_GPIO? | `AT+UART1_RX_GPIO?` | None | Get RX GPIO | `AT+UART1_RX_GPIO?` | `0` |
| AT+UART1_RX_GPIO= | `AT+UART1_RX_GPIO=<value>` | value: GPIO | Set RX GPIO | `AT+UART1_RX_GPIO=0` | `OK` |
| AT+UART1_RTS_GPIO? | `AT+UART1_RTS_GPIO?` | None | Get RTS GPIO | `AT+UART1_RTS_GPIO?` | `0` |
| AT+UART1_RTS_GPIO= | `AT+UART1_RTS_GPIO=<value>` | value: GPIO | Set RTS GPIO | `AT+UART1_RTS_GPIO=0` | `OK` |

### UART2

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+UART2_ENABLE? | `AT+UART2_ENABLE?` | None | Get UART2 status | `AT+UART2_ENABLE?` | `1` or `0` |
| AT+UART2_ENABLE= | `AT+UART2_ENABLE=<0\|1>` | 0/1: disable/enable | Enable/disable UART2 | `AT+UART2_ENABLE=1` | `OK` |
| AT+UART2_TX_GPIO? | `AT+UART2_TX_GPIO?` | None | Get TX GPIO | `AT+UART2_TX_GPIO?` | `0` |
| AT+UART2_TX_GPIO= | `AT+UART2_TX_GPIO=<value>` | value: GPIO | Set TX GPIO | `AT+UART2_TX_GPIO=0` | `OK` |
| AT+UART2_RX_GPIO? | `AT+UART2_RX_GPIO?` | None | Get RX GPIO | `AT+UART2_RX_GPIO?` | `0` |
| AT+UART2_RX_GPIO= | `AT+UART2_RX_GPIO=<value>` | value: GPIO | Set RX GPIO | `AT+UART2_RX_GPIO=0` | `OK` |

---

## Power Management Commands

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+PWR_EN? | `AT+PWR_EN?` | None | Get Power Mgmt status | `AT+PWR_EN?` | `1` or `0` |
| AT+PWR_EN= | `AT+PWR_EN=<0\|1>` | 0/1: disable/enable | Enable/disable Power Mgmt | `AT+PWR_EN=1` | `OK` |
| AT+PWR_MODE? | `AT+PWR_MODE?` | None | Get Power mode | `AT+PWR_MODE?` | `0` |
| AT+PWR_MODE= | `AT+PWR_MODE=<value>` | value: mode | Set Power mode | `AT+PWR_MODE=0` | `OK` |
| AT+PWR_SLEEP_INTERVAL? | `AT+PWR_SLEEP_INTERVAL?` | None | Get Sleep Interval | `AT+PWR_SLEEP_INTERVAL?` | `0` (seconds) |
| AT+PWR_SLEEP_INTERVAL= | `AT+PWR_SLEEP_INTERVAL=<value>` | value: seconds | Set Sleep Interval | `AT+PWR_SLEEP_INTERVAL=0` | `OK` |
| AT+PWR_STANBY_DELAY? | `AT+PWR_STANBY_DELAY?` | None | Get Standby Delay | `AT+PWR_STANBY_DELAY?` | `0` (seconds) |
| AT+PWR_STANBY_DELAY= | `AT+PWR_STANBY_DELAY=<value>` | value: seconds | Set Standby Delay | `AT+PWR_STANBY_DELAY=0` | `OK` |
| AT+PWR_SLEEP_ACTIVATE? | `AT+PWR_SLEEP_ACTIVATE?` | None | Get Sleep Activate | `AT+PWR_SLEEP_ACTIVATE?` | `0` |
| AT+PWR_SLEEP_ACTIVATE= | `AT+PWR_SLEEP_ACTIVATE=<value>` | value: activate | Set Sleep Activate | `AT+PWR_SLEEP_ACTIVATE=0` | `OK` |
| AT+PWR_GPIO? | `AT+PWR_GPIO?` | None | Get Power GPIO | `AT+PWR_GPIO?` | `0` |
| AT+PWR_GPIO= | `AT+PWR_GPIO=<value>` | value: GPIO | Set Power GPIO | `AT+PWR_GPIO=0` | `OK` |
| AT+PWR_ACTIVE? | `AT+PWR_ACTIVE?` | None | Get Power Active | `AT+PWR_ACTIVE?` | `1` or `0` |
| AT+PWR_ACTIVE= | `AT+PWR_ACTIVE=<0\|1>` | 0/1: Low/High | Set Power Active | `AT+PWR_ACTIVE=1` | `OK` |

---

## MQTT Commands

> **Note:** These commands are available only when MQTT feature is enabled

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+EN_MQTT? | `AT+EN_MQTT?` | None | Get MQTT status | `AT+EN_MQTT?` | `1` or `0` |
| AT+EN_MQTT= | `AT+EN_MQTT=<0\|1>` | 0/1: disable/enable | Enable/disable MQTT | `AT+EN_MQTT=1` | `OK` |
| AT+MQTT_HOST? | `AT+MQTT_HOST?` | None | Get Broker Host | `AT+MQTT_HOST?` | `broker.hivemq.com` |
| AT+MQTT_HOST= | `AT+MQTT_HOST="<host>"` | host: broker | Set Broker | `AT+MQTT_HOST="broker.com"` | `OK` |
| AT+MQTT_TOPIC? | `AT+MQTT_TOPIC?` | None | Get Topic | `AT+MQTT_TOPIC?` | `aprs/data` |
| AT+MQTT_TOPIC= | `AT+MQTT_TOPIC="<topic>"` | topic: string | Set Topic | `AT+MQTT_TOPIC="aprs/data"` | `OK` |
| AT+MQTT_SUBSCRIBE? | `AT+MQTT_SUBSCRIBE?` | None | Get Subscribe Topic | `AT+MQTT_SUBSCRIBE?` | `aprs/cmd` |
| AT+MQTT_SUBSCRIBE= | `AT+MQTT_SUBSCRIBE="<topic>"` | topic: string | Set Subscribe | `AT+MQTT_SUBSCRIBE="aprs/cmd"` | `OK` |
| AT+MQTT_USER? | `AT+MQTT_USER?` | None | Get Username | `AT+MQTT_USER?` | `myuser` |
| AT+MQTT_USER= | `AT+MQTT_USER="<user>"` | user: username | Set Username | `AT+MQTT_USER="myuser"` | `OK` |
| AT+MQTT_PASS? | `AT+MQTT_PASS?` | None | Get Password | `AT+MQTT_PASS?` | `mypass` |
| AT+MQTT_PASS= | `AT+MQTT_PASS="<pass>"` | pass: password | Set Password | `AT+MQTT_PASS="mypass"` | `OK` |
| AT+MQTT_PORT? | `AT+MQTT_PORT?` | None | Get Port | `AT+MQTT_PORT?` | `1883` |
| AT+MQTT_PORT= | `AT+MQTT_PORT=<value>` | value: port | Set Port | `AT+MQTT_PORT=1883` | `OK` |
| AT+MQTT_TOPIC_FLAG? | `AT+MQTT_TOPIC_FLAG?` | None | Get Topic Flag | `AT+MQTT_TOPIC_FLAG?` | `0` |
| AT+MQTT_TOPIC_FLAG= | `AT+MQTT_TOPIC_FLAG=<value>` | value: flag | Set Topic Flag | `AT+MQTT_TOPIC_FLAG=0` | `OK` |
| AT+MQTT_SUBSCRIBE_FLAG? | `AT+MQTT_SUBSCRIBE_FLAG?` | None | Get Subscribe Flag | `AT+MQTT_SUBSCRIBE_FLAG?` | `0` |
| AT+MQTT_SUBSCRIBE_FLAG= | `AT+MQTT_SUBSCRIBE_FLAG=<value>` | value: flag | Set Subscribe Flag | `AT+MQTT_SUBSCRIBE_FLAG=0` | `OK` |

---

## AT Command Interface Commands

| Command | Syntax | Parameters | Description | Example | Response |
|---------|--------|------------|-------------|---------|----------|
| AT+RESET_TIMEOUT? | `AT+RESET_TIMEOUT?` | None | Get Reset Timeout | `AT+RESET_TIMEOUT?` | `0` (seconds) |
| AT+RESET_TIMEOUT= | `AT+RESET_TIMEOUT=<value>` | value: seconds | Set Reset Timeout | `AT+RESET_TIMEOUT=0` | `OK` |
| AT+AT_CMD_MQTT? | `AT+AT_CMD_MQTT?` | None | Get AT via MQTT | `AT+AT_CMD_MQTT?` | `1` or `0` |
| AT+AT_CMD_MQTT= | `AT+AT_CMD_MQTT=<0\|1>` | 0/1: disable/enable | Enable/disable AT via MQTT | `AT+AT_CMD_MQTT=1` | `OK` |
| AT+AT_CMD_MSG? | `AT+AT_CMD_MSG?` | None | Get AT via Message | `AT+AT_CMD_MSG?` | `1` or `0` |
| AT+AT_CMD_MSG= | `AT+AT_CMD_MSG=<0\|1>` | 0/1: disable/enable | Enable/disable AT via MSG | `AT+AT_CMD_MSG=1` | `OK` |
| AT+AT_CMD_BLUETOOTH? | `AT+AT_CMD_BLUETOOTH?` | None | Get AT via Bluetooth | `AT+AT_CMD_BLUETOOTH?` | `1` or `0` |
| AT+AT_CMD_BLUETOOTH= | `AT+AT_CMD_BLUETOOTH=<0\|1>` | 0/1: disable/enable | Enable/disable AT via BT | `AT+AT_CMD_BLUETOOTH=1` | `OK` |
| AT+AT_CMD_UART? | `AT+AT_CMD_UART?` | None | Get AT via UART | `AT+AT_CMD_UART?` | `0` |
| AT+AT_CMD_UART= | `AT+AT_CMD_UART=<value>` | value: UART channel | Set AT via UART | `AT+AT_CMD_UART=0` | `OK` |

---

## Appendix

### Command Syntax

- **Read command:** Ends with `?` e.g., `AT+TIME?`
- **Set command:** Ends with `=<value>` e.g., `AT+TIMEZONE=7.0`
- **Execute command:** No `?` or `=` e.g., `AT+RESET`

### Boolean Values

- `0` = Disable / Off / False
- `1` = Enable / On / True

### Important Notes

1. Some commands are available only when related features are enabled (Bluetooth, RF2, MQTT, PPP)
2. Most settings are saved when using `AT+SAVECONFIG` command
3. `AT+RESET` command will reboot the device immediately

---

**This document generated from:** `handleATCommand.cpp`  
**Total Commands:** 331+
