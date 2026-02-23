# คู่มือคำสั่ง AT Command - ESP32 APRS LoRa

เอกสารคู่มือการใช้งาน AT Command สำหรับเฟิร์มแวร์ ESP32 APRS LoRa

---

## สารบัญ

- [คำสั่งพื้นฐาน](#คำสั่งพื้นฐาน)
- [คำสั่ง WiFi](#คำสั่ง-wifi)
- [คำสั่ง Time/เวลา](#คำสั่ง-timeเวลา)
- [คำสั่ง Bluetooth](#คำสั่ง-bluetooth)
- [คำสั่ง RF/LoRa](#คำสั่ง-rflora)
- [คำสั่ง IGATE](#คำสั่ง-igate)
- [คำสั่ง DIGI](#คำสั่ง-digi)
- [คำสั่ง TRACKER](#คำสั่ง-tracker)
- [คำสั่ง WEATHER](#คำสั่ง-weather)
- [คำสั่ง TELEMETRY](#คำสั่ง-telemetry)
- [คำสั่ง OLED/Display](#คำสั่ง-oleddisplay)
- [คำสั่ง Network/Server](#คำสั่ง-networkserver)
- [คำสั่ง WireGuard](#คำสั่ง-wireguard)
- [คำสั่ง GNSS/GPS](#คำสั่ง-gnssgps)
- [คำสั่ง I2C](#คำสั่ง-i2c)
- [คำสั่ง UART](#คำสั่ง-uart)
- [คำสั่ง Power Management](#คำสั่ง-power-management)
- [คำสั่ง MQTT](#คำสั่ง-mqtt)
- [คำสั่ง AT Command Interface](#คำสั่ง-at-command-interface)

---

## คำสั่งพื้นฐาน

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT | `AT` | ไม่มี | ทดสอบการเชื่อมต่อ | `AT` | `OK` |
| AT+RESET | `AT+RESET` หรือ `AT+RESTART` | ไม่มี | รีบูตระบบ | `AT+RESET` | (อุปกรณ์รีบูต) |
| AT+CHIPID? | `AT+CHIPID?` | ไม่มี | อ่าน Chip ID (MAC Address) | `AT+CHIPID?` | `240AC4123456` |
| AT+SAVECONFIG | `AT+SAVECONFIG` | ไม่มี | บันทึกการตั้งค่า | `AT+SAVECONFIG` | `Configuration Saved` |
| LOADCONFIG | `LOADCONFIG` | ไม่มี | โหลดการตั้งค่า | `LOADCONFIG` | `Configuration Loaded` |

---

## คำสั่ง WiFi

### การเชื่อมต่อ WiFi

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+WIFI_DISCONNECT | `AT+WIFI_DISCONNECT` | ไม่มี | ตัดการเชื่อมต่อ WiFi | `AT+WIFI_DISCONNECT` | `WiFi Disconnected` |
| AT+WIFI_CONNECT | `AT+WIFI_CONNECT` | ไม่มี | เชื่อมต่อ WiFi อีกครั้ง | `AT+WIFI_CONNECT` | `WiFi Reconnecting` |
| AT+WIFI_STATUS? | `AT+WIFI_STATUS?` | ไม่มี | ตรวจสอบสถานะ WiFi | `AT+WIFI_STATUS?` | `WiFi Status: Connected` |
| AT+WIFI_SCAN | `AT+WIFI_SCAN` | ไม่มี | สแกนเครือข่าย WiFi | `AT+WIFI_SCAN` | รายชื่อเครือข่าย |
| AT+WIFI? | `AT+WIFI?` | ไม่มี | ข้อมูล WiFi ปัจจุบัน | `AT+WIFI?` | `Mode:STA,SSID:MyNetwork,RSSI:-65dBm` |

### การตั้งค่า WiFi

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+WIFI_MODE? | `AT+WIFI_MODE?` | ไม่มี | อ่านโหมด WiFi | `AT+WIFI_MODE?` | `1` (0=OFF, 1=STA, 2=AP, 3=AP+STA) |
| AT+WIFI_MODE= | `AT+WIFI_MODE=<0-3>` | 0-3: โหมด WiFi | ตั้งค่าโหมด WiFi | `AT+WIFI_MODE=1` | `OK` |
| AT+WIFI_POWER? | `AT+WIFI_POWER?` | ไม่มี | อ่านกำลังส่ง WiFi | `AT+WIFI_POWER?` | `20` (dBm) |
| AT+WIFI_POWER= | `AT+WIFI_POWER=<ค่า>` | ค่า: กำลังส่ง dBm | ตั้งค่ากำลังส่ง WiFi | `AT+WIFI_POWER=20` | `OK` |

### WiFi AP Settings

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+WIFI_AP_CH? | `AT+WIFI_AP_CH?` | ไม่มี | อ่านช่อง AP | `AT+WIFI_AP_CH?` | `1` |
| AT+WIFI_AP_CH= | `AT+WIFI_AP_CH=<1-13>` | 1-13: ช่องสัญญาณ | ตั้งค่าช่อง AP | `AT+WIFI_AP_CH=6` | `OK` |
| AT+WIFI_AP_SSID? | `AT+WIFI_AP_SSID?` | ไม่มี | อ่านชื่อ AP | `AT+WIFI_AP_SSID?` | `ESP32APRS` |
| AT+WIFI_AP_SSID= | `AT+WIFI_AP_SSID="<ชื่อ>"` | ชื่อ: SSID | ตั้งชื่อ AP | `AT+WIFI_AP_SSID="MyAP"` | `OK` |
| AT+WIFI_AP_PASS? | `AT+WIFI_AP_PASS?` | ไม่มี | อ่านรหัสผ่าน AP | `AT+WIFI_AP_PASS?` | `12345678` |
| AT+WIFI_AP_PASS= | `AT+WIFI_AP_PASS="<รหัส>"` | รหัส: รหัสผ่าน | ตั้งรหัสผ่าน AP | `AT+WIFI_AP_PASS="mypass"` | `OK` |

### WiFi STA Profiles (5 โปรไฟล์: 0-4)

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+WIFI[n]EN? | `AT+WIFI[n]EN?` | n: 0-4 | อ่านสถานะโปรไฟล์ | `AT+WIFI0EN?` | `1` หรือ `0` |
| AT+WIFI[n]EN= | `AT+WIFI[n]EN=<0\|1>` | 0/1: เปิด/ปิด | เปิด/ปิดโปรไฟล์ | `AT+WIFI0EN=1` | `OK` |
| AT+WIFI[n]SSID? | `AT+WIFI[n]SSID?` | n: 0-4 | อ่าน SSID โปรไฟล์ | `AT+WIFI0SSID?` | `MyNetwork` |
| AT+WIFI[n]SSID= | `AT+WIFI[n]SSID="<ssid>"` | n: 0-4, ssid: ชื่อ | ตั้ง SSID โปรไฟล์ | `AT+WIFI0SSID="Home"` | `OK` |
| AT+WIFI[n]PASS? | `AT+WIFI[n]PASS?` | n: 0-4 | อ่านรหัสผ่านโปรไฟล์ | `AT+WIFI0PASS?` | `secret123` |
| AT+WIFI[n]PASS= | `AT+WIFI[n]PASS="<pass>"` | n: 0-4, pass: รหัส | ตั้งรหัสผ่านโปรไฟล์ | `AT+WIFI0PASS="mypass"` | `OK` |

> **หมายเหตุ:** แทน `[n]` ด้วย 0, 1, 2, 3, หรือ 4 สำหรับแต่ละโปรไฟล์ WiFi STA

---

## คำสั่ง Time/เวลา

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+TIME? | `AT+TIME?` | ไม่มี | อ่านเวลาปัจจุบัน | `AT+TIME?` | `2024-02-23 14:30:00` |
| AT+TIME= | `AT+TIME="YYYY-MM-DD HH:MM:SS"` | สตริงวันที่/เวลา | ตั้งเวลา | `AT+TIME="2024-02-23 14:30:00"` | `Set Time: 2024-02-23 14:30:00` |
| AT+TIMEZONE? | `AT+TIMEZONE?` | ไม่มี | อ่านเขตเวลา | `AT+TIMEZONE?` | `7.000000` |
| AT+TIMEZONE= | `AT+TIMEZONE=<ค่า>` | ค่า: ชั่วโมง (float) | ตั้งเขตเวลา | `AT+TIMEZONE=7.0` | `OK` |
| AT+SYNCTIME? | `AT+SYNCTIME?` | ไม่มี | อ่านสถานะ NTP | `AT+SYNCTIME?` | `1` หรือ `0` |
| AT+SYNCTIME= | `AT+SYNCTIME=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด NTP | `AT+SYNCTIME=1` | `OK` |

---

## คำสั่ง Bluetooth

> **หมายเหตุ:** คำสั่งเหล่านี้ใช้งานได้เฉพาะเมื่อเปิดฟีเจอร์ BLUETOOTH

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+BT_SLAVE? | `AT+BT_SLAVE?` | ไม่มี | อ่านสถานะ Slave | `AT+BT_SLAVE?` | `1` หรือ `0` |
| AT+BT_SLAVE= | `AT+BT_SLAVE=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Slave | `AT+BT_SLAVE=1` | `OK` |
| AT+BT_MASTER? | `AT+BT_MASTER?` | ไม่มี | อ่านสถานะ Master | `AT+BT_MASTER?` | `1` หรือ `0` |
| AT+BT_MASTER= | `AT+BT_MASTER=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Master | `AT+BT_MASTER=1` | `OK` |
| AT+BT_MODE? | `AT+BT_MODE?` | ไม่มี | อ่านโหมด Bluetooth | `AT+BT_MODE?` | `0` |
| AT+BT_MODE= | `AT+BT_MODE=<ค่า>` | ค่า: โหมด | ตั้งโหมด Bluetooth | `AT+BT_MODE=0` | `OK` |
| AT+BT_NAME? | `AT+BT_NAME?` | ไม่มี | อ่านชื่ออุปกรณ์ | `AT+BT_NAME?` | `ESP32APRS` |
| AT+BT_NAME= | `AT+BT_NAME="<ชื่อ>"` | ชื่อ: สตริง | ตั้งชื่ออุปกรณ์ | `AT+BT_NAME="MyAPRS"` | `OK` |
| AT+BT_PIN? | `AT+BT_PIN?` | ไม่มี | อ่าน PIN | `AT+BT_PIN?` | `1234` |
| AT+BT_PIN= | `AT+BT_PIN=<ค่า>` | ค่า: PIN | ตั้ง PIN | `AT+BT_PIN=1234` | `OK` |
| AT+BT_POWER? | `AT+BT_POWER?` | ไม่มี | อ่านกำลังส่ง | `AT+BT_POWER?` | `0` |
| AT+BT_POWER= | `AT+BT_POWER=<ค่า>` | ค่า: กำลังส่ง | ตั้งกำลังส่ง | `AT+BT_POWER=0` | `OK` |
| AT+BT_UUID? | `AT+BT_UUID?` | ไม่มี | อ่าน UUID (ESP32-S3/C3) | `AT+BT_UUID?` | `00001801-...` |
| AT+BT_UUID= | `AT+BT_UUID="<uuid>"` | uuid: สตริง | ตั้ง UUID | `AT+BT_UUID="00001801..."` | `OK` |
| AT+BT_UUID_RX? | `AT+BT_UUID_RX?` | ไม่มี | อ่าน UUID RX | `AT+BT_UUID_RX?` | (UUID) |
| AT+BT_UUID_RX= | `AT+BT_UUID_RX="<uuid>"` | uuid: สตริง | ตั้ง UUID RX | `AT+BT_UUID_RX="..."` | `OK` |
| AT+BT_UUID_TX? | `AT+BT_UUID_TX?` | ไม่มี | อ่าน UUID TX | `AT+BT_UUID_TX?` | (UUID) |
| AT+BT_UUID_TX= | `AT+BT_UUID_TX="<uuid>"` | uuid: สตริง | ตั้ง UUID TX | `AT+BT_UUID_TX="..."` | `OK` |

---

## คำสั่ง RF/LoRa

### การตั้งค่า RF หลัก

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+RF_EN? | `AT+RF_EN?` | ไม่มี | อ่านสถานะ RF | `AT+RF_EN?` | `1` หรือ `0` |
| AT+RF_EN= | `AT+RF_EN=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด RF | `AT+RF_EN=1` | `OK` |
| AT+RF_TYPE? | `AT+RF_TYPE?` | ไม่มี | อ่านประเภท RF | `AT+RF_TYPE?` | `0` |
| AT+RF_TYPE= | `AT+RF_TYPE=<ค่า>` | ค่า: ประเภท | ตั้งประเภท RF | `AT+RF_TYPE=0` | `OK` |
| AT+RF_MODE? | `AT+RF_MODE?` | ไม่มี | อ่านโหมด RF | `AT+RF_MODE?` | `0` |
| AT+RF_MODE= | `AT+RF_MODE=<ค่า>` | ค่า: โหมด | ตั้งโหมด RF | `AT+RF_MODE=0` | `OK` |
| AT+RF_FREQ? | `AT+RF_FREQ?` | ไม่มี | อ่านความถี่ (MHz) | `AT+RF_FREQ?` | `144.39000` |
| AT+RF_FREQ= | `AT+RF_FREQ=<ค่า>` | ค่า: ความถี่ MHz | ตั้งความถี่ | `AT+RF_FREQ=144.390` | `OK` |
| AT+RF_FREQ_OFFSET? | `AT+RF_FREQ_OFFSET?` | ไม่มี | อ่าน offset ความถี่ | `AT+RF_FREQ_OFFSET?` | `0` |
| AT+RF_FREQ_OFFSET= | `AT+RF_FREQ_OFFSET=<ค่า>` | ค่า: offset Hz | ตั้ง offset | `AT+RF_FREQ_OFFSET=0` | `OK` |
| AT+RF_BW? | `AT+RF_BW?` | ไม่มี | อ่าน Bandwidth (kHz) | `AT+RF_BW?` | `125.00` |
| AT+RF_BW= | `AT+RF_BW=<ค่า>` | ค่า: BW kHz | ตั้ง Bandwidth | `AT+RF_BW=125.0` | `OK` |
| AT+RF_BR? | `AT+RF_BR?` | ไม่มี | อ่าน Bit Rate | `AT+RF_BR?` | `1200.00` |
| AT+RF_BR= | `AT+RF_BR=<ค่า>` | ค่า: Bit Rate | ตั้ง Bit Rate | `AT+RF_BR=1200.0` | `OK` |
| AT+RF_SF? | `AT+RF_SF?` | ไม่มี | อ่าน Spreading Factor | `AT+RF_SF?` | `7` |
| AT+RF_SF= | `AT+RF_SF=<7-12>` | 7-12: SF | ตั้ง SF | `AT+RF_SF=7` | `OK` |
| AT+RF_CR? | `AT+RF_CR?` | ไม่มี | อ่าน Coding Rate | `AT+RF_CR?` | `5` |
| AT+RF_CR= | `AT+RF_CR=<ค่า>` | ค่า: CR | ตั้ง Coding Rate | `AT+RF_CR=5` | `OK` |
| AT+RF_SYNC? | `AT+RF_SYNC?` | ไม่มี | อ่าน Sync Word | `AT+RF_SYNC?` | `0x12` |
| AT+RF_SYNC= | `AT+RF_SYNC=<ค่า>` | ค่า: Sync Word | ตั้ง Sync Word | `AT+RF_SYNC=0x12` | `OK` |
| AT+RF_POWER? | `AT+RF_POWER?` | ไม่มี | อ่านกำลังส่ง | `AT+RF_POWER?` | `17` |
| AT+RF_POWER= | `AT+RF_POWER=<ค่า>` | ค่า: dBm | ตั้งกำลังส่ง | `AT+RF_POWER=17` | `OK` |
| AT+RF_PREAMABLE? | `AT+RF_PREAMABLE?` | ไม่มี | อ่าน Preamble Length | `AT+RF_PREAMABLE?` | `8` |
| AT+RF_PREAMABLE= | `AT+RF_PREAMABLE=<ค่า>` | ค่า: ความยาว | ตั้ง Preamble | `AT+RF_PREAMABLE=8` | `OK` |
| AT+RF_LNA? | `AT+RF_LNA?` | ไม่มี | อ่าน LNA Gain | `AT+RF_LNA?` | `0` |
| AT+RF_LNA= | `AT+RF_LNA=<ค่า>` | ค่า: Gain | ตั้ง LNA Gain | `AT+RF_LNA=0` | `OK` |
| AT+RF_AX25? | `AT+RF_AX25?` | ไม่มี | อ่านโหมด AX.25 | `AT+RF_AX25?` | `1` หรือ `0` |
| AT+RF_AX25= | `AT+RF_AX25=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด AX.25 | `AT+RF_AX25=1` | `OK` |
| AT+RF_SHAPING? | `AT+RF_SHAPING?` | ไม่มี | อ่าน Shaping | `AT+RF_SHAPING?` | `0` |
| AT+RF_SHAPING= | `AT+RF_SHAPING=<ค่า>` | ค่า: ประเภท | ตั้ง Shaping | `AT+RF_SHAPING=0` | `OK` |
| AT+RF_ENCODING? | `AT+RF_ENCODING?` | ไม่มี | อ่าน Encoding | `AT+RF_ENCODING?` | `0` |
| AT+RF_ENCODING= | `AT+RF_ENCODING=<ค่า>` | ค่า: ประเภท | ตั้ง Encoding | `AT+RF_ENCODING=0` | `OK` |
| AT+RF_RX_BOOST? | `AT+RF_RX_BOOST?` | ไม่มี | อ่าน RX Boost | `AT+RF_RX_BOOST?` | `1` หรือ `0` |
| AT+RF_RX_BOOST= | `AT+RF_RX_BOOST=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด RX Boost | `AT+RF_RX_BOOST=1` | `OK` |

### RF GPIO Pins

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+RF_TX_GPIO? | `AT+RF_TX_GPIO?` | ไม่มี | อ่าน TX GPIO | `AT+RF_TX_GPIO?` | `4` |
| AT+RF_TX_GPIO= | `AT+RF_TX_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง TX GPIO | `AT+RF_TX_GPIO=4` | `OK` |
| AT+RF_RX_GPIO? | `AT+RF_RX_GPIO?` | ไม่มี | อ่าน RX GPIO | `AT+RF_RX_GPIO?` | `5` |
| AT+RF_RX_GPIO= | `AT+RF_RX_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง RX GPIO | `AT+RF_RX_GPIO=5` | `OK` |
| AT+RF_DIO1_GPIO? | `AT+RF_DIO1_GPIO?` | ไม่มี | อ่าน DIO1 GPIO | `AT+RF_DIO1_GPIO?` | `33` |
| AT+RF_DIO1_GPIO= | `AT+RF_DIO1_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง DIO1 GPIO | `AT+RF_DIO1_GPIO=33` | `OK` |
| AT+RF_RESET_GPIO? | `AT+RF_RESET_GPIO?` | ไม่มี | อ่าน Reset GPIO | `AT+RF_RESET_GPIO?` | `14` |
| AT+RF_RESET_GPIO= | `AT+RF_RESET_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง Reset GPIO | `AT+RF_RESET_GPIO=14` | `OK` |
| AT+RF_DIO0_GPIO? | `AT+RF_DIO0_GPIO?` | ไม่มี | อ่าน DIO0 GPIO | `AT+RF_DIO0_GPIO?` | `2` |
| AT+RF_DIO0_GPIO= | `AT+RF_DIO0_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง DIO0 GPIO | `AT+RF_DIO0_GPIO=2` | `OK` |
| AT+RF_DIO2_GPIO? | `AT+RF_DIO2_GPIO?` | ไม่มี | อ่าน DIO2 GPIO | `AT+RF_DIO2_GPIO?` | `0` |
| AT+RF_DIO2_GPIO= | `AT+RF_DIO2_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง DIO2 GPIO | `AT+RF_DIO2_GPIO=0` | `OK` |
| AT+RF_NSS_GPIO? | `AT+RF_NSS_GPIO?` | ไม่มี | อ่าน NSS GPIO | `AT+RF_NSS_GPIO?` | `5` |
| AT+RF_NSS_GPIO= | `AT+RF_NSS_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง NSS GPIO | `AT+RF_NSS_GPIO=5` | `OK` |
| AT+RF_SCLK_GPIO? | `AT+RF_SCLK_GPIO?` | ไม่มี | อ่าน SCLK GPIO | `AT+RF_SCLK_GPIO?` | `18` |
| AT+RF_SCLK_GPIO= | `AT+RF_SCLK_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง SCLK GPIO | `AT+RF_SCLK_GPIO=18` | `OK` |
| AT+RF_MISO_GPIO? | `AT+RF_MISO_GPIO?` | ไม่มี | อ่าน MISO GPIO | `AT+RF_MISO_GPIO?` | `19` |
| AT+RF_MISO_GPIO= | `AT+RF_MISO_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง MISO GPIO | `AT+RF_MISO_GPIO=19` | `OK` |
| AT+RF_MOSI_GPIO? | `AT+RF_MOSI_GPIO?` | ไม่มี | อ่าน MOSI GPIO | `AT+RF_MOSI_GPIO?` | `23` |
| AT+RF_MOSI_GPIO= | `AT+RF_MOSI_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง MOSI GPIO | `AT+RF_MOSI_GPIO=23` | `OK` |
| AT+RF_TX_ACTIVE? | `AT+RF_TX_ACTIVE?` | ไม่มี | อ่าน TX Active State | `AT+RF_TX_ACTIVE?` | `1` หรือ `0` |
| AT+RF_TX_ACTIVE= | `AT+RF_TX_ACTIVE=<0\|1>` | 0/1: Low/High | ตั้ง TX Active | `AT+RF_TX_ACTIVE=1` | `OK` |
| AT+RF_RX_ACTIVE? | `AT+RF_RX_ACTIVE?` | ไม่มี | อ่าน RX Active State | `AT+RF_RX_ACTIVE?` | `1` หรือ `0` |
| AT+RF_RX_ACTIVE= | `AT+RF_RX_ACTIVE=<0\|1>` | 0/1: Low/High | ตั้ง RX Active | `AT+RF_RX_ACTIVE=1` | `OK` |
| AT+RF_RESET_ACTIVE? | `AT+RF_RESET_ACTIVE?` | ไม่มี | อ่าน Reset Active | `AT+RF_RESET_ACTIVE?` | `1` หรือ `0` |
| AT+RF_RESET_ACTIVE= | `AT+RF_RESET_ACTIVE=<0\|1>` | 0/1: Low/High | ตั้ง Reset Active | `AT+RF_RESET_ACTIVE=1` | `OK` |
| AT+RF_NSS_ACTIVE? | `AT+RF_NSS_ACTIVE?` | ไม่มี | อ่าน NSS Active | `AT+RF_NSS_ACTIVE?` | `1` หรือ `0` |
| AT+RF_NSS_ACTIVE= | `AT+RF_NSS_ACTIVE=<0\|1>` | 0/1: Low/High | ตั้ง NSS Active | `AT+RF_NSS_ACTIVE=1` | `OK` |

> **หมายเหตุ:** คำสั่ง RF1 (เช่น `AT+RF1_FREQ`, `AT+RF1_GPIO`) ใช้สำหรับ RF ช่องที่ 2 (ถ้ามี)

---

## คำสั่ง IGATE

### การตั้งค่าพื้นฐาน IGATE

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+IGATE_EN? | `AT+IGATE_EN?` | ไม่มี | อ่านสถานะ IGATE | `AT+IGATE_EN?` | `1` หรือ `0` |
| AT+IGATE_EN= | `AT+IGATE_EN=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด IGATE | `AT+IGATE_EN=1` | `OK` |
| AT+RF2INET? | `AT+RF2INET?` | ไม่มี | อ่าน RF→INET | `AT+RF2INET?` | `1` หรือ `0` |
| AT+RF2INET= | `AT+RF2INET=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด RF→INET | `AT+RF2INET=1` | `OK` |
| AT+INET2RF? | `AT+INET2RF?` | ไม่มี | อ่าน INET→RF | `AT+INET2RF?` | `1` หรือ `0` |
| AT+INET2RF= | `AT+INET2RF=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด INET→RF | `AT+INET2RF=1` | `OK` |
| AT+IGATE_LOC2RF? | `AT+IGATE_LOC2RF?` | ไม่มี | อ่าน Location→RF | `AT+IGATE_LOC2RF?` | `1` หรือ `0` |
| AT+IGATE_LOC2RF= | `AT+IGATE_LOC2RF=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Loc→RF | `AT+IGATE_LOC2RF=1` | `OK` |
| AT+IGATE_LOC2INET? | `AT+IGATE_LOC2INET?` | ไม่มี | อ่าน Location→INET | `AT+IGATE_LOC2INET?` | `1` หรือ `0` |
| AT+IGATE_LOC2INET= | `AT+IGATE_LOC2INET=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Loc→INET | `AT+IGATE_LOC2INET=1` | `OK` |

### การตั้งค่า Server

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+IGATE_MYCALL? | `AT+IGATE_MYCALL?` | ไม่มี | อ่าน Callsign | `AT+IGATE_MYCALL?` | `MYCALL` |
| AT+IGATE_MYCALL= | `AT+IGATE_MYCALL="<call>"` | call: Callsign | ตั้ง Callsign | `AT+IGATE_MYCALL="HS1ABC"` | `OK` |
| AT+IGATE_HOST? | `AT+IGATE_HOST?` | ไม่มี | อ่าน Server Host | `AT+IGATE_HOST?` | `rotate.aprs2.net` |
| AT+IGATE_HOST= | `AT+IGATE_HOST="<host>"` | host: ที่อยู่ | ตั้ง Server | `AT+IGATE_HOST="noam.aprs2.net"` | `OK` |
| AT+APRS_PORT? | `AT+APRS_PORT?` | ไม่มี | อ่าน Port | `AT+APRS_PORT?` | `14580` |
| AT+APRS_PORT= | `AT+APRS_PORT=<ค่า>` | ค่า: Port | ตั้ง Port | `AT+APRS_PORT=14580` | `OK` |
| AT+IGATE_SSID? | `AT+IGATE_SSID?` | ไม่มี | อ่าน SSID | `AT+IGATE_SSID?` | `1` |
| AT+IGATE_SSID= | `AT+IGATE_SSID=<0-15>` | 0-15: SSID | ตั้ง SSID | `AT+IGATE_SSID=1` | `OK` |

### การตั้งค่า Beacon

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+IGATE_LAT? | `AT+IGATE_LAT?` | ไม่มี | อ่านละติจูด | `AT+IGATE_LAT?` | `13.756331` |
| AT+IGATE_LAT= | `AT+IGATE_LAT=<ค่า>` | ค่า: ละติจูด | ตั้งละติจูด | `AT+IGATE_LAT=13.756331` | `OK` |
| AT+IGATE_LON? | `AT+IGATE_LON?` | ไม่มี | อ่านลองจิจูด | `AT+IGATE_LON?` | `100.501765` |
| AT+IGATE_LON= | `AT+IGATE_LON=<ค่า>` | ค่า: ลองจิจูด | ตั้งลองจิจูด | `AT+IGATE_LON=100.501765` | `OK` |
| AT+IGATE_ALT? | `AT+IGATE_ALT?` | ไม่มี | อ่านความสูง | `AT+IGATE_ALT?` | `10.5` |
| AT+IGATE_ALT= | `AT+IGATE_ALT=<ค่า>` | ค่า: เมตร | ตั้งความสูง | `AT+IGATE_ALT=10.5` | `OK` |
| AT+IGATE_INTERVAL? | `AT+IGATE_INTERVAL?` | ไม่มี | อ่านช่วง Beacon | `AT+IGATE_INTERVAL?` | `600` (วินาที) |
| AT+IGATE_INTERVAL= | `AT+IGATE_INTERVAL=<ค่า>` | ค่า: วินาที | ตั้งช่วง Beacon | `AT+IGATE_INTERVAL=600` | `OK` |
| AT+IGATE_SYMBOL? | `AT+IGATE_SYMBOL?` | ไม่มี | อ่านสัญลักษณ์ APRS | `AT+IGATE_SYMBOL?` | `/I` |
| AT+IGATE_SYMBOL= | `AT+IGATE_SYMBOL="<sym>"` | sym: สัญลักษณ์ | ตั้งสัญลักษณ์ | `AT+IGATE_SYMBOL="/I"` | `OK` |
| AT+IGATE_COMMENT? | `AT+IGATE_COMMENT?` | ไม่มี | อ่านข้อความ | `AT+IGATE_COMMENT?` | `ESP32 APRS` |
| AT+IGATE_COMMENT= | `AT+IGATE_COMMENT="<text>"` | text: ข้อความ | ตั้งข้อความ | `AT+IGATE_COMMENT="ESP32 APRS"` | `OK` |

### Filter และอื่นๆ

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+IGATE_FILTER? | `AT+IGATE_FILTER?` | ไม่มี | อ่าน Filter | `AT+IGATE_FILTER?` | `m/100` |
| AT+IGATE_FILTER= | `AT+IGATE_FILTER="<filter>"` | filter: สตริง | ตั้ง Filter | `AT+IGATE_FILTER="m/100"` | `OK` |
| AT+IGATE_MONICALL? | `AT+IGATE_MONICALL?` | ไม่มี | อ่าน Monitor Callsign | `AT+IGATE_MONICALL?` | `*` |
| AT+IGATE_MONICALL= | `AT+IGATE_MONICALL="<call>"` | call: Callsign | ตั้ง Monitor | `AT+IGATE_MONICALL="*"` | `OK` |
| AT+IGATE_BCN? | `AT+IGATE_BCN?` | ไม่มี | อ่าน Beacon | `AT+IGATE_BCN?` | `1` หรือ `0` |
| AT+IGATE_BCN= | `AT+IGATE_BCN=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Beacon | `AT+IGATE_BCN=1` | `OK` |
| AT+IGATE_GPS? | `AT+IGATE_GPS?` | ไม่มี | อ่าน GPS | `AT+IGATE_GPS?` | `1` หรือ `0` |
| AT+IGATE_GPS= | `AT+IGATE_GPS=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด GPS | `AT+IGATE_GPS=1` | `OK` |
| AT+IGATE_TIMESTAMP? | `AT+IGATE_TIMESTAMP?` | ไม่มี | อ่าน Timestamp | `AT+IGATE_TIMESTAMP?` | `1` หรือ `0` |
| AT+IGATE_TIMESTAMP= | `AT+IGATE_TIMESTAMP=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Timestamp | `AT+IGATE_TIMESTAMP=1` | `OK` |

---

## คำสั่ง DIGI

### การตั้งค่าพื้นฐาน DIGI

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+DIGI_EN? | `AT+DIGI_EN?` | ไม่มี | อ่านสถานะ DIGI | `AT+DIGI_EN?` | `1` หรือ `0` |
| AT+DIGI_EN= | `AT+DIGI_EN=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด DIGI | `AT+DIGI_EN=1` | `OK` |
| AT+DIGI_AUTO? | `AT+DIGI_AUTO?` | ไม่มี | อ่าน Auto Mode | `AT+DIGI_AUTO?` | `1` หรือ `0` |
| AT+DIGI_AUTO= | `AT+DIGI_AUTO=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Auto | `AT+DIGI_AUTO=1` | `OK` |
| AT+DIGI_LOC2RF? | `AT+DIGI_LOC2RF?` | ไม่มี | อ่าน Location→RF | `AT+DIGI_LOC2RF?` | `1` หรือ `0` |
| AT+DIGI_LOC2RF= | `AT+DIGI_LOC2RF=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Loc→RF | `AT+DIGI_LOC2RF=1` | `OK` |
| AT+DIGI_LOC2INET? | `AT+DIGI_LOC2INET?` | ไม่มี | อ่าน Location→INET | `AT+DIGI_LOC2INET?` | `1` หรือ `0` |
| AT+DIGI_LOC2INET= | `AT+DIGI_LOC2INET=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Loc→INET | `AT+DIGI_LOC2INET=1` | `OK` |

### การตั้งค่า Beacon DIGI

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+DIGI_MYCALL? | `AT+DIGI_MYCALL?` | ไม่มี | อ่าน Callsign | `AT+DIGI_MYCALL?` | `MYCALL` |
| AT+DIGI_MYCALL= | `AT+DIGI_MYCALL="<call>"` | call: Callsign | ตั้ง Callsign | `AT+DIGI_MYCALL="HS1ABC"` | `OK` |
| AT+DIGI_SSID? | `AT+DIGI_SSID?` | ไม่มี | อ่าน SSID | `AT+DIGI_SSID?` | `1` |
| AT+DIGI_SSID= | `AT+DIGI_SSID=<0-15>` | 0-15: SSID | ตั้ง SSID | `AT+DIGI_SSID=1` | `OK` |
| AT+DIGI_PATH? | `AT+DIGI_PATH?` | ไม่มี | อ่าน Path | `AT+DIGI_PATH?` | `0` |
| AT+DIGI_PATH= | `AT+DIGI_PATH=<ค่า>` | ค่า: ประเภท | ตั้ง Path | `AT+DIGI_PATH=0` | `OK` |
| AT+DIGI_DELAY? | `AT+DIGI_DELAY?` | ไม่มี | อ่าน Delay | `AT+DIGI_DELAY?` | `0` (ms) |
| AT+DIGI_DELAY= | `AT+DIGI_DELAY=<ค่า>` | ค่า: ms | ตั้ง Delay | `AT+DIGI_DELAY=0` | `OK` |
| AT+DIGI_INTERVAL? | `AT+DIGI_INTERVAL?` | ไม่มี | อ่านช่วง Beacon | `AT+DIGI_INTERVAL?` | `600` (วินาที) |
| AT+DIGI_INTERVAL= | `AT+DIGI_INTERVAL=<ค่า>` | ค่า: วินาที | ตั้งช่วง Beacon | `AT+DIGI_INTERVAL=600` | `OK` |
| AT+DIGI_SYMBOL? | `AT+DIGI_SYMBOL?` | ไม่มี | อ่านสัญลักษณ์ | `AT+DIGI_SYMBOL?` | `/D` |
| AT+DIGI_SYMBOL= | `AT+DIGI_SYMBOL="<sym>"` | sym: สัญลักษณ์ | ตั้งสัญลักษณ์ | `AT+DIGI_SYMBOL="/D"` | `OK` |
| AT+DIGI_COMMENT? | `AT+DIGI_COMMENT?` | ไม่มี | อ่านข้อความ | `AT+DIGI_COMMENT?` | `DIGI` |
| AT+DIGI_COMMENT= | `AT+DIGI_COMMENT="<text>"` | text: ข้อความ | ตั้งข้อความ | `AT+DIGI_COMMENT="DIGI"` | `OK` |

### Filter และอื่นๆ

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+DIGIFILTER? | `AT+DIGIFILTER?` | ไม่มี | อ่าน Filter | `AT+DIGIFILTER?` | `0` |
| AT+DIGIFILTER= | `AT+DIGIFILTER=<ค่า>` | ค่า: ประเภท | ตั้ง Filter | `AT+DIGIFILTER=0` | `OK` |
| AT+DIGI_BCN? | `AT+DIGI_BCN?` | ไม่มี | อ่าน Beacon | `AT+DIGI_BCN?` | `1` หรือ `0` |
| AT+DIGI_BCN= | `AT+DIGI_BCN=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Beacon | `AT+DIGI_BCN=1` | `OK` |
| AT+DIGI_GPS? | `AT+DIGI_GPS?` | ไม่มี | อ่าน GPS | `AT+DIGI_GPS?` | `1` หรือ `0` |
| AT+DIGI_GPS= | `AT+DIGI_GPS=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด GPS | `AT+DIGI_GPS=1` | `OK` |
| AT+DIGI_TIMESTAMP? | `AT+DIGI_TIMESTAMP?` | ไม่มี | อ่าน Timestamp | `AT+DIGI_TIMESTAMP?` | `1` หรือ `0` |
| AT+DIGI_TIMESTAMP= | `AT+DIGI_TIMESTAMP=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Timestamp | `AT+DIGI_TIMESTAMP=1` | `OK` |

---

## คำสั่ง TRACKER

### การตั้งค่าพื้นฐาน Tracker

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+TRK_EN? | `AT+TRK_EN?` | ไม่มี | อ่านสถานะ Tracker | `AT+TRK_EN?` | `1` หรือ `0` |
| AT+TRK_EN= | `AT+TRK_EN=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Tracker | `AT+TRK_EN=1` | `OK` |
| AT+TRK_LOC2RF? | `AT+TRK_LOC2RF?` | ไม่มี | อ่าน Location→RF | `AT+TRK_LOC2RF?` | `1` หรือ `0` |
| AT+TRK_LOC2RF= | `AT+TRK_LOC2RF=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Loc→RF | `AT+TRK_LOC2RF=1` | `OK` |
| AT+TRK_LOC2INET? | `AT+TRK_LOC2INET?` | ไม่มี | อ่าน Location→INET | `AT+TRK_LOC2INET?` | `1` หรือ `0` |
| AT+TRK_LOC2INET= | `AT+TRK_LOC2INET=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Loc→INET | `AT+TRK_LOC2INET=1` | `OK` |

### การตั้งค่า Beacon Tracker

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+TRK_MYCALL? | `AT+TRK_MYCALL?` | ไม่มี | อ่าน Callsign | `AT+TRK_MYCALL?` | `MYCALL` |
| AT+TRK_MYCALL= | `AT+TRK_MYCALL="<call>"` | call: Callsign | ตั้ง Callsign | `AT+TRK_MYCALL="HS1ABC"` | `OK` |
| AT+TRK_SSID? | `AT+TRK_SSID?` | ไม่มี | อ่าน SSID | `AT+TRK_SSID?` | `9` |
| AT+TRK_SSID= | `AT+TRK_SSID=<0-15>` | 0-15: SSID | ตั้ง SSID | `AT+TRK_SSID=9` | `OK` |
| AT+TRK_PATH? | `AT+TRK_PATH?` | ไม่มี | อ่าน Path | `AT+TRK_PATH?` | `0` |
| AT+TRK_PATH= | `AT+TRK_PATH=<ค่า>` | ค่า: ประเภท | ตั้ง Path | `AT+TRK_PATH=0` | `OK` |
| AT+TRK_INTERVAL? | `AT+TRK_INTERVAL?` | ไม่มี | อ่านช่วง Beacon | `AT+TRK_INTERVAL?` | `60` (วินาที) |
| AT+TRK_INTERVAL= | `AT+TRK_INTERVAL=<ค่า>` | ค่า: วินาที | ตั้งช่วง Beacon | `AT+TRK_INTERVAL=60` | `OK` |
| AT+TRK_SYMBOL? | `AT+TRK_SYMBOL?` | ไม่มี | อ่านสัญลักษณ์ | `AT+TRK_SYMBOL?` | `/>` |
| AT+TRK_SYMBOL= | `AT+TRK_SYMBOL="<sym>"` | sym: สัญลักษณ์ | ตั้งสัญลักษณ์ | `AT+TRK_SYMBOL="/>"` | `OK` |
| AT+TRK_COMMENT? | `AT+TRK_COMMENT?` | ไม่มี | อ่านข้อความ | `AT+TRK_COMMENT?` | `Mobile` |
| AT+TRK_COMMENT= | `AT+TRK_COMMENT="<text>"` | text: ข้อความ | ตั้งข้อความ | `AT+TRK_COMMENT="Mobile"` | `OK` |

### SmartBeacon

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+TRK_SMARTBEACON? | `AT+TRK_SMARTBEACON?` | ไม่มี | อ่าน SmartBeacon | `AT+TRK_SMARTBEACON?` | `1` หรือ `0` |
| AT+TRK_SMARTBEACON= | `AT+TRK_SMARTBEACON=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด SmartBeacon | `AT+TRK_SMARTBEACON=1` | `OK` |
| AT+TRK_HSPEED? | `AT+TRK_HSPEED?` | ไม่มี | อ่านความเร็วสูง | `AT+TRK_HSPEED?` | `90` (km/h) |
| AT+TRK_HSPEED= | `AT+TRK_HSPEED=<ค่า>` | ค่า: km/h | ตั้งความเร็วสูง | `AT+TRK_HSPEED=90` | `OK` |
| AT+TRK_LSPEED? | `AT+TRK_LSPEED?` | ไม่มี | อ่านความเร็วต่ำ | `AT+TRK_LSPEED?` | `5` (km/h) |
| AT+TRK_LSPEED= | `AT+TRK_LSPEED=<ค่า>` | ค่า: km/h | ตั้งความเร็วต่ำ | `AT+TRK_LSPEED=5` | `OK` |
| AT+TRK_MAXINTERVAL? | `AT+TRK_MAXINTERVAL?` | ไม่มี | อ่านช่วงสูงสุด | `AT+TRK_MAXINTERVAL?` | `180` (วินาที) |
| AT+TRK_MAXINTERVAL= | `AT+TRK_MAXINTERVAL=<ค่า>` | ค่า: วินาที | ตั้งช่วงสูงสุด | `AT+TRK_MAXINTERVAL=180` | `OK` |
| AT+TRK_MININTERVAL? | `AT+TRK_MININTERVAL?` | ไม่มี | อ่านช่วงต่ำสุด | `AT+TRK_MININTERVAL?` | `10` (วินาที) |
| AT+TRK_MININTERVAL= | `AT+TRK_MININTERVAL=<ค่า>` | ค่า: วินาที | ตั้งช่วงต่ำสุด | `AT+TRK_MININTERVAL=10` | `OK` |
| AT+TRK_MINANGLE? | `AT+TRK_MINANGLE?` | ไม่มี | อ่านมุมต่ำสุด | `AT+TRK_MINANGLE?` | `30` (องศา) |
| AT+TRK_MINANGLE= | `AT+TRK_MINANGLE=<ค่า>` | ค่า: องศา | ตั้งมุมต่ำสุด | `AT+TRK_MINANGLE=30` | `OK` |
| AT+TRK_SLOWINTERVAL? | `AT+TRK_SLOWINTERVAL?` | ไม่มี | อ่านช่วงช้า | `AT+TRK_SLOWINTERVAL?` | `120` (วินาที) |
| AT+TRK_SLOWINTERVAL= | `AT+TRK_SLOWINTERVAL=<ค่า>` | ค่า: วินาที | ตั้งช่วงช้า | `AT+TRK_SLOWINTERVAL=120` | `OK` |

### ตัวเลือกเพิ่มเติม

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+TRK_COMPRESS? | `AT+TRK_COMPRESS?` | ไม่มี | อ่าน Compression | `AT+TRK_COMPRESS?` | `1` หรือ `0` |
| AT+TRK_COMPRESS= | `AT+TRK_COMPRESS=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Compression | `AT+TRK_COMPRESS=1` | `OK` |
| AT+TRK_ALTITUDE? | `AT+TRK_ALTITUDE?` | ไม่มี | อ่าน Altitude Report | `AT+TRK_ALTITUDE?` | `1` หรือ `0` |
| AT+TRK_ALTITUDE= | `AT+TRK_ALTITUDE=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Altitude | `AT+TRK_ALTITUDE=1` | `OK` |
| AT+TRK_LOG? | `AT+TRK_LOG?` | ไม่มี | อ่าน Logging | `AT+TRK_LOG?` | `1` หรือ `0` |
| AT+TRK_LOG= | `AT+TRK_LOG=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Logging | `AT+TRK_LOG=1` | `OK` |
| AT+TRK_RSSI? | `AT+TRK_RSSI?` | ไม่มี | อ่าน RSSI Report | `AT+TRK_RSSI?` | `1` หรือ `0` |
| AT+TRK_RSSI= | `AT+TRK_RSSI=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด RSSI | `AT+TRK_RSSI=1` | `OK` |
| AT+TRK_SAT? | `AT+TRK_SAT?` | ไม่มี | อ่าน Satellite Report | `AT+TRK_SAT?` | `1` หรือ `0` |
| AT+TRK_SAT= | `AT+TRK_SAT=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Satellite | `AT+TRK_SAT=1` | `OK` |
| AT+TRK_DX? | `AT+TRK_DX?` | ไม่มี | อ่าน DX Report | `AT+TRK_DX?` | `1` หรือ `0` |
| AT+TRK_DX= | `AT+TRK_DX=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด DX | `AT+TRK_DX=1` | `OK` |
| AT+TRK_GPS? | `AT+TRK_GPS?` | ไม่มี | อ่าน GPS | `AT+TRK_GPS?` | `1` หรือ `0` |
| AT+TRK_GPS= | `AT+TRK_GPS=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด GPS | `AT+TRK_GPS=1` | `OK` |
| AT+TRK_TIMESTAMP? | `AT+TRK_TIMESTAMP?` | ไม่มี | อ่าน Timestamp | `AT+TRK_TIMESTAMP?` | `1` หรือ `0` |
| AT+TRK_TIMESTAMP= | `AT+TRK_TIMESTAMP=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Timestamp | `AT+TRK_TIMESTAMP=1` | `OK` |

---

## คำสั่ง WEATHER

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+WX_EN? | `AT+WX_EN?` | ไม่มี | อ่านสถานะ Weather | `AT+WX_EN?` | `1` หรือ `0` |
| AT+WX_EN= | `AT+WX_EN=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Weather | `AT+WX_EN=1` | `OK` |
| AT+WX_2RF? | `AT+WX_2RF?` | ไม่มี | อ่าน WX→RF | `AT+WX_2RF?` | `1` หรือ `0` |
| AT+WX_2RF= | `AT+WX_2RF=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด WX→RF | `AT+WX_2RF=1` | `OK` |
| AT+WX_2INET? | `AT+WX_2INET?` | ไม่มี | อ่าน WX→INET | `AT+WX_2INET?` | `1` หรือ `0` |
| AT+WX_2INET= | `AT+WX_2INET=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด WX→INET | `AT+WX_2INET=1` | `OK` |
| AT+WX_MYCALL? | `AT+WX_MYCALL?` | ไม่มี | อ่าน Callsign | `AT+WX_MYCALL?` | `MYCALL` |
| AT+WX_MYCALL= | `AT+WX_MYCALL="<call>"` | call: Callsign | ตั้ง Callsign | `AT+WX_MYCALL="HS1ABC"` | `OK` |
| AT+WX_SSID? | `AT+WX_SSID?` | ไม่มี | อ่าน SSID | `AT+WX_SSID?` | `0` |
| AT+WX_SSID= | `AT+WX_SSID=<0-15>` | 0-15: SSID | ตั้ง SSID | `AT+WX_SSID=0` | `OK` |
| AT+WX_PATH? | `AT+WX_PATH?` | ไม่มี | อ่าน Path | `AT+WX_PATH?` | `0` |
| AT+WX_PATH= | `AT+WX_PATH=<ค่า>` | ค่า: ประเภท | ตั้ง Path | `AT+WX_PATH=0` | `OK` |
| AT+WX_INTERVAL? | `AT+WX_INTERVAL?` | ไม่มี | อ่านช่วง Beacon | `AT+WX_INTERVAL?` | `300` (วินาที) |
| AT+WX_INTERVAL= | `AT+WX_INTERVAL=<ค่า>` | ค่า: วินาที | ตั้งช่วง Beacon | `AT+WX_INTERVAL=300` | `OK` |
| AT+WX_OBJECT? | `AT+WX_OBJECT?` | ไม่มี | อ่าน Object Name | `AT+WX_OBJECT?` | `WX` |
| AT+WX_OBJECT= | `AT+WX_OBJECT="<name>"` | name: ชื่อ | ตั้ง Object | `AT+WX_OBJECT="WX"` | `OK` |
| AT+WX_COMMENT? | `AT+WX_COMMENT?` | ไม่มี | อ่านข้อความ | `AT+WX_COMMENT?` | `WX Station` |
| AT+WX_COMMENT= | `AT+WX_COMMENT="<text>"` | text: ข้อความ | ตั้งข้อความ | `AT+WX_COMMENT="WX Station"` | `OK` |
| AT+WX_GPS? | `AT+WX_GPS?` | ไม่มี | อ่าน GPS | `AT+WX_GPS?` | `1` หรือ `0` |
| AT+WX_GPS= | `AT+WX_GPS=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด GPS | `AT+WX_GPS=1` | `OK` |
| AT+WX_LAT? | `AT+WX_LAT?` | ไม่มี | อ่านละติจูด | `AT+WX_LAT?` | `13.756331` |
| AT+WX_LAT= | `AT+WX_LAT=<ค่า>` | ค่า: ละติจูด | ตั้งละติจูด | `AT+WX_LAT=13.756331` | `OK` |
| AT+WX_LON? | `AT+WX_LON?` | ไม่มี | อ่านลองจิจูด | `AT+WX_LON?` | `100.501765` |
| AT+WX_LON= | `AT+WX_LON=<ค่า>` | ค่า: ลองจิจูด | ตั้งลองจิจูด | `AT+WX_LON=100.501765` | `OK` |
| AT+WX_ALT? | `AT+WX_ALT?` | ไม่มี | อ่านความสูง | `AT+WX_ALT?` | `10.5` |
| AT+WX_ALT= | `AT+WX_ALT=<ค่า>` | ค่า: เมตร | ตั้งความสูง | `AT+WX_ALT=10.5` | `OK` |
| AT+WX_TIMESTAMP? | `AT+WX_TIMESTAMP?` | ไม่มี | อ่าน Timestamp | `AT+WX_TIMESTAMP?` | `1` หรือ `0` |
| AT+WX_TIMESTAMP= | `AT+WX_TIMESTAMP=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Timestamp | `AT+WX_TIMESTAMP=1` | `OK` |

---

## คำสั่ง TELEMETRY

### Telemetry 0 (TLM0)

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+TLM0_EN? | `AT+TLM0_EN?` | ไม่มี | อ่านสถานะ TLM0 | `AT+TLM0_EN?` | `1` หรือ `0` |
| AT+TLM0_EN= | `AT+TLM0_EN=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด TLM0 | `AT+TLM0_EN=1` | `OK` |
| AT+TLM0_2RF? | `AT+TLM0_2RF?` | ไม่มี | อ่าน TLM0→RF | `AT+TLM0_2RF?` | `1` หรือ `0` |
| AT+TLM0_2RF= | `AT+TLM0_2RF=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด TLM0→RF | `AT+TLM0_2RF=1` | `OK` |
| AT+TLM0_2INET? | `AT+TLM0_2INET?` | ไม่มี | อ่าน TLM0→INET | `AT+TLM0_2INET?` | `1` หรือ `0` |
| AT+TLM0_2INET= | `AT+TLM0_2INET=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด TLM0→INET | `AT+TLM0_2INET=1` | `OK` |
| AT+TLM0_MYCALL? | `AT+TLM0_MYCALL?` | ไม่มี | อ่าน Callsign | `AT+TLM0_MYCALL?` | `MYCALL` |
| AT+TLM0_MYCALL= | `AT+TLM0_MYCALL="<call>"` | call: Callsign | ตั้ง Callsign | `AT+TLM0_MYCALL="HS1ABC"` | `OK` |
| AT+TLM0_SSID? | `AT+TLM0_SSID?` | ไม่มี | อ่าน SSID | `AT+TLM0_SSID?` | `0` |
| AT+TLM0_SSID= | `AT+TLM0_SSID=<0-15>` | 0-15: SSID | ตั้ง SSID | `AT+TLM0_SSID=0` | `OK` |
| AT+TLM0_PATH? | `AT+TLM0_PATH?` | ไม่มี | อ่าน Path | `AT+TLM0_PATH?` | `0` |
| AT+TLM0_PATH= | `AT+TLM0_PATH=<ค่า>` | ค่า: ประเภท | ตั้ง Path | `AT+TLM0_PATH=0` | `OK` |
| AT+TLM0_DATA_INTERVAL? | `AT+TLM0_DATA_INTERVAL?` | ไม่มี | อ่านช่วงข้อมูล | `AT+TLM0_DATA_INTERVAL?` | `60` (วินาที) |
| AT+TLM0_DATA_INTERVAL= | `AT+TLM0_DATA_INTERVAL=<ค่า>` | ค่า: วินาที | ตั้งช่วงข้อมูล | `AT+TLM0_DATA_INTERVAL=60` | `OK` |
| AT+TLM0_INFO_INTERVAL? | `AT+TLM0_INFO_INTERVAL?` | ไม่มี | อ่านช่วง Info | `AT+TLM0_INFO_INTERVAL?` | `300` (วินาที) |
| AT+TLM0_INFO_INTERVAL= | `AT+TLM0_INFO_INTERVAL=<ค่า>` | ค่า: วินาที | ตั้งช่วง Info | `AT+TLM0_INFO_INTERVAL=300` | `OK` |
| AT+TLM0_BITS_ACTIVE? | `AT+TLM0_BITS_ACTIVE?` | ไม่มี | อ่าน Bits Active | `AT+TLM0_BITS_ACTIVE?` | `0` |
| AT+TLM0_BITS_ACTIVE= | `AT+TLM0_BITS_ACTIVE=<ค่า>` | ค่า: bits | ตั้ง Bits | `AT+TLM0_BITS_ACTIVE=0` | `OK` |
| AT+TLM0_COMMENT? | `AT+TLM0_COMMENT?` | ไม่มี | อ่านข้อความ | `AT+TLM0_COMMENT?` | `Telemetry` |
| AT+TLM0_COMMENT= | `AT+TLM0_COMMENT="<text>"` | text: ข้อความ | ตั้งข้อความ | `AT+TLM0_COMMENT="Telemetry"` | `OK` |

---

## คำสั่ง OLED/Display

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+OLED_ENABLE? | `AT+OLED_ENABLE?` | ไม่มี | อ่านสถานะ OLED | `AT+OLED_ENABLE?` | `1` หรือ `0` |
| AT+OLED_ENABLE= | `AT+OLED_ENABLE=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด OLED | `AT+OLED_ENABLE=1` | `OK` |
| AT+OLED_TIMEOUT? | `AT+OLED_TIMEOUT?` | ไม่มี | อ่าน Timeout | `AT+OLED_TIMEOUT?` | `60` (วินาที) |
| AT+OLED_TIMEOUT= | `AT+OLED_TIMEOUT=<ค่า>` | ค่า: วินาที | ตั้ง Timeout | `AT+OLED_TIMEOUT=60` | `OK` |
| AT+DIM? | `AT+DIM?` | ไม่มี | อ่านระดับ Dimming | `AT+DIM?` | `0` |
| AT+DIM= | `AT+DIM=<ค่า>` | ค่า: ระดับ | ตั้ง Dimming | `AT+DIM=0` | `OK` |
| AT+CONTRAST? | `AT+CONTRAST?` | ไม่มี | อ่าน Contrast | `AT+CONTRAST?` | `0` |
| AT+CONTRAST= | `AT+CONTRAST=<ค่า>` | ค่า: ระดับ | ตั้ง Contrast | `AT+CONTRAST=0` | `OK` |
| AT+STARTUP? | `AT+STARTUP?` | ไม่มี | อ่านหน้าจอเริ่มต้น | `AT+STARTUP?` | `0` |
| AT+STARTUP= | `AT+STARTUP=<ค่า>` | ค่า: ประเภท | ตั้งหน้าจอเริ่มต้น | `AT+STARTUP=0` | `OK` |
| AT+H_UP? | `AT+H_UP?` | ไม่มี | อ่าน H-up Display | `AT+H_UP?` | `1` หรือ `0` |
| AT+H_UP= | `AT+H_UP=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด H-up | `AT+H_UP=1` | `OK` |
| AT+TX_DISPLAY? | `AT+TX_DISPLAY?` | ไม่มี | อ่าน TX Display | `AT+TX_DISPLAY?` | `1` หรือ `0` |
| AT+TX_DISPLAY= | `AT+TX_DISPLAY=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด TX Display | `AT+TX_DISPLAY=1` | `OK` |
| AT+RX_DISPLAY? | `AT+RX_DISPLAY?` | ไม่มี | อ่าน RX Display | `AT+RX_DISPLAY?` | `1` หรือ `0` |
| AT+RX_DISPLAY= | `AT+RX_DISPLAY=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด RX Display | `AT+RX_DISPLAY=1` | `OK` |
| AT+DISPFILTER? | `AT+DISPFILTER?` | ไม่มี | อ่าน Display Filter | `AT+DISPFILTER?` | `0` |
| AT+DISPFILTER= | `AT+DISPFILTER=<ค่า>` | ค่า: ประเภท | ตั้ง Filter | `AT+DISPFILTER=0` | `OK` |
| AT+DISPRF? | `AT+DISPRF?` | ไม่มี | อ่าน RF Display | `AT+DISPRF?` | `1` หรือ `0` |
| AT+DISPRF= | `AT+DISPRF=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด RF Display | `AT+DISPRF=1` | `OK` |
| AT+DISPINET? | `AT+DISPINET?` | ไม่มี | อ่าน INET Display | `AT+DISPINET?` | `1` หรือ `0` |
| AT+DISPINET= | `AT+DISPINET=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด INET Display | `AT+DISPINET=1` | `OK` |
| AT+DISP_FLIP? | `AT+DISP_FLIP?` | ไม่มี | อ่าน Display Flip | `AT+DISP_FLIP?` | `1` หรือ `0` |
| AT+DISP_FLIP= | `AT+DISP_FLIP=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Flip | `AT+DISP_FLIP=1` | `OK` |
| AT+DISP_BRIGHTNESS? | `AT+DISP_BRIGHTNESS?` | ไม่มี | อ่านความสว่าง | `AT+DISP_BRIGHTNESS?` | `100` |
| AT+DISP_BRIGHTNESS= | `AT+DISP_BRIGHTNESS=<ค่า>` | ค่า: % | ตั้งความสว่าง | `AT+DISP_BRIGHTNESS=100` | `OK` |

---

## คำสั่ง Network/Server

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+TX_TIMESLOT? | `AT+TX_TIMESLOT?` | ไม่มี | อ่าน TX Timeslot | `AT+TX_TIMESLOT?` | `0` |
| AT+TX_TIMESLOT= | `AT+TX_TIMESLOT=<ค่า>` | ค่า: timeslot | ตั้ง TX Timeslot | `AT+TX_TIMESLOT=0` | `OK` |
| AT+NTP_HOST? | `AT+NTP_HOST?` | ไม่มี | อ่าน NTP Server | `AT+NTP_HOST?` | `pool.ntp.org` |
| AT+NTP_HOST= | `AT+NTP_HOST="<host>"` | host: NTP | ตั้ง NTP Server | `AT+NTP_HOST="time.google.com"` | `OK` |
| AT+VPN? | `AT+VPN?` | ไม่มี | อ่าน VPN | `AT+VPN?` | `1` หรือ `0` |
| AT+VPN= | `AT+VPN=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด VPN | `AT+VPN=1` | `OK` |
| AT+MODEM? | `AT+MODEM?` | ไม่มี | อ่าน Modem | `AT+MODEM?` | `1` หรือ `0` |
| AT+MODEM= | `AT+MODEM=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Modem | `AT+MODEM=1` | `OK` |
| AT+HOST_NAME? | `AT+HOST_NAME?` | ไม่มี | อ่าน Hostname | `AT+HOST_NAME?` | `esp32aprs` |
| AT+HOST_NAME= | `AT+HOST_NAME="<name>"` | name: hostname | ตั้ง Hostname | `AT+HOST_NAME="myaprs"` | `OK` |
| AT+HTTP_USERNAME? | `AT+HTTP_USERNAME?` | ไม่มี | อ่าน HTTP User | `AT+HTTP_USERNAME?` | `admin` |
| AT+HTTP_USERNAME= | `AT+HTTP_USERNAME="<user>"` | user: username | ตั้ง HTTP User | `AT+HTTP_USERNAME="admin"` | `OK` |
| AT+HTTP_PASSWORD? | `AT+HTTP_PASSWORD?` | ไม่มี | อ่าน HTTP Pass | `AT+HTTP_PASSWORD?` | `password` |
| AT+HTTP_PASSWORD= | `AT+HTTP_PASSWORD="<pass>"` | pass: password | ตั้ง HTTP Pass | `AT+HTTP_PASSWORD="secret"` | `OK` |

---

## คำสั่ง WireGuard

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+WG_PORT? | `AT+WG_PORT?` | ไม่มี | อ่าน WireGuard Port | `AT+WG_PORT?` | `51820` |
| AT+WG_PORT= | `AT+WG_PORT=<ค่า>` | ค่า: port | ตั้ง WG Port | `AT+WG_PORT=51820` | `OK` |
| AT+WG_PEER_ADDRESS? | `AT+WG_PEER_ADDRESS?` | ไม่มี | อ่าน Peer Address | `AT+WG_PEER_ADDRESS?` | `1.2.3.4` |
| AT+WG_PEER_ADDRESS= | `AT+WG_PEER_ADDRESS="<ip>"` | ip: IP Peer | ตั้ง Peer Address | `AT+WG_PEER_ADDRESS="1.2.3.4"` | `OK` |
| AT+WG_LOCAL_ADDRESS? | `AT+WG_LOCAL_ADDRESS?` | ไม่มี | อ่าน Local Address | `AT+WG_LOCAL_ADDRESS?` | `10.0.0.2` |
| AT+WG_LOCAL_ADDRESS= | `AT+WG_LOCAL_ADDRESS="<ip>"` | ip: Local IP | ตั้ง Local Address | `AT+WG_LOCAL_ADDRESS="10.0.0.2"` | `OK` |
| AT+WG_NETMASK_ADDRESS? | `AT+WG_NETMASK_ADDRESS?` | ไม่มี | อ่าน Netmask | `AT+WG_NETMASK_ADDRESS?` | `255.255.255.0` |
| AT+WG_NETMASK_ADDRESS= | `AT+WG_NETMASK_ADDRESS="<mask>"` | mask: netmask | ตั้ง Netmask | `AT+WG_NETMASK_ADDRESS="255.255.255.0"` | `OK` |
| AT+WG_GW_ADDRESS? | `AT+WG_GW_ADDRESS?` | ไม่มี | อ่าน Gateway | `AT+WG_GW_ADDRESS?` | `10.0.0.1` |
| AT+WG_GW_ADDRESS= | `AT+WG_GW_ADDRESS="<ip>"` | ip: Gateway IP | ตั้ง Gateway | `AT+WG_GW_ADDRESS="10.0.0.1"` | `OK` |
| AT+WG_PUBLIC_KEY? | `AT+WG_PUBLIC_KEY?` | ไม่มี | อ่าน Public Key | `AT+WG_PUBLIC_KEY?` | (base64) |
| AT+WG_PUBLIC_KEY= | `AT+WG_PUBLIC_KEY="<key>"` | key: base64 | ตั้ง Public Key | `AT+WG_PUBLIC_KEY="abc..."` | `OK` |
| AT+WG_PRIVATE_KEY? | `AT+WG_PRIVATE_KEY?` | ไม่มี | อ่าน Private Key | `AT+WG_PRIVATE_KEY?` | (base64) |
| AT+WG_PRIVATE_KEY= | `AT+WG_PRIVATE_KEY="<key>"` | key: base64 | ตั้ง Private Key | `AT+WG_PRIVATE_KEY="xyz..."` | `OK` |

---

## คำสั่ง GNSS/GPS

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+GNSS_ENABLE? | `AT+GNSS_ENABLE?` | ไม่มี | อ่านสถานะ GNSS | `AT+GNSS_ENABLE?` | `1` หรือ `0` |
| AT+GNSS_ENABLE= | `AT+GNSS_ENABLE=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด GNSS | `AT+GNSS_ENABLE=1` | `OK` |
| AT+GNSS_PPS_GPIO? | `AT+GNSS_PPS_GPIO?` | ไม่มี | อ่าน PPS GPIO | `AT+GNSS_PPS_GPIO?` | `0` |
| AT+GNSS_PPS_GPIO= | `AT+GNSS_PPS_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง PPS GPIO | `AT+GNSS_PPS_GPIO=0` | `OK` |
| AT+GNSS_CHANNEL? | `AT+GNSS_CHANNEL?` | ไม่มี | อ่าน Channel | `AT+GNSS_CHANNEL?` | `0` |
| AT+GNSS_CHANNEL= | `AT+GNSS_CHANNEL=<ค่า>` | ค่า: channel | ตั้ง Channel | `AT+GNSS_CHANNEL=0` | `OK` |
| AT+GNSS_TCP_PORT? | `AT+GNSS_TCP_PORT?` | ไม่มี | อ่าน TCP Port | `AT+GNSS_TCP_PORT?` | `0` |
| AT+GNSS_TCP_PORT= | `AT+GNSS_TCP_PORT=<ค่า>` | ค่า: port | ตั้ง TCP Port | `AT+GNSS_TCP_PORT=0` | `OK` |
| AT+GNSS_TCP_HOST? | `AT+GNSS_TCP_HOST?` | ไม่มี | อ่าน TCP Host | `AT+GNSS_TCP_HOST?` | `gps.server.com` |
| AT+GNSS_TCP_HOST= | `AT+GNSS_TCP_HOST="<host>"` | host: server | ตั้ง TCP Host | `AT+GNSS_TCP_HOST="gps.server.com"` | `OK` |
| AT+GNSS_AT_COMMAND? | `AT+GNSS_AT_COMMAND?` | ไม่มี | อ่าน AT Command | `AT+GNSS_AT_COMMAND?` | `PMTK...` |
| AT+GNSS_AT_COMMAND= | `AT+GNSS_AT_COMMAND="<cmd>"` | cmd: AT command | ตั้ง AT Command | `AT+GNSS_AT_COMMAND="PMTK..."` | `OK` |

---

## คำสั่ง I2C

### I2C ช่อง 0

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+I2C_ENABLE? | `AT+I2C_ENABLE?` | ไม่มี | อ่านสถานะ I2C | `AT+I2C_ENABLE?` | `1` หรือ `0` |
| AT+I2C_ENABLE= | `AT+I2C_ENABLE=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด I2C | `AT+I2C_ENABLE=1` | `OK` |
| AT+I2C_SDA_PIN? | `AT+I2C_SDA_PIN?` | ไม่มี | อ่าน SDA Pin | `AT+I2C_SDA_PIN?` | `21` |
| AT+I2C_SDA_PIN= | `AT+I2C_SDA_PIN=<ค่า>` | ค่า: GPIO | ตั้ง SDA Pin | `AT+I2C_SDA_PIN=21` | `OK` |
| AT+I2C_SCK_PIN? | `AT+I2C_SCK_PIN?` | ไม่มี | อ่าน SCK Pin | `AT+I2C_SCK_PIN?` | `22` |
| AT+I2C_SCK_PIN= | `AT+I2C_SCK_PIN=<ค่า>` | ค่า: GPIO | ตั้ง SCK Pin | `AT+I2C_SCK_PIN=22` | `OK` |
| AT+I2C_RST_PIN? | `AT+I2C_RST_PIN?` | ไม่มี | อ่าน Reset Pin | `AT+I2C_RST_PIN?` | `0` |
| AT+I2C_RST_PIN= | `AT+I2C_RST_PIN=<ค่า>` | ค่า: GPIO | ตั้ง Reset Pin | `AT+I2C_RST_PIN=0` | `OK` |
| AT+I2C_FREQ? | `AT+I2C_FREQ?` | ไม่มี | อ่านความถี่ | `AT+I2C_FREQ?` | `400000` |
| AT+I2C_FREQ= | `AT+I2C_FREQ=<ค่า>` | ค่า: Hz | ตั้งความถี่ | `AT+I2C_FREQ=400000` | `OK` |

### I2C ช่อง 1

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+I2C1_ENABLE? | `AT+I2C1_ENABLE?` | ไม่มี | อ่านสถานะ I2C1 | `AT+I2C1_ENABLE?` | `1` หรือ `0` |
| AT+I2C1_ENABLE= | `AT+I2C1_ENABLE=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด I2C1 | `AT+I2C1_ENABLE=1` | `OK` |
| AT+I2C1_SDA_PIN? | `AT+I2C1_SDA_PIN?` | ไม่มี | อ่าน SDA Pin | `AT+I2C1_SDA_PIN?` | `0` |
| AT+I2C1_SDA_PIN= | `AT+I2C1_SDA_PIN=<ค่า>` | ค่า: GPIO | ตั้ง SDA Pin | `AT+I2C1_SDA_PIN=0` | `OK` |
| AT+I2C1_SCK_PIN? | `AT+I2C1_SCK_PIN?` | ไม่มี | อ่าน SCK Pin | `AT+I2C1_SCK_PIN?` | `0` |
| AT+I2C1_SCK_PIN= | `AT+I2C1_SCK_PIN=<ค่า>` | ค่า: GPIO | ตั้ง SCK Pin | `AT+I2C1_SCK_PIN=0` | `OK` |
| AT+I2C1_FREQ? | `AT+I2C1_FREQ?` | ไม่มี | อ่านความถี่ | `AT+I2C1_FREQ?` | `400000` |
| AT+I2C1_FREQ= | `AT+I2C1_FREQ=<ค่า>` | ค่า: Hz | ตั้งความถี่ | `AT+I2C1_FREQ=400000` | `OK` |

---

## คำสั่ง UART

### UART0

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+UART0_ENABLE? | `AT+UART0_ENABLE?` | ไม่มี | อ่านสถานะ UART0 | `AT+UART0_ENABLE?` | `1` หรือ `0` |
| AT+UART0_ENABLE= | `AT+UART0_ENABLE=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด UART0 | `AT+UART0_ENABLE=1` | `OK` |
| AT+UART0_TX_GPIO? | `AT+UART0_TX_GPIO?` | ไม่มี | อ่าน TX GPIO | `AT+UART0_TX_GPIO?` | `1` |
| AT+UART0_TX_GPIO= | `AT+UART0_TX_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง TX GPIO | `AT+UART0_TX_GPIO=1` | `OK` |
| AT+UART0_RX_GPIO? | `AT+UART0_RX_GPIO?` | ไม่มี | อ่าน RX GPIO | `AT+UART0_RX_GPIO?` | `3` |
| AT+UART0_RX_GPIO= | `AT+UART0_RX_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง RX GPIO | `AT+UART0_RX_GPIO=3` | `OK` |
| AT+UART0_RTS_GPIO? | `AT+UART0_RTS_GPIO?` | ไม่มี | อ่าน RTS GPIO | `AT+UART0_RTS_GPIO?` | `0` |
| AT+UART0_RTS_GPIO= | `AT+UART0_RTS_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง RTS GPIO | `AT+UART0_RTS_GPIO=0` | `OK` |

### UART1

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+UART1_ENABLE? | `AT+UART1_ENABLE?` | ไม่มี | อ่านสถานะ UART1 | `AT+UART1_ENABLE?` | `1` หรือ `0` |
| AT+UART1_ENABLE= | `AT+UART1_ENABLE=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด UART1 | `AT+UART1_ENABLE=1` | `OK` |
| AT+UART1_TX_GPIO? | `AT+UART1_TX_GPIO?` | ไม่มี | อ่าน TX GPIO | `AT+UART1_TX_GPIO?` | `0` |
| AT+UART1_TX_GPIO= | `AT+UART1_TX_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง TX GPIO | `AT+UART1_TX_GPIO=0` | `OK` |
| AT+UART1_RX_GPIO? | `AT+UART1_RX_GPIO?` | ไม่มี | อ่าน RX GPIO | `AT+UART1_RX_GPIO?` | `0` |
| AT+UART1_RX_GPIO= | `AT+UART1_RX_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง RX GPIO | `AT+UART1_RX_GPIO=0` | `OK` |
| AT+UART1_RTS_GPIO? | `AT+UART1_RTS_GPIO?` | ไม่มี | อ่าน RTS GPIO | `AT+UART1_RTS_GPIO?` | `0` |
| AT+UART1_RTS_GPIO= | `AT+UART1_RTS_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง RTS GPIO | `AT+UART1_RTS_GPIO=0` | `OK` |

### UART2

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+UART2_ENABLE? | `AT+UART2_ENABLE?` | ไม่มี | อ่านสถานะ UART2 | `AT+UART2_ENABLE?` | `1` หรือ `0` |
| AT+UART2_ENABLE= | `AT+UART2_ENABLE=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด UART2 | `AT+UART2_ENABLE=1` | `OK` |
| AT+UART2_TX_GPIO? | `AT+UART2_TX_GPIO?` | ไม่มี | อ่าน TX GPIO | `AT+UART2_TX_GPIO?` | `0` |
| AT+UART2_TX_GPIO= | `AT+UART2_TX_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง TX GPIO | `AT+UART2_TX_GPIO=0` | `OK` |
| AT+UART2_RX_GPIO? | `AT+UART2_RX_GPIO?` | ไม่มี | อ่าน RX GPIO | `AT+UART2_RX_GPIO?` | `0` |
| AT+UART2_RX_GPIO= | `AT+UART2_RX_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง RX GPIO | `AT+UART2_RX_GPIO=0` | `OK` |

---

## คำสั่ง Power Management

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+PWR_EN? | `AT+PWR_EN?` | ไม่มี | อ่านสถานะ Power Mgmt | `AT+PWR_EN?` | `1` หรือ `0` |
| AT+PWR_EN= | `AT+PWR_EN=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด Power Mgmt | `AT+PWR_EN=1` | `OK` |
| AT+PWR_MODE? | `AT+PWR_MODE?` | ไม่มี | อ่านโหมด Power | `AT+PWR_MODE?` | `0` |
| AT+PWR_MODE= | `AT+PWR_MODE=<ค่า>` | ค่า: โหมด | ตั้งโหมด Power | `AT+PWR_MODE=0` | `OK` |
| AT+PWR_SLEEP_INTERVAL? | `AT+PWR_SLEEP_INTERVAL?` | ไม่มี | อ่านช่วง Sleep | `AT+PWR_SLEEP_INTERVAL?` | `0` (วินาที) |
| AT+PWR_SLEEP_INTERVAL= | `AT+PWR_SLEEP_INTERVAL=<ค่า>` | ค่า: วินาที | ตั้งช่วง Sleep | `AT+PWR_SLEEP_INTERVAL=0` | `OK` |
| AT+PWR_STANBY_DELAY? | `AT+PWR_STANBY_DELAY?` | ไม่มี | อ่าน Delay Standby | `AT+PWR_STANBY_DELAY?` | `0` (วินาที) |
| AT+PWR_STANBY_DELAY= | `AT+PWR_STANBY_DELAY=<ค่า>` | ค่า: วินาที | ตั้ง Delay Standby | `AT+PWR_STANBY_DELAY=0` | `OK` |
| AT+PWR_SLEEP_ACTIVATE? | `AT+PWR_SLEEP_ACTIVATE?` | ไม่มี | อ่าน Sleep Activate | `AT+PWR_SLEEP_ACTIVATE?` | `0` |
| AT+PWR_SLEEP_ACTIVATE= | `AT+PWR_SLEEP_ACTIVATE=<ค่า>` | ค่า: activate | ตั้ง Sleep Activate | `AT+PWR_SLEEP_ACTIVATE=0` | `OK` |
| AT+PWR_GPIO? | `AT+PWR_GPIO?` | ไม่มี | อ่าน Power GPIO | `AT+PWR_GPIO?` | `0` |
| AT+PWR_GPIO= | `AT+PWR_GPIO=<ค่า>` | ค่า: GPIO | ตั้ง Power GPIO | `AT+PWR_GPIO=0` | `OK` |
| AT+PWR_ACTIVE? | `AT+PWR_ACTIVE?` | ไม่มี | อ่าน Power Active | `AT+PWR_ACTIVE?` | `1` หรือ `0` |
| AT+PWR_ACTIVE= | `AT+PWR_ACTIVE=<0\|1>` | 0/1: Low/High | ตั้ง Power Active | `AT+PWR_ACTIVE=1` | `OK` |

---

## คำสั่ง MQTT

> **หมายเหตุ:** คำสั่งเหล่านี้ใช้งานได้เฉพาะเมื่อเปิดฟีเจอร์ MQTT

| คำสั่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+EN_MQTT? | `AT+EN_MQTT?` | ไม่มี | อ่านสถานะ MQTT | `AT+EN_MQTT?` | `1` หรือ `0` |
| AT+EN_MQTT= | `AT+EN_MQTT=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด MQTT | `AT+EN_MQTT=1` | `OK` |
| AT+MQTT_HOST? | `AT+MQTT_HOST?` | ไม่มี | อ่าน Broker Host | `AT+MQTT_HOST?` | `broker.hivemq.com` |
| AT+MQTT_HOST= | `AT+MQTT_HOST="<host>"` | host: broker | ตั้ง Broker | `AT+MQTT_HOST="broker.com"` | `OK` |
| AT+MQTT_TOPIC? | `AT+MQTT_TOPIC?` | ไม่มี | อ่าน Topic | `AT+MQTT_TOPIC?` | `aprs/data` |
| AT+MQTT_TOPIC= | `AT+MQTT_TOPIC="<topic>"` | topic: สตริง | ตั้ง Topic | `AT+MQTT_TOPIC="aprs/data"` | `OK` |
| AT+MQTT_SUBSCRIBE? | `AT+MQTT_SUBSCRIBE?` | ไม่มี | อ่าน Subscribe Topic | `AT+MQTT_SUBSCRIBE?` | `aprs/cmd` |
| AT+MQTT_SUBSCRIBE= | `AT+MQTT_SUBSCRIBE="<topic>"` | topic: สตริง | ตั้ง Subscribe | `AT+MQTT_SUBSCRIBE="aprs/cmd"` | `OK` |
| AT+MQTT_USER? | `AT+MQTT_USER?` | ไม่มี | อ่าน Username | `AT+MQTT_USER?` | `myuser` |
| AT+MQTT_USER= | `AT+MQTT_USER="<user>"` | user: username | ตั้ง Username | `AT+MQTT_USER="myuser"` | `OK` |
| AT+MQTT_PASS? | `AT+MQTT_PASS?` | ไม่มี | อ่าน Password | `AT+MQTT_PASS?` | `mypass` |
| AT+MQTT_PASS= | `AT+MQTT_PASS="<pass>"` | pass: password | ตั้ง Password | `AT+MQTT_PASS="mypass"` | `OK` |
| AT+MQTT_PORT? | `AT+MQTT_PORT?` | ไม่มี | อ่าน Port | `AT+MQTT_PORT?` | `1883` |
| AT+MQTT_PORT= | `AT+MQTT_PORT=<ค่า>` | ค่า: port | ตั้ง Port | `AT+MQTT_PORT=1883` | `OK` |
| AT+MQTT_TOPIC_FLAG? | `AT+MQTT_TOPIC_FLAG?` | ไม่มี | อ่าน Topic Flag | `AT+MQTT_TOPIC_FLAG?` | `0` |
| AT+MQTT_TOPIC_FLAG= | `AT+MQTT_TOPIC_FLAG=<ค่า>` | ค่า: flag | ตั้ง Topic Flag | `AT+MQTT_TOPIC_FLAG=0` | `OK` |
| AT+MQTT_SUBSCRIBE_FLAG? | `AT+MQTT_SUBSCRIBE_FLAG?` | ไม่มี | อ่าน Subscribe Flag | `AT+MQTT_SUBSCRIBE_FLAG?` | `0` |
| AT+MQTT_SUBSCRIBE_FLAG= | `AT+MQTT_SUBSCRIBE_FLAG=<ค่า>` | ค่า: flag | ตั้ง Subscribe Flag | `AT+MQTT_SUBSCRIBE_FLAG=0` | `OK` |

---

## คำสั่ง AT Command Interface

| คำส��่ง | ไวยากรณ์ | พารามิเตอร์ | คำอธิบาย | ตัวอย่าง | การตอบกลับ |
|--------|----------|------------|----------|----------|-----------|
| AT+RESET_TIMEOUT? | `AT+RESET_TIMEOUT?` | ไม่มี | อ่าน Reset Timeout | `AT+RESET_TIMEOUT?` | `0` (วินาที) |
| AT+RESET_TIMEOUT= | `AT+RESET_TIMEOUT=<ค่า>` | ค่า: วินาที | ตั้ง Reset Timeout | `AT+RESET_TIMEOUT=0` | `OK` |
| AT+AT_CMD_MQTT? | `AT+AT_CMD_MQTT?` | ไม่มี | อ่าน AT via MQTT | `AT+AT_CMD_MQTT?` | `1` หรือ `0` |
| AT+AT_CMD_MQTT= | `AT+AT_CMD_MQTT=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด AT via MQTT | `AT+AT_CMD_MQTT=1` | `OK` |
| AT+AT_CMD_MSG? | `AT+AT_CMD_MSG?` | ไม่มี | อ่าน AT via Message | `AT+AT_CMD_MSG?` | `1` หรือ `0` |
| AT+AT_CMD_MSG= | `AT+AT_CMD_MSG=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด AT via MSG | `AT+AT_CMD_MSG=1` | `OK` |
| AT+AT_CMD_BLUETOOTH? | `AT+AT_CMD_BLUETOOTH?` | ไม่มี | อ่าน AT via Bluetooth | `AT+AT_CMD_BLUETOOTH?` | `1` หรือ `0` |
| AT+AT_CMD_BLUETOOTH= | `AT+AT_CMD_BLUETOOTH=<0\|1>` | 0/1: ปิด/เปิด | เปิด/ปิด AT via BT | `AT+AT_CMD_BLUETOOTH=1` | `OK` |
| AT+AT_CMD_UART? | `AT+AT_CMD_UART?` | ไม่มี | อ่าน AT via UART | `AT+AT_CMD_UART?` | `0` |
| AT+AT_CMD_UART= | `AT+AT_CMD_UART=<ค่า>` | ค่า: UART channel | ตั้ง AT via UART | `AT+AT_CMD_UART=0` | `OK` |

---

## ภาคผนวก

### รูปแบบคำสั่ง

- **คำสั่งอ่านค่า:** ลงท้ายด้วย `?` เช่น `AT+TIME?`
- **คำสั่งตั้งค่า:** ลงท้ายด้วย `=<ค่า>` เช่น `AT+TIMEZONE=7.0`
- **คำสั่งดำเนินการ:** ไม่มี `?` หรือ `=` เช่น `AT+RESET`

### ค่า Boolean

- `0` = ปิด / ไม่ใช้งาน / False
- `1` = เปิด / ใช้งาน / True

### หมายเหตุสำคัญ

1. คำสั่งบางรายการใช้งานได้เฉพาะเมื่อเปิดฟีเจอร์ที่เกี่ยวข้อง (Bluetooth, RF2, MQTT, PPP)
2. การตั้งค่าส่วนใหญ่จะถูกบันทึกเมื่อใช้คำสั่ง `AT+SAVECONFIG`
3. คำสั่ง `AT+RESET` จะทำให้อุปกรณ์รีบูตทันที

---

**เอกสารฉบับนี้สร้างจากไฟล์:** `handleATCommand.cpp`  
**จำนวนคำสั่งทั้งหมด:** 331+ คำสั่ง
