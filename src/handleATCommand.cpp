#include "Arduino.h"
#include "handleATCommand.h"

void handleATCommand(String cmd) {
  if (cmd == "AT") {
    Serial.println("OK");
    return;
  }

  else if (cmd == "AT+TIMEZONE?") {
    Serial.println(config.timeZone, 6);
  } else if (cmd.startsWith("AT+TIMEZONE=")) {
    config.timeZone = cmd.substring(12).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+SYNCTIME?") {
    Serial.println(config.synctime ? "1" : "0");
  } else if (cmd == "AT+SYNCTIME=1") {
    config.synctime = true;
    Serial.println("OK");
  } else if (cmd == "AT+SYNCTIME=0") {
    config.synctime = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+TITLE?") {
    Serial.println(config.title ? "1" : "0");
  } else if (cmd == "AT+TITLE=1") {
    config.title = true;
    Serial.println("OK");
  } else if (cmd == "AT+TITLE=0") {
    config.title = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+WIFI_MODE?") {
    Serial.println(config.wifi_mode);
  } else if (cmd.startsWith("AT+WIFI_MODE=")) {
    config.wifi_mode = cmd.substring(13).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+WIFI_POWER?") {
    Serial.println(config.wifi_power);
  } else if (cmd.startsWith("AT+WIFI_POWER=")) {
    config.wifi_power = cmd.substring(14).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+WIFI_AP_CH?") {
    Serial.println(config.wifi_ap_ch);
  } else if (cmd.startsWith("AT+WIFI_AP_CH=")) {
    config.wifi_ap_ch = cmd.substring(14).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+WIFI_AP_SSID?") {
    Serial.println(config.wifi_ap_ssid);
  } else if (cmd.startsWith("AT+WIFI_AP_SSID=")) {
    cmd.remove(0, 16);
    cmd.replace("\"", "");
    strncpy(config.wifi_ap_ssid, cmd.c_str(), sizeof(config.wifi_ap_ssid));
    Serial.println("OK");
  }

  else if (cmd == "AT+WIFI_AP_PASS?") {
    Serial.println(config.wifi_ap_pass);
  } else if (cmd.startsWith("AT+WIFI_AP_PASS=")) {
    cmd.remove(0, 16);
    cmd.replace("\"", "");
    strncpy(config.wifi_ap_pass, cmd.c_str(), sizeof(config.wifi_ap_pass));
    Serial.println("OK");
  }

  else if (cmd == "AT+BT_SLAVE?") {
    Serial.println(config.bt_slave ? "1" : "0");
  } else if (cmd == "AT+BT_SLAVE=1") {
    config.bt_slave = true;
    Serial.println("OK");
  } else if (cmd == "AT+BT_SLAVE=0") {
    config.bt_slave = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+BT_MASTER?") {
    Serial.println(config.bt_master ? "1" : "0");
  } else if (cmd == "AT+BT_MASTER=1") {
    config.bt_master = true;
    Serial.println("OK");
  } else if (cmd == "AT+BT_MASTER=0") {
    config.bt_master = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+BT_MODE?") {
    Serial.println(config.bt_mode);
  } else if (cmd.startsWith("AT+BT_MODE=")) {
    config.bt_mode = cmd.substring(11).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+BT_NAME?") {
    Serial.println(config.bt_name);
  } else if (cmd.startsWith("AT+BT_NAME=")) {
    cmd.remove(0, 11);
    cmd.replace("\"", "");
    strncpy(config.bt_name, cmd.c_str(), sizeof(config.bt_name));
    Serial.println("OK");
  }

  else if (cmd == "AT+BT_PIN?") {
    Serial.println(config.bt_pin);
  } else if (cmd.startsWith("AT+BT_PIN=")) {
    config.bt_pin = cmd.substring(10).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+BT_POWER?") {
    Serial.println(config.bt_power);
  } else if (cmd.startsWith("AT+BT_POWER=")) {
    config.bt_power = cmd.substring(12).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+BT_UUID?") {
    Serial.println(config.bt_uuid);
  } else if (cmd.startsWith("AT+BT_UUID=")) {
    cmd.remove(0, 11);
    cmd.replace("\"", "");
    strncpy(config.bt_uuid, cmd.c_str(), sizeof(config.bt_uuid));
    Serial.println("OK");
  }

  else if (cmd == "AT+BT_UUID_RX?") {
    Serial.println(config.bt_uuid_rx);
  } else if (cmd.startsWith("AT+BT_UUID_RX=")) {
    cmd.remove(0, 14);
    cmd.replace("\"", "");
    strncpy(config.bt_uuid_rx, cmd.c_str(), sizeof(config.bt_uuid_rx));
    Serial.println("OK");
  }

  else if (cmd == "AT+BT_UUID_TX?") {
    Serial.println(config.bt_uuid_tx);
  } else if (cmd.startsWith("AT+BT_UUID_TX=")) {
    cmd.remove(0, 14);
    cmd.replace("\"", "");
    strncpy(config.bt_uuid_tx, cmd.c_str(), sizeof(config.bt_uuid_tx));
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_EN?") {
    Serial.println(config.rf_en ? "1" : "0");
  } else if (cmd == "AT+RF_EN=1") {
    config.rf_en = true;
    Serial.println("OK");
  } else if (cmd == "AT+RF_EN=0") {
    config.rf_en = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_TYPE?") {
    Serial.println(config.rf_type);
  } else if (cmd.startsWith("AT+RF_TYPE=")) {
    config.rf_type = cmd.substring(11).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_MODE?") {
    Serial.println(config.rf_mode);
  } else if (cmd.startsWith("AT+RF_MODE=")) {
    config.rf_mode = cmd.substring(11).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_FREQ?") {
    Serial.println(config.rf_freq, 6);
  } else if (cmd.startsWith("AT+RF_FREQ=")) {
    config.rf_freq = cmd.substring(11).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_FREQ_OFFSET?") {
    Serial.println(config.rf_freq_offset);
  } else if (cmd.startsWith("AT+RF_FREQ_OFFSET=")) {
    config.rf_freq_offset = cmd.substring(18).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_BW?") {
    Serial.println(config.rf_bw, 6);
  } else if (cmd.startsWith("AT+RF_BW=")) {
    config.rf_bw = cmd.substring(9).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_BR?") {
    Serial.println(config.rf_br, 6);
  } else if (cmd.startsWith("AT+RF_BR=")) {
    config.rf_br = cmd.substring(9).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_SF?") {
    Serial.println(config.rf_sf);
  } else if (cmd.startsWith("AT+RF_SF=")) {
    config.rf_sf = cmd.substring(9).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_CR?") {
    Serial.println(config.rf_cr);
  } else if (cmd.startsWith("AT+RF_CR=")) {
    config.rf_cr = cmd.substring(9).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_SYNC?") {
    Serial.println(config.rf_sync);
  } else if (cmd.startsWith("AT+RF_SYNC=")) {
    config.rf_sync = cmd.substring(11).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_POWER?") {
    Serial.println(config.rf_power);
  } else if (cmd.startsWith("AT+RF_POWER=")) {
    config.rf_power = cmd.substring(12).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_PREAMABLE?") {
    Serial.println(config.rf_preamable);
  } else if (cmd.startsWith("AT+RF_PREAMABLE=")) {
    config.rf_preamable = cmd.substring(16).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_LNA?") {
    Serial.println(config.rf_lna);
  } else if (cmd.startsWith("AT+RF_LNA=")) {
    config.rf_lna = cmd.substring(10).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_AX25?") {
    Serial.println(config.rf_ax25 ? "1" : "0");
  } else if (cmd == "AT+RF_AX25=1") {
    config.rf_ax25 = true;
    Serial.println("OK");
  } else if (cmd == "AT+RF_AX25=0") {
    config.rf_ax25 = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_SHAPING?") {
    Serial.println(config.rf_shaping);
  } else if (cmd.startsWith("AT+RF_SHAPING=")) {
    config.rf_shaping = cmd.substring(14).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_ENCODING?") {
    Serial.println(config.rf_encoding);
  } else if (cmd.startsWith("AT+RF_ENCODING=")) {
    config.rf_encoding = cmd.substring(15).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF_RX_BOOST?") {
    Serial.println(config.rf_rx_boost ? "1" : "0");
  } else if (cmd == "AT+RF_RX_BOOST=1") {
    config.rf_rx_boost = true;
    Serial.println("OK");
  } else if (cmd == "AT+RF_RX_BOOST=0") {
    config.rf_rx_boost = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_EN?") {
    Serial.println(config.rf1_en ? "1" : "0");
  } else if (cmd == "AT+RF1_EN=1") {
    config.rf1_en = true;
    Serial.println("OK");
  } else if (cmd == "AT+RF1_EN=0") {
    config.rf1_en = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_TYPE?") {
    Serial.println(config.rf1_type);
  } else if (cmd.startsWith("AT+RF1_TYPE=")) {
    config.rf1_type = cmd.substring(12).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_MODE?") {
    Serial.println(config.rf1_mode);
  } else if (cmd.startsWith("AT+RF1_MODE=")) {
    config.rf1_mode = cmd.substring(12).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_FREQ?") {
    Serial.println(config.rf1_freq, 6);
  } else if (cmd.startsWith("AT+RF1_FREQ=")) {
    config.rf1_freq = cmd.substring(12).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_FREQ_OFFSET?") {
    Serial.println(config.rf1_freq_offset);
  } else if (cmd.startsWith("AT+RF1_FREQ_OFFSET=")) {
    config.rf1_freq_offset = cmd.substring(19).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_BW?") {
    Serial.println(config.rf1_bw, 6);
  } else if (cmd.startsWith("AT+RF1_BW=")) {
    config.rf1_bw = cmd.substring(10).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_BR?") {
    Serial.println(config.rf1_br, 6);
  } else if (cmd.startsWith("AT+RF1_BR=")) {
    config.rf1_br = cmd.substring(10).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_SF?") {
    Serial.println(config.rf1_sf);
  } else if (cmd.startsWith("AT+RF1_SF=")) {
    config.rf1_sf = cmd.substring(10).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_CR?") {
    Serial.println(config.rf1_cr);
  } else if (cmd.startsWith("AT+RF1_CR=")) {
    config.rf1_cr = cmd.substring(10).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_SYNC?") {
    Serial.println(config.rf1_sync);
  } else if (cmd.startsWith("AT+RF1_SYNC=")) {
    config.rf1_sync = cmd.substring(12).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_POWER?") {
    Serial.println(config.rf1_power);
  } else if (cmd.startsWith("AT+RF1_POWER=")) {
    config.rf1_power = cmd.substring(13).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_PREAMABLE?") {
    Serial.println(config.rf1_preamable);
  } else if (cmd.startsWith("AT+RF1_PREAMABLE=")) {
    config.rf1_preamable = cmd.substring(17).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_LNA?") {
    Serial.println(config.rf1_lna);
  } else if (cmd.startsWith("AT+RF1_LNA=")) {
    config.rf1_lna = cmd.substring(11).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_AX25?") {
    Serial.println(config.rf1_ax25 ? "1" : "0");
  } else if (cmd == "AT+RF1_AX25=1") {
    config.rf1_ax25 = true;
    Serial.println("OK");
  } else if (cmd == "AT+RF1_AX25=0") {
    config.rf1_ax25 = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_SHAPING?") {
    Serial.println(config.rf1_shaping);
  } else if (cmd.startsWith("AT+RF1_SHAPING=")) {
    config.rf1_shaping = cmd.substring(15).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_ENCODING?") {
    Serial.println(config.rf1_encoding);
  } else if (cmd.startsWith("AT+RF1_ENCODING=")) {
    config.rf1_encoding = cmd.substring(16).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+RF1_RX_BOOST?") {
    Serial.println(config.rf1_rx_boost ? "1" : "0");
  } else if (cmd == "AT+RF1_RX_BOOST=1") {
    config.rf1_rx_boost = true;
    Serial.println("OK");
  } else if (cmd == "AT+RF1_RX_BOOST=0") {
    config.rf1_rx_boost = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_EN?") {
    Serial.println(config.igate_en ? "1" : "0");
  } else if (cmd == "AT+IGATE_EN=1") {
    config.igate_en = true;
    Serial.println("OK");
  } else if (cmd == "AT+IGATE_EN=0") {
    config.igate_en = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+RF2INET?") {
    Serial.println(config.rf2inet ? "1" : "0");
  } else if (cmd == "AT+RF2INET=1") {
    config.rf2inet = true;
    Serial.println("OK");
  } else if (cmd == "AT+RF2INET=0") {
    config.rf2inet = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+INET2RF?") {
    Serial.println(config.inet2rf ? "1" : "0");
  } else if (cmd == "AT+INET2RF=1") {
    config.inet2rf = true;
    Serial.println("OK");
  } else if (cmd == "AT+INET2RF=0") {
    config.inet2rf = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_LOC2RF?") {
    Serial.println(config.igate_loc2rf ? "1" : "0");
  } else if (cmd == "AT+IGATE_LOC2RF=1") {
    config.igate_loc2rf = true;
    Serial.println("OK");
  } else if (cmd == "AT+IGATE_LOC2RF=0") {
    config.igate_loc2rf = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_LOC2INET?") {
    Serial.println(config.igate_loc2inet ? "1" : "0");
  } else if (cmd == "AT+IGATE_LOC2INET=1") {
    config.igate_loc2inet = true;
    Serial.println("OK");
  } else if (cmd == "AT+IGATE_LOC2INET=0") {
    config.igate_loc2inet = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+RF2INETFILTER?") {
    Serial.println(config.rf2inetFilter);
  } else if (cmd.startsWith("AT+RF2INETFILTER=")) {
    config.rf2inetFilter = cmd.substring(17).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+INET2RFFILTER?") {
    Serial.println(config.inet2rfFilter);
  } else if (cmd.startsWith("AT+INET2RFFILTER=")) {
    config.inet2rfFilter = cmd.substring(17).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+APRS_SSID?") {
    Serial.println(config.aprs_ssid);
  } else if (cmd.startsWith("AT+APRS_SSID=")) {
    config.aprs_ssid = cmd.substring(13).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+APRS_PORT?") {
    Serial.println(config.aprs_port);
  } else if (cmd.startsWith("AT+APRS_PORT=")) {
    config.aprs_port = cmd.substring(13).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+APRS_MYCALL?") {
    Serial.println(config.aprs_mycall);
  } else if (cmd.startsWith("AT+APRS_MYCALL=")) {
    cmd.remove(0, 15);
    cmd.replace("\"", "");
    strncpy(config.aprs_mycall, cmd.c_str(), sizeof(config.aprs_mycall));
    Serial.println("OK");
  }

  else if (cmd == "AT+APRS_HOST?") {
    Serial.println(config.aprs_host);
  } else if (cmd.startsWith("AT+APRS_HOST=")) {
    cmd.remove(0, 13);
    cmd.replace("\"", "");
    strncpy(config.aprs_host, cmd.c_str(), sizeof(config.aprs_host));
    Serial.println("OK");
  }

  else if (cmd == "AT+APRS_PASSCODE?") {
    Serial.println(config.aprs_passcode);
  } else if (cmd.startsWith("AT+APRS_PASSCODE=")) {
    cmd.remove(0, 17);
    cmd.replace("\"", "");
    strncpy(config.aprs_passcode, cmd.c_str(), sizeof(config.aprs_passcode));
    Serial.println("OK");
  }

  else if (cmd == "AT+APRS_MONICALL?") {
    Serial.println(config.aprs_moniCall);
  } else if (cmd.startsWith("AT+APRS_MONICALL=")) {
    cmd.remove(0, 17);
    cmd.replace("\"", "");
    strncpy(config.aprs_moniCall, cmd.c_str(), sizeof(config.aprs_moniCall));
    Serial.println("OK");
  }

  else if (cmd == "AT+APRS_FILTER?") {
    Serial.println(config.aprs_filter);
  } else if (cmd.startsWith("AT+APRS_FILTER=")) {
    cmd.remove(0, 15);
    cmd.replace("\"", "");
    strncpy(config.aprs_filter, cmd.c_str(), sizeof(config.aprs_filter));
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_BCN?") {
    Serial.println(config.igate_bcn ? "1" : "0");
  } else if (cmd == "AT+IGATE_BCN=1") {
    config.igate_bcn = true;
    Serial.println("OK");
  } else if (cmd == "AT+IGATE_BCN=0") {
    config.igate_bcn = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_GPS?") {
    Serial.println(config.igate_gps ? "1" : "0");
  } else if (cmd == "AT+IGATE_GPS=1") {
    config.igate_gps = true;
    Serial.println("OK");
  } else if (cmd == "AT+IGATE_GPS=0") {
    config.igate_gps = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_TIMESTAMP?") {
    Serial.println(config.igate_timestamp ? "1" : "0");
  } else if (cmd == "AT+IGATE_TIMESTAMP=1") {
    config.igate_timestamp = true;
    Serial.println("OK");
  } else if (cmd == "AT+IGATE_TIMESTAMP=0") {
    config.igate_timestamp = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_LAT?") {
    Serial.println(config.igate_lat, 6);
  } else if (cmd.startsWith("AT+IGATE_LAT=")) {
    config.igate_lat = cmd.substring(13).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_LON?") {
    Serial.println(config.igate_lon, 6);
  } else if (cmd.startsWith("AT+IGATE_LON=")) {
    config.igate_lon = cmd.substring(13).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_ALT?") {
    Serial.println(config.igate_alt, 6);
  } else if (cmd.startsWith("AT+IGATE_ALT=")) {
    config.igate_alt = cmd.substring(13).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_INTERVAL?") {
    Serial.println(config.igate_interval);
  } else if (cmd.startsWith("AT+IGATE_INTERVAL=")) {
    config.igate_interval = cmd.substring(18).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_OBJECT?") {
    Serial.println(config.igate_object);
  } else if (cmd.startsWith("AT+IGATE_OBJECT=")) {
    cmd.remove(0, 16);
    cmd.replace("\"", "");
    strncpy(config.igate_object, cmd.c_str(), sizeof(config.igate_object));
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_PHG?") {
    Serial.println(config.igate_phg);
  } else if (cmd.startsWith("AT+IGATE_PHG=")) {
    cmd.remove(0, 13);
    cmd.replace("\"", "");
    strncpy(config.igate_phg, cmd.c_str(), sizeof(config.igate_phg));
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_PATH?") {
    Serial.println(config.igate_path);
  } else if (cmd.startsWith("AT+IGATE_PATH=")) {
    config.igate_path = cmd.substring(14).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_COMMENT?") {
    Serial.println(config.igate_comment);
  } else if (cmd.startsWith("AT+IGATE_COMMENT=")) {
    cmd.remove(0, 17);
    cmd.replace("\"", "");
    strncpy(config.igate_comment, cmd.c_str(), sizeof(config.igate_comment));
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_STS_INTERVAL?") {
    Serial.println(config.igate_sts_interval);
  } else if (cmd.startsWith("AT+IGATE_STS_INTERVAL=")) {
    config.igate_sts_interval = cmd.substring(22).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_STATUS?") {
    Serial.println(config.igate_status);
  } else if (cmd.startsWith("AT+IGATE_STATUS=")) {
    cmd.remove(0, 16);
    cmd.replace("\"", "");
    strncpy(config.igate_status, cmd.c_str(), sizeof(config.igate_status));
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_EN?") {
    Serial.println(config.digi_en ? "1" : "0");
  } else if (cmd == "AT+DIGI_EN=1") {
    config.digi_en = true;
    Serial.println("OK");
  } else if (cmd == "AT+DIGI_EN=0") {
    config.digi_en = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_AUTO?") {
    Serial.println(config.digi_auto ? "1" : "0");
  } else if (cmd == "AT+DIGI_AUTO=1") {
    config.digi_auto = true;
    Serial.println("OK");
  } else if (cmd == "AT+DIGI_AUTO=0") {
    config.digi_auto = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_LOC2RF?") {
    Serial.println(config.digi_loc2rf ? "1" : "0");
  } else if (cmd == "AT+DIGI_LOC2RF=1") {
    config.digi_loc2rf = true;
    Serial.println("OK");
  } else if (cmd == "AT+DIGI_LOC2RF=0") {
    config.digi_loc2rf = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_LOC2INET?") {
    Serial.println(config.digi_loc2inet ? "1" : "0");
  } else if (cmd == "AT+DIGI_LOC2INET=1") {
    config.digi_loc2inet = true;
    Serial.println("OK");
  } else if (cmd == "AT+DIGI_LOC2INET=0") {
    config.digi_loc2inet = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_TIMESTAMP?") {
    Serial.println(config.digi_timestamp ? "1" : "0");
  } else if (cmd == "AT+DIGI_TIMESTAMP=1") {
    config.digi_timestamp = true;
    Serial.println("OK");
  } else if (cmd == "AT+DIGI_TIMESTAMP=0") {
    config.digi_timestamp = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_SSID?") {
    Serial.println(config.digi_ssid);
  } else if (cmd.startsWith("AT+DIGI_SSID=")) {
    config.digi_ssid = cmd.substring(13).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_MYCALL?") {
    Serial.println(config.digi_mycall);
  } else if (cmd.startsWith("AT+DIGI_MYCALL=")) {
    cmd.remove(0, 15);
    cmd.replace("\"", "");
    strncpy(config.digi_mycall, cmd.c_str(), sizeof(config.digi_mycall));
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_PATH?") {
    Serial.println(config.digi_path);
  } else if (cmd.startsWith("AT+DIGI_PATH=")) {
    config.digi_path = cmd.substring(13).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_DELAY?") {
    Serial.println(config.digi_delay);
  } else if (cmd.startsWith("AT+DIGI_DELAY=")) {
    config.digi_delay = cmd.substring(14).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGIFILTER?") {
    Serial.println(config.digiFilter);
  } else if (cmd.startsWith("AT+DIGIFILTER=")) {
    config.digiFilter = cmd.substring(14).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_BCN?") {
    Serial.println(config.digi_bcn ? "1" : "0");
  } else if (cmd == "AT+DIGI_BCN=1") {
    config.digi_bcn = true;
    Serial.println("OK");
  } else if (cmd == "AT+DIGI_BCN=0") {
    config.digi_bcn = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_GPS?") {
    Serial.println(config.digi_gps ? "1" : "0");
  } else if (cmd == "AT+DIGI_GPS=1") {
    config.digi_gps = true;
    Serial.println("OK");
  } else if (cmd == "AT+DIGI_GPS=0") {
    config.digi_gps = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_LAT?") {
    Serial.println(config.digi_lat, 6);
  } else if (cmd.startsWith("AT+DIGI_LAT=")) {
    config.digi_lat = cmd.substring(12).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_LON?") {
    Serial.println(config.digi_lon, 6);
  } else if (cmd.startsWith("AT+DIGI_LON=")) {
    config.digi_lon = cmd.substring(12).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_ALT?") {
    Serial.println(config.digi_alt, 6);
  } else if (cmd.startsWith("AT+DIGI_ALT=")) {
    config.digi_alt = cmd.substring(12).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_INTERVAL?") {
    Serial.println(config.digi_interval);
  } else if (cmd.startsWith("AT+DIGI_INTERVAL=")) {
    config.digi_interval = cmd.substring(17).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_PHG?") {
    Serial.println(config.digi_phg);
  } else if (cmd.startsWith("AT+DIGI_PHG=")) {
    cmd.remove(0, 12);
    cmd.replace("\"", "");
    strncpy(config.digi_phg, cmd.c_str(), sizeof(config.digi_phg));
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_COMMENT?") {
    Serial.println(config.digi_comment);
  } else if (cmd.startsWith("AT+DIGI_COMMENT=")) {
    cmd.remove(0, 16);
    cmd.replace("\"", "");
    strncpy(config.digi_comment, cmd.c_str(), sizeof(config.digi_comment));
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_STS_INTERVAL?") {
    Serial.println(config.digi_sts_interval);
  } else if (cmd.startsWith("AT+DIGI_STS_INTERVAL=")) {
    config.digi_sts_interval = cmd.substring(21).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_STATUS?") {
    Serial.println(config.digi_status);
  } else if (cmd.startsWith("AT+DIGI_STATUS=")) {
    cmd.remove(0, 15);
    cmd.replace("\"", "");
    strncpy(config.digi_status, cmd.c_str(), sizeof(config.digi_status));
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_EN?") {
    Serial.println(config.trk_en ? "1" : "0");
  } else if (cmd == "AT+TRK_EN=1") {
    config.trk_en = true;
    Serial.println("OK");
  } else if (cmd == "AT+TRK_EN=0") {
    config.trk_en = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_LOC2RF?") {
    Serial.println(config.trk_loc2rf ? "1" : "0");
  } else if (cmd == "AT+TRK_LOC2RF=1") {
    config.trk_loc2rf = true;
    Serial.println("OK");
  } else if (cmd == "AT+TRK_LOC2RF=0") {
    config.trk_loc2rf = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_LOC2INET?") {
    Serial.println(config.trk_loc2inet ? "1" : "0");
  } else if (cmd == "AT+TRK_LOC2INET=1") {
    config.trk_loc2inet = true;
    Serial.println("OK");
  } else if (cmd == "AT+TRK_LOC2INET=0") {
    config.trk_loc2inet = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_TIMESTAMP?") {
    Serial.println(config.trk_timestamp ? "1" : "0");
  } else if (cmd == "AT+TRK_TIMESTAMP=1") {
    config.trk_timestamp = true;
    Serial.println("OK");
  } else if (cmd == "AT+TRK_TIMESTAMP=0") {
    config.trk_timestamp = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_SSID?") {
    Serial.println(config.trk_ssid);
  } else if (cmd.startsWith("AT+TRK_SSID=")) {
    config.trk_ssid = cmd.substring(12).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_MYCALL?") {
    Serial.println(config.trk_mycall);
  } else if (cmd.startsWith("AT+TRK_MYCALL=")) {
    cmd.remove(0, 14);
    cmd.replace("\"", "");
    strncpy(config.trk_mycall, cmd.c_str(), sizeof(config.trk_mycall));
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_PATH?") {
    Serial.println(config.trk_path);
  } else if (cmd.startsWith("AT+TRK_PATH=")) {
    config.trk_path = cmd.substring(12).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_GPS?") {
    Serial.println(config.trk_gps ? "1" : "0");
  } else if (cmd == "AT+TRK_GPS=1") {
    config.trk_gps = true;
    Serial.println("OK");
  } else if (cmd == "AT+TRK_GPS=0") {
    config.trk_gps = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_LAT?") {
    Serial.println(config.trk_lat, 6);
  } else if (cmd.startsWith("AT+TRK_LAT=")) {
    config.trk_lat = cmd.substring(11).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_LON?") {
    Serial.println(config.trk_lon, 6);
  } else if (cmd.startsWith("AT+TRK_LON=")) {
    config.trk_lon = cmd.substring(11).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_ALT?") {
    Serial.println(config.trk_alt, 6);
  } else if (cmd.startsWith("AT+TRK_ALT=")) {
    config.trk_alt = cmd.substring(11).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_COMMENT?") {
    Serial.println(config.trk_comment);
  } else if (cmd.startsWith("AT+TRK_COMMENT=")) {
    cmd.remove(0, 15);
    cmd.replace("\"", "");
    strncpy(config.trk_comment, cmd.c_str(), sizeof(config.trk_comment));
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_STS_INTERVAL?") {
    Serial.println(config.trk_sts_interval);
  } else if (cmd.startsWith("AT+TRK_STS_INTERVAL=")) {
    config.trk_sts_interval = cmd.substring(20).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_STATUS?") {
    Serial.println(config.trk_status);
  } else if (cmd.startsWith("AT+TRK_STATUS=")) {
    cmd.remove(0, 14);
    cmd.replace("\"", "");
    strncpy(config.trk_status, cmd.c_str(), sizeof(config.trk_status));
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_EN?") {
    Serial.println(config.wx_en ? "1" : "0");
  } else if (cmd == "AT+WX_EN=1") {
    config.wx_en = true;
    Serial.println("OK");
  } else if (cmd == "AT+WX_EN=0") {
    config.wx_en = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_2RF?") {
    Serial.println(config.wx_2rf ? "1" : "0");
  } else if (cmd == "AT+WX_2RF=1") {
    config.wx_2rf = true;
    Serial.println("OK");
  } else if (cmd == "AT+WX_2RF=0") {
    config.wx_2rf = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_2INET?") {
    Serial.println(config.wx_2inet ? "1" : "0");
  } else if (cmd == "AT+WX_2INET=1") {
    config.wx_2inet = true;
    Serial.println("OK");
  } else if (cmd == "AT+WX_2INET=0") {
    config.wx_2inet = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_TIMESTAMP?") {
    Serial.println(config.wx_timestamp ? "1" : "0");
  } else if (cmd == "AT+WX_TIMESTAMP=1") {
    config.wx_timestamp = true;
    Serial.println("OK");
  } else if (cmd == "AT+WX_TIMESTAMP=0") {
    config.wx_timestamp = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_SSID?") {
    Serial.println(config.wx_ssid);
  } else if (cmd.startsWith("AT+WX_SSID=")) {
    config.wx_ssid = cmd.substring(11).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_MYCALL?") {
    Serial.println(config.wx_mycall);
  } else if (cmd.startsWith("AT+WX_MYCALL=")) {
    cmd.remove(0, 13);
    cmd.replace("\"", "");
    strncpy(config.wx_mycall, cmd.c_str(), sizeof(config.wx_mycall));
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_PATH?") {
    Serial.println(config.wx_path);
  } else if (cmd.startsWith("AT+WX_PATH=")) {
    config.wx_path = cmd.substring(11).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_GPS?") {
    Serial.println(config.wx_gps ? "1" : "0");
  } else if (cmd == "AT+WX_GPS=1") {
    config.wx_gps = true;
    Serial.println("OK");
  } else if (cmd == "AT+WX_GPS=0") {
    config.wx_gps = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_LAT?") {
    Serial.println(config.wx_lat, 6);
  } else if (cmd.startsWith("AT+WX_LAT=")) {
    config.wx_lat = cmd.substring(10).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_LON?") {
    Serial.println(config.wx_lon, 6);
  } else if (cmd.startsWith("AT+WX_LON=")) {
    config.wx_lon = cmd.substring(10).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_ALT?") {
    Serial.println(config.wx_alt, 6);
  } else if (cmd.startsWith("AT+WX_ALT=")) {
    config.wx_alt = cmd.substring(10).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_INTERVAL?") {
    Serial.println(config.wx_interval);
  } else if (cmd.startsWith("AT+WX_INTERVAL=")) {
    config.wx_interval = cmd.substring(15).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_FLAGE?") {
    Serial.println(config.wx_flage);
  } else if (cmd.startsWith("AT+WX_FLAGE=")) {
    config.wx_flage = cmd.substring(12).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_OBJECT?") {
    Serial.println(config.wx_object);
  } else if (cmd.startsWith("AT+WX_OBJECT=")) {
    cmd.remove(0, 13);
    cmd.replace("\"", "");
    strncpy(config.wx_object, cmd.c_str(), sizeof(config.wx_object));
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_COMMENT?") {
    Serial.println(config.wx_comment);
  } else if (cmd.startsWith("AT+WX_COMMENT=")) {
    cmd.remove(0, 14);
    cmd.replace("\"", "");
    strncpy(config.wx_comment, cmd.c_str(), sizeof(config.wx_comment));
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM0_EN?") {
    Serial.println(config.tlm0_en ? "1" : "0");
  } else if (cmd == "AT+TLM0_EN=1") {
    config.tlm0_en = true;
    Serial.println("OK");
  } else if (cmd == "AT+TLM0_EN=0") {
    config.tlm0_en = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM0_2RF?") {
    Serial.println(config.tlm0_2rf ? "1" : "0");
  } else if (cmd == "AT+TLM0_2RF=1") {
    config.tlm0_2rf = true;
    Serial.println("OK");
  } else if (cmd == "AT+TLM0_2RF=0") {
    config.tlm0_2rf = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM0_2INET?") {
    Serial.println(config.tlm0_2inet ? "1" : "0");
  } else if (cmd == "AT+TLM0_2INET=1") {
    config.tlm0_2inet = true;
    Serial.println("OK");
  } else if (cmd == "AT+TLM0_2INET=0") {
    config.tlm0_2inet = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM0_SSID?") {
    Serial.println(config.tlm0_ssid);
  } else if (cmd.startsWith("AT+TLM0_SSID=")) {
    config.tlm0_ssid = cmd.substring(13).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM0_MYCALL?") {
    Serial.println(config.tlm0_mycall);
  } else if (cmd.startsWith("AT+TLM0_MYCALL=")) {
    cmd.remove(0, 15);
    cmd.replace("\"", "");
    strncpy(config.tlm0_mycall, cmd.c_str(), sizeof(config.tlm0_mycall));
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM0_PATH?") {
    Serial.println(config.tlm0_path);
  } else if (cmd.startsWith("AT+TLM0_PATH=")) {
    config.tlm0_path = cmd.substring(13).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM0_DATA_INTERVAL?") {
    Serial.println(config.tlm0_data_interval);
  } else if (cmd.startsWith("AT+TLM0_DATA_INTERVAL=")) {
    config.tlm0_data_interval = cmd.substring(22).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM0_INFO_INTERVAL?") {
    Serial.println(config.tlm0_info_interval);
  } else if (cmd.startsWith("AT+TLM0_INFO_INTERVAL=")) {
    config.tlm0_info_interval = cmd.substring(22).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM0_BITS_ACTIVE?") {
    Serial.println(config.tlm0_BITS_Active);
  } else if (cmd.startsWith("AT+TLM0_BITS_ACTIVE=")) {
    config.tlm0_BITS_Active = cmd.substring(20).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM0_COMMENT?") {
    Serial.println(config.tlm0_comment);
  } else if (cmd.startsWith("AT+TLM0_COMMENT=")) {
    cmd.remove(0, 16);
    cmd.replace("\"", "");
    strncpy(config.tlm0_comment, cmd.c_str(), sizeof(config.tlm0_comment));
    Serial.println("OK");
  }

  else if (cmd == "AT+TML0_DATA_CHANNEL?") {
    Serial.println(config.tml0_data_channel);
  } else if (cmd.startsWith("AT+TML0_DATA_CHANNEL=")) {
    config.tml0_data_channel = cmd.substring(21).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM1_EN?") {
    Serial.println(config.tlm1_en ? "1" : "0");
  } else if (cmd == "AT+TLM1_EN=1") {
    config.tlm1_en = true;
    Serial.println("OK");
  } else if (cmd == "AT+TLM1_EN=0") {
    config.tlm1_en = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM1_2RF?") {
    Serial.println(config.tlm1_2rf ? "1" : "0");
  } else if (cmd == "AT+TLM1_2RF=1") {
    config.tlm1_2rf = true;
    Serial.println("OK");
  } else if (cmd == "AT+TLM1_2RF=0") {
    config.tlm1_2rf = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM1_2INET?") {
    Serial.println(config.tlm1_2inet ? "1" : "0");
  } else if (cmd == "AT+TLM1_2INET=1") {
    config.tlm1_2inet = true;
    Serial.println("OK");
  } else if (cmd == "AT+TLM1_2INET=0") {
    config.tlm1_2inet = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM1_SSID?") {
    Serial.println(config.tlm1_ssid);
  } else if (cmd.startsWith("AT+TLM1_SSID=")) {
    config.tlm1_ssid = cmd.substring(13).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM1_MYCALL?") {
    Serial.println(config.tlm1_mycall);
  } else if (cmd.startsWith("AT+TLM1_MYCALL=")) {
    cmd.remove(0, 15);
    cmd.replace("\"", "");
    strncpy(config.tlm1_mycall, cmd.c_str(), sizeof(config.tlm1_mycall));
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM1_PATH?") {
    Serial.println(config.tlm1_path);
  } else if (cmd.startsWith("AT+TLM1_PATH=")) {
    config.tlm1_path = cmd.substring(13).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM1_DATA_INTERVAL?") {
    Serial.println(config.tlm1_data_interval);
  } else if (cmd.startsWith("AT+TLM1_DATA_INTERVAL=")) {
    config.tlm1_data_interval = cmd.substring(22).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM1_INFO_INTERVAL?") {
    Serial.println(config.tlm1_info_interval);
  } else if (cmd.startsWith("AT+TLM1_INFO_INTERVAL=")) {
    config.tlm1_info_interval = cmd.substring(22).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM1_BITS_ACTIVE?") {
    Serial.println(config.tlm1_BITS_Active);
  } else if (cmd.startsWith("AT+TLM1_BITS_ACTIVE=")) {
    config.tlm1_BITS_Active = cmd.substring(20).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TLM1_COMMENT?") {
    Serial.println(config.tlm1_comment);
  } else if (cmd.startsWith("AT+TLM1_COMMENT=")) {
    cmd.remove(0, 16);
    cmd.replace("\"", "");
    strncpy(config.tlm1_comment, cmd.c_str(), sizeof(config.tlm1_comment));
    Serial.println("OK");
  }

  else if (cmd == "AT+TML1_DATA_CHANNEL?") {
    Serial.println(config.tml1_data_channel);
  } else if (cmd.startsWith("AT+TML1_DATA_CHANNEL=")) {
    config.tml1_data_channel = cmd.substring(21).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+OLED_ENABLE?") {
    Serial.println(config.oled_enable ? "1" : "0");
  } else if (cmd == "AT+OLED_ENABLE=1") {
    config.oled_enable = true;
    Serial.println("OK");
  } else if (cmd == "AT+OLED_ENABLE=0") {
    config.oled_enable = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+OLED_TIMEOUT?") {
    Serial.println(config.oled_timeout);
  } else if (cmd.startsWith("AT+OLED_TIMEOUT=")) {
    config.oled_timeout = cmd.substring(16).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+DIM?") {
    Serial.println(config.dim);
  } else if (cmd.startsWith("AT+DIM=")) {
    config.dim = cmd.substring(7).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+CONTRAST?") {
    Serial.println(config.contrast);
  } else if (cmd.startsWith("AT+CONTRAST=")) {
    config.contrast = cmd.substring(12).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+STARTUP?") {
    Serial.println(config.startup);
  } else if (cmd.startsWith("AT+STARTUP=")) {
    config.startup = cmd.substring(11).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+DISPFILTER?") {
    Serial.println(config.dispFilter);
  } else if (cmd.startsWith("AT+DISPFILTER=")) {
    config.dispFilter = cmd.substring(14).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+DISPRF?") {
    Serial.println(config.dispRF ? "1" : "0");
  } else if (cmd == "AT+DISPRF=1") {
    config.dispRF = true;
    Serial.println("OK");
  } else if (cmd == "AT+DISPRF=0") {
    config.dispRF = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+DISPINET?") {
    Serial.println(config.dispINET ? "1" : "0");
  } else if (cmd == "AT+DISPINET=1") {
    config.dispINET = true;
    Serial.println("OK");
  } else if (cmd == "AT+DISPINET=0") {
    config.dispINET = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+TX_TIMESLOT?") {
    Serial.println(config.tx_timeslot);
  } else if (cmd.startsWith("AT+TX_TIMESLOT=")) {
    config.tx_timeslot = cmd.substring(15).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+NTP_HOST?") {
    Serial.println(config.ntp_host);
  } else if (cmd.startsWith("AT+NTP_HOST=")) {
    cmd.remove(0, 12);
    cmd.replace("\"", "");
    strncpy(config.ntp_host, cmd.c_str(), sizeof(config.ntp_host));
    Serial.println("OK");
  }

  else if (cmd == "AT+VPN?") {
    Serial.println(config.vpn ? "1" : "0");
  } else if (cmd == "AT+VPN=1") {
    config.vpn = true;
    Serial.println("OK");
  } else if (cmd == "AT+VPN=0") {
    config.vpn = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+MODEM?") {
    Serial.println(config.modem ? "1" : "0");
  } else if (cmd == "AT+MODEM=1") {
    config.modem = true;
    Serial.println("OK");
  } else if (cmd == "AT+MODEM=0") {
    config.modem = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+WG_PORT?") {
    Serial.println(config.wg_port);
  } else if (cmd.startsWith("AT+WG_PORT=")) {
    config.wg_port = cmd.substring(11).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+WG_PEER_ADDRESS?") {
    Serial.println(config.wg_peer_address);
  } else if (cmd.startsWith("AT+WG_PEER_ADDRESS=")) {
    cmd.remove(0, 19);
    cmd.replace("\"", "");
    strncpy(config.wg_peer_address, cmd.c_str(), sizeof(config.wg_peer_address));
    Serial.println("OK");
  }

  else if (cmd == "AT+WG_LOCAL_ADDRESS?") {
    Serial.println(config.wg_local_address);
  } else if (cmd.startsWith("AT+WG_LOCAL_ADDRESS=")) {
    cmd.remove(0, 20);
    cmd.replace("\"", "");
    strncpy(config.wg_local_address, cmd.c_str(), sizeof(config.wg_local_address));
    Serial.println("OK");
  }

  else if (cmd == "AT+WG_NETMASK_ADDRESS?") {
    Serial.println(config.wg_netmask_address);
  } else if (cmd.startsWith("AT+WG_NETMASK_ADDRESS=")) {
    cmd.remove(0, 22);
    cmd.replace("\"", "");
    strncpy(config.wg_netmask_address, cmd.c_str(), sizeof(config.wg_netmask_address));
    Serial.println("OK");
  }

  else if (cmd == "AT+WG_GW_ADDRESS?") {
    Serial.println(config.wg_gw_address);
  } else if (cmd.startsWith("AT+WG_GW_ADDRESS=")) {
    cmd.remove(0, 17);
    cmd.replace("\"", "");
    strncpy(config.wg_gw_address, cmd.c_str(), sizeof(config.wg_gw_address));
    Serial.println("OK");
  }

  else if (cmd == "AT+WG_PUBLIC_KEY?") {
    Serial.println(config.wg_public_key);
  } else if (cmd.startsWith("AT+WG_PUBLIC_KEY=")) {
    cmd.remove(0, 17);
    cmd.replace("\"", "");
    strncpy(config.wg_public_key, cmd.c_str(), sizeof(config.wg_public_key));
    Serial.println("OK");
  }

  else if (cmd == "AT+WG_PRIVATE_KEY?") {
    Serial.println(config.wg_private_key);
  } else if (cmd.startsWith("AT+WG_PRIVATE_KEY=")) {
    cmd.remove(0, 18);
    cmd.replace("\"", "");
    strncpy(config.wg_private_key, cmd.c_str(), sizeof(config.wg_private_key));
    Serial.println("OK");
  }

  else if (cmd == "AT+HTTP_USERNAME?") {
    Serial.println(config.http_username);
  } else if (cmd.startsWith("AT+HTTP_USERNAME=")) {
    cmd.remove(0, 17);
    cmd.replace("\"", "");
    strncpy(config.http_username, cmd.c_str(), sizeof(config.http_username));
    Serial.println("OK");
  }

  else if (cmd == "AT+HTTP_PASSWORD?") {
    Serial.println(config.http_password);
  } else if (cmd.startsWith("AT+HTTP_PASSWORD=")) {
    cmd.remove(0, 17);
    cmd.replace("\"", "");
    strncpy(config.http_password, cmd.c_str(), sizeof(config.http_password));
    Serial.println("OK");
  }

  else if (cmd == "AT+GNSS_ENABLE?") {
    Serial.println(config.gnss_enable ? "1" : "0");
  } else if (cmd == "AT+GNSS_ENABLE=1") {
    config.gnss_enable = true;
    Serial.println("OK");
  } else if (cmd == "AT+GNSS_ENABLE=0") {
    config.gnss_enable = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+GNSS_TCP_PORT?") {
    Serial.println(config.gnss_tcp_port);
  } else if (cmd.startsWith("AT+GNSS_TCP_PORT=")) {
    config.gnss_tcp_port = cmd.substring(17).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+GNSS_TCP_HOST?") {
    Serial.println(config.gnss_tcp_host);
  } else if (cmd.startsWith("AT+GNSS_TCP_HOST=")) {
    cmd.remove(0, 17);
    cmd.replace("\"", "");
    strncpy(config.gnss_tcp_host, cmd.c_str(), sizeof(config.gnss_tcp_host));
    Serial.println("OK");
  }

  else if (cmd == "AT+GNSS_AT_COMMAND?") {
    Serial.println(config.gnss_at_command);
  } else if (cmd.startsWith("AT+GNSS_AT_COMMAND=")) {
    cmd.remove(0, 19);
    cmd.replace("\"", "");
    strncpy(config.gnss_at_command, cmd.c_str(), sizeof(config.gnss_at_command));
    Serial.println("OK");
  }

  else if (cmd == "AT+I2C_ENABLE?") {
    Serial.println(config.i2c_enable ? "1" : "0");
  } else if (cmd == "AT+I2C_ENABLE=1") {
    config.i2c_enable = true;
    Serial.println("OK");
  } else if (cmd == "AT+I2C_ENABLE=0") {
    config.i2c_enable = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+I2C1_ENABLE?") {
    Serial.println(config.i2c1_enable ? "1" : "0");
  } else if (cmd == "AT+I2C1_ENABLE=1") {
    config.i2c1_enable = true;
    Serial.println("OK");
  } else if (cmd == "AT+I2C1_ENABLE=0") {
    config.i2c1_enable = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+PWR_EN?") {
    Serial.println(config.pwr_en ? "1" : "0");
  } else if (cmd == "AT+PWR_EN=1") {
    config.pwr_en = true;
    Serial.println("OK");
  } else if (cmd == "AT+PWR_EN=0") {
    config.pwr_en = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+PWR_MODE?") {
    Serial.println(config.pwr_mode);
  } else if (cmd.startsWith("AT+PWR_MODE=")) {
    config.pwr_mode = cmd.substring(12).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+PWR_SLEEP_INTERVAL?") {
    Serial.println(config.pwr_sleep_interval);
  } else if (cmd.startsWith("AT+PWR_SLEEP_INTERVAL=")) {
    config.pwr_sleep_interval = cmd.substring(22).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+PWR_STANBY_DELAY?") {
    Serial.println(config.pwr_stanby_delay);
  } else if (cmd.startsWith("AT+PWR_STANBY_DELAY=")) {
    config.pwr_stanby_delay = cmd.substring(20).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+PWR_SLEEP_ACTIVATE?") {
    Serial.println(config.pwr_sleep_activate);
  } else if (cmd.startsWith("AT+PWR_SLEEP_ACTIVATE=")) {
    config.pwr_sleep_activate = cmd.substring(22).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+DISP_FLIP?") {
    Serial.println(config.disp_flip ? "1" : "0");
  } else if (cmd == "AT+DISP_FLIP=1") {
    config.disp_flip = true;
    Serial.println("OK");
  } else if (cmd == "AT+DISP_FLIP=0") {
    config.disp_flip = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+DISP_BRIGHTNESS?") {
    Serial.println(config.disp_brightness);
  } else if (cmd.startsWith("AT+DISP_BRIGHTNESS=")) {
    config.disp_brightness = cmd.substring(19).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_TLM_AVG?") {
    Serial.println(config.trk_tlm_avg ? "1" : "0");
  } else if (cmd == "AT+TRK_TLM_AVG=1") {
    config.trk_tlm_avg = true;
    Serial.println("OK");
  } else if (cmd == "AT+TRK_TLM_AVG=0") {
    config.trk_tlm_avg = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_TLM_SENSOR?") {
    Serial.println(config.trk_tlm_sensor);
  } else if (cmd.startsWith("AT+TRK_TLM_SENSOR=")) {
    config.trk_tlm_sensor = cmd.substring(18).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_TLM_PRECISION?") {
    Serial.println(config.trk_tlm_precision);
  } else if (cmd.startsWith("AT+TRK_TLM_PRECISION=")) {
    config.trk_tlm_precision = cmd.substring(21).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_TLM_OFFSET?") {
    Serial.println(config.trk_tlm_offset, 6);
  } else if (cmd.startsWith("AT+TRK_TLM_OFFSET=")) {
    config.trk_tlm_offset = cmd.substring(18).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_TLM_AVG?") {
    Serial.println(config.digi_tlm_avg ? "1" : "0");
  } else if (cmd == "AT+DIGI_TLM_AVG=1") {
    config.digi_tlm_avg = true;
    Serial.println("OK");
  } else if (cmd == "AT+DIGI_TLM_AVG=0") {
    config.digi_tlm_avg = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_TLM_SENSOR?") {
    Serial.println(config.digi_tlm_sensor);
  } else if (cmd.startsWith("AT+DIGI_TLM_SENSOR=")) {
    config.digi_tlm_sensor = cmd.substring(19).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_TLM_PRECISION?") {
    Serial.println(config.digi_tlm_precision);
  } else if (cmd.startsWith("AT+DIGI_TLM_PRECISION=")) {
    config.digi_tlm_precision = cmd.substring(22).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_TLM_OFFSET?") {
    Serial.println(config.digi_tlm_offset, 6);
  } else if (cmd.startsWith("AT+DIGI_TLM_OFFSET=")) {
    config.digi_tlm_offset = cmd.substring(19).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_TLM_AVG?") {
    Serial.println(config.igate_tlm_avg ? "1" : "0");
  } else if (cmd == "AT+IGATE_TLM_AVG=1") {
    config.igate_tlm_avg = true;
    Serial.println("OK");
  } else if (cmd == "AT+IGATE_TLM_AVG=0") {
    config.igate_tlm_avg = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_TLM_SENSOR?") {
    Serial.println(config.igate_tlm_sensor);
  } else if (cmd.startsWith("AT+IGATE_TLM_SENSOR=")) {
    config.igate_tlm_sensor = cmd.substring(20).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_TLM_PRECISION?") {
    Serial.println(config.igate_tlm_precision);
  } else if (cmd.startsWith("AT+IGATE_TLM_PRECISION=")) {
    config.igate_tlm_precision = cmd.substring(23).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_TLM_OFFSET?") {
    Serial.println(config.igate_tlm_offset, 6);
  } else if (cmd.startsWith("AT+IGATE_TLM_OFFSET=")) {
    config.igate_tlm_offset = cmd.substring(20).toFloat();
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_SENSOR_ENABLE?") {
    Serial.println(config.wx_sensor_enable ? "1" : "0");
  } else if (cmd == "AT+WX_SENSOR_ENABLE=1") {
    config.wx_sensor_enable = true;
    Serial.println("OK");
  } else if (cmd == "AT+WX_SENSOR_ENABLE=0") {
    config.wx_sensor_enable = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_SENSOR_AVG?") {
    Serial.println(config.wx_sensor_avg ? "1" : "0");
  } else if (cmd == "AT+WX_SENSOR_AVG=1") {
    config.wx_sensor_avg = true;
    Serial.println("OK");
  } else if (cmd == "AT+WX_SENSOR_AVG=0") {
    config.wx_sensor_avg = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_SENSOR_CH?") {
    Serial.println(config.wx_sensor_ch);
  } else if (cmd.startsWith("AT+WX_SENSOR_CH=")) {
    config.wx_sensor_ch = cmd.substring(16).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+PPP_APN?") {
    Serial.println(config.ppp_apn);
  } else if (cmd.startsWith("AT+PPP_APN=")) {
    cmd.remove(0, 11);
    cmd.replace("\"", "");
    strncpy(config.ppp_apn, cmd.c_str(), sizeof(config.ppp_apn));
    Serial.println("OK");
  }

  else if (cmd == "AT+PPP_PIN?") {
    Serial.println(config.ppp_pin);
  } else if (cmd.startsWith("AT+PPP_PIN=")) {
    cmd.remove(0, 11);
    cmd.replace("\"", "");
    strncpy(config.ppp_pin, cmd.c_str(), sizeof(config.ppp_pin));
    Serial.println("OK");
  }

  else if (cmd == "AT+EN_MQTT?") {
    Serial.println(config.en_mqtt ? "1" : "0");
  } else if (cmd == "AT+EN_MQTT=1") {
    config.en_mqtt = true;
    Serial.println("OK");
  } else if (cmd == "AT+EN_MQTT=0") {
    config.en_mqtt = false;
    Serial.println("OK");
  }

  else if (cmd == "AT+MQTT_HOST?") {
    Serial.println(config.mqtt_host);
  } else if (cmd.startsWith("AT+MQTT_HOST=")) {
    cmd.remove(0, 13);
    cmd.replace("\"", "");
    strncpy(config.mqtt_host, cmd.c_str(), sizeof(config.mqtt_host));
    Serial.println("OK");
  }

  else if (cmd == "AT+MQTT_TOPIC?") {
    Serial.println(config.mqtt_topic);
  } else if (cmd.startsWith("AT+MQTT_TOPIC=")) {
    cmd.remove(0, 14);
    cmd.replace("\"", "");
    strncpy(config.mqtt_topic, cmd.c_str(), sizeof(config.mqtt_topic));
    Serial.println("OK");
  }

  else if (cmd == "AT+MQTT_SUBSCRIBE?") {
    Serial.println(config.mqtt_subscribe);
  } else if (cmd.startsWith("AT+MQTT_SUBSCRIBE=")) {
    cmd.remove(0, 18);
    cmd.replace("\"", "");
    strncpy(config.mqtt_subscribe, cmd.c_str(), sizeof(config.mqtt_subscribe));
    Serial.println("OK");
  }

  else if (cmd == "AT+MQTT_USER?") {
    Serial.println(config.mqtt_user);
  } else if (cmd.startsWith("AT+MQTT_USER=")) {
    cmd.remove(0, 13);
    cmd.replace("\"", "");
    strncpy(config.mqtt_user, cmd.c_str(), sizeof(config.mqtt_user));
    Serial.println("OK");
  }

  else if (cmd == "AT+MQTT_PASS?") {
    Serial.println(config.mqtt_pass);
  } else if (cmd.startsWith("AT+MQTT_PASS=")) {
    cmd.remove(0, 13);
    cmd.replace("\"", "");
    strncpy(config.mqtt_pass, cmd.c_str(), sizeof(config.mqtt_pass));
    Serial.println("OK");
  }

  else if (cmd == "AT+MQTT_PORT?") {
    Serial.println(config.mqtt_port);
  } else if (cmd.startsWith("AT+MQTT_PORT=")) {
    config.mqtt_port = cmd.substring(13).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+MQTT_TOPIC_FLAG?") {
    Serial.println(config.mqtt_topic_flag);
  } else if (cmd.startsWith("AT+MQTT_TOPIC_FLAG=")) {
    config.mqtt_topic_flag = cmd.substring(19).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+MQTT_SUBSCRIBE_FLAG?") {
    Serial.println(config.mqtt_subscribe_flag);
  } else if (cmd.startsWith("AT+MQTT_SUBSCRIBE_FLAG=")) {
    config.mqtt_subscribe_flag = cmd.substring(23).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_MICE_TYPE?") {
    Serial.println(config.trk_mice_type);
  } else if (cmd.startsWith("AT+TRK_MICE_TYPE=")) {
    config.trk_mice_type = cmd.substring(17).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+TRK_TLM_INTERVAL?") {
    Serial.println(config.trk_tlm_interval);
  } else if (cmd.startsWith("AT+TRK_TLM_INTERVAL=")) {
    config.trk_tlm_interval = cmd.substring(20).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+DIGI_TLM_INTERVAL?") {
    Serial.println(config.digi_tlm_interval);
  } else if (cmd.startsWith("AT+DIGI_TLM_INTERVAL=")) {
    config.digi_tlm_interval = cmd.substring(21).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+IGATE_TLM_INTERVAL?") {
    Serial.println(config.igate_tlm_interval);
  } else if (cmd.startsWith("AT+IGATE_TLM_INTERVAL=")) {
    config.igate_tlm_interval = cmd.substring(22).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+WX_TLM_INTERVAL?") {
    Serial.println(config.wx_tlm_interval);
  } else if (cmd.startsWith("AT+WX_TLM_INTERVAL=")) {
    config.wx_tlm_interval = cmd.substring(19).toInt();
    Serial.println("OK");
  }

  else if (cmd == "AT+HOST_NAME?") {
    Serial.println(config.host_name);
  } else if (cmd.startsWith("AT+HOST_NAME=")) {
    cmd.remove(0, 13);
    cmd.replace("\"", "");
    strncpy(config.host_name, cmd.c_str(), sizeof(config.host_name));
    Serial.println("OK");
  }
  else {
    Serial.println("ERR");
  }
}
