#include "Arduino.h"
#include "handleATCommand.h"
#include "config.h"

extern Configuration config;

String handleATCommand(String cmd) {
  if (cmd == "AT") {
    return "OK";
  }

  else if (cmd == "AT+TIMEZONE?") {
    return String(config.timeZone, 6);
  } else if (cmd.startsWith("AT+TIMEZONE=")) {
    config.timeZone = cmd.substring(12).toFloat();
    return "OK";
  }

  else if (cmd == "AT+SYNCTIME?") {
    return String(config.synctime ? "1" : "0");
  } else if (cmd == "AT+SYNCTIME=1") {
    config.synctime = true;
    return "OK";
  } else if (cmd == "AT+SYNCTIME=0") {
    config.synctime = false;
    return "OK";
  }

  else if (cmd == "AT+TITLE?") {
    return String(config.title ? "1" : "0");
  } else if (cmd == "AT+TITLE=1") {
    config.title = true;
    return "OK";
  } else if (cmd == "AT+TITLE=0") {
    config.title = false;
    return "OK";
  }

  else if (cmd == "AT+WIFI_MODE?") {
    return String(config.wifi_mode);
  } else if (cmd.startsWith("AT+WIFI_MODE=")) {
    config.wifi_mode = cmd.substring(13).toInt();
    return "OK";
  }

  else if (cmd == "AT+WIFI_POWER?") {
    return String(config.wifi_power);
  } else if (cmd.startsWith("AT+WIFI_POWER=")) {
    config.wifi_power = cmd.substring(14).toInt();
    return "OK";
  }

  else if (cmd == "AT+WIFI_AP_CH?") {
    return String(config.wifi_ap_ch);
  } else if (cmd.startsWith("AT+WIFI_AP_CH=")) {
    config.wifi_ap_ch = cmd.substring(14).toInt();
    return "OK";
  }

  else if (cmd == "AT+WIFI_AP_SSID?") {
    return String(config.wifi_ap_ssid);
  } else if (cmd.startsWith("AT+WIFI_AP_SSID=")) {
    cmd.remove(0, 16);
    cmd.replace("\"", "");
    strncpy(config.wifi_ap_ssid, cmd.c_str(), sizeof(config.wifi_ap_ssid));
    return "OK";
  }

  else if (cmd == "AT+WIFI_AP_PASS?") {
    return String(config.wifi_ap_pass);
  } else if (cmd.startsWith("AT+WIFI_AP_PASS=")) {
    cmd.remove(0, 16);
    cmd.replace("\"", "");
    strncpy(config.wifi_ap_pass, cmd.c_str(), sizeof(config.wifi_ap_pass));
    return "OK";
  }

#ifdef BLUETOOTH
  else if (cmd == "AT+BT_SLAVE?") {
    return String(config.bt_slave ? "1" : "0");
  } else if (cmd == "AT+BT_SLAVE=1") {
    config.bt_slave = true;
    return "OK";
  } else if (cmd == "AT+BT_SLAVE=0") {
    config.bt_slave = false;
    return "OK";
  }

  else if (cmd == "AT+BT_MASTER?") {
    return String(config.bt_master ? "1" : "0");
  } else if (cmd == "AT+BT_MASTER=1") {
    config.bt_master = true;
    return "OK";
  } else if (cmd == "AT+BT_MASTER=0") {
    config.bt_master = false;
    return "OK";
  }

  else if (cmd == "AT+BT_MODE?") {
    return String(config.bt_mode);
  } else if (cmd.startsWith("AT+BT_MODE=")) {
    config.bt_mode = cmd.substring(11).toInt();
    return "OK";
  }

  else if (cmd == "AT+BT_NAME?") {
    return String(config.bt_name);
  } else if (cmd.startsWith("AT+BT_NAME=")) {
    cmd.remove(0, 11);
    cmd.replace("\"", "");
    strncpy(config.bt_name, cmd.c_str(), sizeof(config.bt_name));
    return "OK";
  }

  else if (cmd == "AT+BT_PIN?") {
    return String(config.bt_pin);
  } else if (cmd.startsWith("AT+BT_PIN=")) {
    config.bt_pin = cmd.substring(10).toInt();
    return "OK";
  }

  else if (cmd == "AT+BT_POWER?") {
    return String(config.bt_power);
  } else if (cmd.startsWith("AT+BT_POWER=")) {
    config.bt_power = cmd.substring(12).toInt();
    return "OK";
  }

  else if (cmd == "AT+BT_UUID?") {
    return String(config.bt_uuid);
  } else if (cmd.startsWith("AT+BT_UUID=")) {
    cmd.remove(0, 11);
    cmd.replace("\"", "");
    strncpy(config.bt_uuid, cmd.c_str(), sizeof(config.bt_uuid));
    return "OK";
  }

  else if (cmd == "AT+BT_UUID_RX?") {
    return String(config.bt_uuid_rx);
  } else if (cmd.startsWith("AT+BT_UUID_RX=")) {
    cmd.remove(0, 14);
    cmd.replace("\"", "");
    strncpy(config.bt_uuid_rx, cmd.c_str(), sizeof(config.bt_uuid_rx));
    return "OK";
  }

  else if (cmd == "AT+BT_UUID_TX?") {
    return String(config.bt_uuid_tx);
  } else if (cmd.startsWith("AT+BT_UUID_TX=")) {
    cmd.remove(0, 14);
    cmd.replace("\"", "");
    strncpy(config.bt_uuid_tx, cmd.c_str(), sizeof(config.bt_uuid_tx));
    return "OK";
  }
#endif

  else if (cmd == "AT+RF_EN?") {
    return String(config.rf_en ? "1" : "0");
  } else if (cmd == "AT+RF_EN=1") {
    config.rf_en = true;
    return "OK";
  } else if (cmd == "AT+RF_EN=0") {
    config.rf_en = false;
    return "OK";
  }

  else if (cmd == "AT+RF_TYPE?") {
    return String(config.rf_type);
  } else if (cmd.startsWith("AT+RF_TYPE=")) {
    config.rf_type = cmd.substring(11).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF_MODE?") {
    return String(config.rf_mode);
  } else if (cmd.startsWith("AT+RF_MODE=")) {
    config.rf_mode = cmd.substring(11).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF_FREQ?") {
    return String(config.rf_freq, 6);
  } else if (cmd.startsWith("AT+RF_FREQ=")) {
    config.rf_freq = cmd.substring(11).toFloat();
    return "OK";
  }

  else if (cmd == "AT+RF_FREQ_OFFSET?") {
    return String(config.rf_freq_offset);
  } else if (cmd.startsWith("AT+RF_FREQ_OFFSET=")) {
    config.rf_freq_offset = cmd.substring(18).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF_BW?") {
    return String(config.rf_bw, 6);
  } else if (cmd.startsWith("AT+RF_BW=")) {
    config.rf_bw = cmd.substring(9).toFloat();
    return "OK";
  }

  else if (cmd == "AT+RF_BR?") {
    return String(config.rf_br, 6);
  } else if (cmd.startsWith("AT+RF_BR=")) {
    config.rf_br = cmd.substring(9).toFloat();
    return "OK";
  }

  else if (cmd == "AT+RF_SF?") {
    return String(config.rf_sf);
  } else if (cmd.startsWith("AT+RF_SF=")) {
    config.rf_sf = cmd.substring(9).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF_CR?") {
    return String(config.rf_cr);
  } else if (cmd.startsWith("AT+RF_CR=")) {
    config.rf_cr = cmd.substring(9).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF_SYNC?") {
    return String(config.rf_sync);
  } else if (cmd.startsWith("AT+RF_SYNC=")) {
    config.rf_sync = cmd.substring(11).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF_POWER?") {
    return String(config.rf_power);
  } else if (cmd.startsWith("AT+RF_POWER=")) {
    config.rf_power = cmd.substring(12).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF_PREAMABLE?") {
    return String(config.rf_preamable);
  } else if (cmd.startsWith("AT+RF_PREAMABLE=")) {
    config.rf_preamable = cmd.substring(16).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF_LNA?") {
    return String(config.rf_lna);
  } else if (cmd.startsWith("AT+RF_LNA=")) {
    config.rf_lna = cmd.substring(10).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF_AX25?") {
    return String(config.rf_ax25 ? "1" : "0");
  } else if (cmd == "AT+RF_AX25=1") {
    config.rf_ax25 = true;
    return "OK";
  } else if (cmd == "AT+RF_AX25=0") {
    config.rf_ax25 = false;
    return "OK";
  }

  else if (cmd == "AT+RF_SHAPING?") {
    return String(config.rf_shaping);
  } else if (cmd.startsWith("AT+RF_SHAPING=")) {
    config.rf_shaping = cmd.substring(14).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF_ENCODING?") {
    return String(config.rf_encoding);
  } else if (cmd.startsWith("AT+RF_ENCODING=")) {
    config.rf_encoding = cmd.substring(15).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF_RX_BOOST?") {
    return String(config.rf_rx_boost ? "1" : "0");
  } else if (cmd == "AT+RF_RX_BOOST=1") {
    config.rf_rx_boost = true;
    return "OK";
  } else if (cmd == "AT+RF_RX_BOOST=0") {
    config.rf_rx_boost = false;
    return "OK";
  }

  else if (cmd == "AT+RF1_EN?") {
    return String(config.rf1_en ? "1" : "0");
  } else if (cmd == "AT+RF1_EN=1") {
    config.rf1_en = true;
    return "OK";
  } else if (cmd == "AT+RF1_EN=0") {
    config.rf1_en = false;
    return "OK";
  }

  else if (cmd == "AT+RF1_TYPE?") {
    return String(config.rf1_type);
  } else if (cmd.startsWith("AT+RF1_TYPE=")) {
    config.rf1_type = cmd.substring(12).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF1_MODE?") {
    return String(config.rf1_mode);
  } else if (cmd.startsWith("AT+RF1_MODE=")) {
    config.rf1_mode = cmd.substring(12).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF1_FREQ?") {
    return String(config.rf1_freq, 6);
  } else if (cmd.startsWith("AT+RF1_FREQ=")) {
    config.rf1_freq = cmd.substring(12).toFloat();
    return "OK";
  }

  else if (cmd == "AT+RF1_FREQ_OFFSET?") {
    return String(config.rf1_freq_offset);
  } else if (cmd.startsWith("AT+RF1_FREQ_OFFSET=")) {
    config.rf1_freq_offset = cmd.substring(19).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF1_BW?") {
    return String(config.rf1_bw, 6);
  } else if (cmd.startsWith("AT+RF1_BW=")) {
    config.rf1_bw = cmd.substring(10).toFloat();
    return "OK";
  }

  else if (cmd == "AT+RF1_BR?") {
    return String(config.rf1_br, 6);
  } else if (cmd.startsWith("AT+RF1_BR=")) {
    config.rf1_br = cmd.substring(10).toFloat();
    return "OK";
  }

  else if (cmd == "AT+RF1_SF?") {
    return String(config.rf1_sf);
  } else if (cmd.startsWith("AT+RF1_SF=")) {
    config.rf1_sf = cmd.substring(10).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF1_CR?") {
    return String(config.rf1_cr);
  } else if (cmd.startsWith("AT+RF1_CR=")) {
    config.rf1_cr = cmd.substring(10).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF1_SYNC?") {
    return String(config.rf1_sync);
  } else if (cmd.startsWith("AT+RF1_SYNC=")) {
    config.rf1_sync = cmd.substring(12).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF1_POWER?") {
    return String(config.rf1_power);
  } else if (cmd.startsWith("AT+RF1_POWER=")) {
    config.rf1_power = cmd.substring(13).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF1_PREAMABLE?") {
    return String(config.rf1_preamable);
  } else if (cmd.startsWith("AT+RF1_PREAMABLE=")) {
    config.rf1_preamable = cmd.substring(17).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF1_LNA?") {
    return String(config.rf1_lna);
  } else if (cmd.startsWith("AT+RF1_LNA=")) {
    config.rf1_lna = cmd.substring(11).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF1_AX25?") {
    return String(config.rf1_ax25 ? "1" : "0");
  } else if (cmd == "AT+RF1_AX25=1") {
    config.rf1_ax25 = true;
    return "OK";
  } else if (cmd == "AT+RF1_AX25=0") {
    config.rf1_ax25 = false;
    return "OK";
  }

  else if (cmd == "AT+RF1_SHAPING?") {
    return String(config.rf1_shaping);
  } else if (cmd.startsWith("AT+RF1_SHAPING=")) {
    config.rf1_shaping = cmd.substring(15).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF1_ENCODING?") {
    return String(config.rf1_encoding);
  } else if (cmd.startsWith("AT+RF1_ENCODING=")) {
    config.rf1_encoding = cmd.substring(16).toInt();
    return "OK";
  }

  else if (cmd == "AT+RF1_RX_BOOST?") {
    return String(config.rf1_rx_boost ? "1" : "0");
  } else if (cmd == "AT+RF1_RX_BOOST=1") {
    config.rf1_rx_boost = true;
    return "OK";
  } else if (cmd == "AT+RF1_RX_BOOST=0") {
    config.rf1_rx_boost = false;
    return "OK";
  }

  else if (cmd == "AT+IGATE_EN?") {
    return String(config.igate_en ? "1" : "0");
  } else if (cmd == "AT+IGATE_EN=1") {
    config.igate_en = true;
    return "OK";
  } else if (cmd == "AT+IGATE_EN=0") {
    config.igate_en = false;
    return "OK";
  }

  else if (cmd == "AT+RF2INET?") {
    return String(config.rf2inet ? "1" : "0");
  } else if (cmd == "AT+RF2INET=1") {
    config.rf2inet = true;
    return "OK";
  } else if (cmd == "AT+RF2INET=0") {
    config.rf2inet = false;
    return "OK";
  }

  else if (cmd == "AT+INET2RF?") {
    return String(config.inet2rf ? "1" : "0");
  } else if (cmd == "AT+INET2RF=1") {
    config.inet2rf = true;
    return "OK";
  } else if (cmd == "AT+INET2RF=0") {
    config.inet2rf = false;
    return "OK";
  }

  else if (cmd == "AT+IGATE_LOC2RF?") {
    return String(config.igate_loc2rf ? "1" : "0");
  } else if (cmd == "AT+IGATE_LOC2RF=1") {
    config.igate_loc2rf = true;
    return "OK";
  } else if (cmd == "AT+IGATE_LOC2RF=0") {
    config.igate_loc2rf = false;
    return "OK";
  }

  else if (cmd == "AT+IGATE_LOC2INET?") {
    return String(config.igate_loc2inet ? "1" : "0");
  } else if (cmd == "AT+IGATE_LOC2INET=1") {
    config.igate_loc2inet = true;
    return "OK";
  } else if (cmd == "AT+IGATE_LOC2INET=0") {
    config.igate_loc2inet = false;
    return "OK";
  }

  else if (cmd == "AT+RF2INETFILTER?") {
    return String(config.rf2inetFilter);
  } else if (cmd.startsWith("AT+RF2INETFILTER=")) {
    config.rf2inetFilter = cmd.substring(17).toInt();
    return "OK";
  }

  else if (cmd == "AT+INET2RFFILTER?") {
    return String(config.inet2rfFilter);
  } else if (cmd.startsWith("AT+INET2RFFILTER=")) {
    config.inet2rfFilter = cmd.substring(17).toInt();
    return "OK";
  }

  else if (cmd == "AT+APRS_SSID?") {
    return String(config.aprs_ssid);
  } else if (cmd.startsWith("AT+APRS_SSID=")) {
    config.aprs_ssid = cmd.substring(13).toInt();
    return "OK";
  }

  else if (cmd == "AT+APRS_PORT?") {
    return String(config.aprs_port);
  } else if (cmd.startsWith("AT+APRS_PORT=")) {
    config.aprs_port = cmd.substring(13).toInt();
    return "OK";
  }

  else if (cmd == "AT+APRS_MYCALL?") {
    return String(config.aprs_mycall);
  } else if (cmd.startsWith("AT+APRS_MYCALL=")) {
    cmd.remove(0, 15);
    cmd.replace("\"", "");
    strncpy(config.aprs_mycall, cmd.c_str(), sizeof(config.aprs_mycall));
    return "OK";
  }

  else if (cmd == "AT+APRS_HOST?") {
    return String(config.aprs_host);
  } else if (cmd.startsWith("AT+APRS_HOST=")) {
    cmd.remove(0, 13);
    cmd.replace("\"", "");
    strncpy(config.aprs_host, cmd.c_str(), sizeof(config.aprs_host));
    return "OK";
  }

  else if (cmd == "AT+APRS_PASSCODE?") {
    return String(config.aprs_passcode);
  } else if (cmd.startsWith("AT+APRS_PASSCODE=")) {
    cmd.remove(0, 17);
    cmd.replace("\"", "");
    strncpy(config.aprs_passcode, cmd.c_str(), sizeof(config.aprs_passcode));
    return "OK";
  }

  else if (cmd == "AT+APRS_MONICALL?") {
    return String(config.aprs_moniCall);
  } else if (cmd.startsWith("AT+APRS_MONICALL=")) {
    cmd.remove(0, 17);
    cmd.replace("\"", "");
    strncpy(config.aprs_moniCall, cmd.c_str(), sizeof(config.aprs_moniCall));
    return "OK";
  }

  else if (cmd == "AT+APRS_FILTER?") {
    return String(config.aprs_filter);
  } else if (cmd.startsWith("AT+APRS_FILTER=")) {
    cmd.remove(0, 15);
    cmd.replace("\"", "");
    strncpy(config.aprs_filter, cmd.c_str(), sizeof(config.aprs_filter));
    return "OK";
  }

  else if (cmd == "AT+IGATE_BCN?") {
    return String(config.igate_bcn ? "1" : "0");
  } else if (cmd == "AT+IGATE_BCN=1") {
    config.igate_bcn = true;
    return "OK";
  } else if (cmd == "AT+IGATE_BCN=0") {
    config.igate_bcn = false;
    return "OK";
  }

  else if (cmd == "AT+IGATE_GPS?") {
    return String(config.igate_gps ? "1" : "0");
  } else if (cmd == "AT+IGATE_GPS=1") {
    config.igate_gps = true;
    return "OK";
  } else if (cmd == "AT+IGATE_GPS=0") {
    config.igate_gps = false;
    return "OK";
  }

  else if (cmd == "AT+IGATE_TIMESTAMP?") {
    return String(config.igate_timestamp ? "1" : "0");
  } else if (cmd == "AT+IGATE_TIMESTAMP=1") {
    config.igate_timestamp = true;
    return "OK";
  } else if (cmd == "AT+IGATE_TIMESTAMP=0") {
    config.igate_timestamp = false;
    return "OK";
  }

  else if (cmd == "AT+IGATE_LAT?") {
    return String(config.igate_lat, 6);
  } else if (cmd.startsWith("AT+IGATE_LAT=")) {
    config.igate_lat = cmd.substring(13).toFloat();
    return "OK";
  }

  else if (cmd == "AT+IGATE_LON?") {
    return String(config.igate_lon, 6);
  } else if (cmd.startsWith("AT+IGATE_LON=")) {
    config.igate_lon = cmd.substring(13).toFloat();
    return "OK";
  }

  else if (cmd == "AT+IGATE_ALT?") {
    return String(config.igate_alt, 6);
  } else if (cmd.startsWith("AT+IGATE_ALT=")) {
    config.igate_alt = cmd.substring(13).toFloat();
    return "OK";
  }

  else if (cmd == "AT+IGATE_INTERVAL?") {
    return String(config.igate_interval);
  } else if (cmd.startsWith("AT+IGATE_INTERVAL=")) {
    config.igate_interval = cmd.substring(18).toInt();
    return "OK";
  }

  else if (cmd == "AT+IGATE_OBJECT?") {
    return String(config.igate_object);
  } else if (cmd.startsWith("AT+IGATE_OBJECT=")) {
    cmd.remove(0, 16);
    cmd.replace("\"", "");
    strncpy(config.igate_object, cmd.c_str(), sizeof(config.igate_object));
    return "OK";
  }

  else if (cmd == "AT+IGATE_PHG?") {
    return String(config.igate_phg);
  } else if (cmd.startsWith("AT+IGATE_PHG=")) {
    cmd.remove(0, 13);
    cmd.replace("\"", "");
    strncpy(config.igate_phg, cmd.c_str(), sizeof(config.igate_phg));
    return "OK";
  }

  else if (cmd == "AT+IGATE_PATH?") {
    return String(config.igate_path);
  } else if (cmd.startsWith("AT+IGATE_PATH=")) {
    config.igate_path = cmd.substring(14).toInt();
    return "OK";
  }

  else if (cmd == "AT+IGATE_COMMENT?") {
    return String(config.igate_comment);
  } else if (cmd.startsWith("AT+IGATE_COMMENT=")) {
    cmd.remove(0, 17);
    cmd.replace("\"", "");
    strncpy(config.igate_comment, cmd.c_str(), sizeof(config.igate_comment));
    return "OK";
  }

  else if (cmd == "AT+IGATE_STS_INTERVAL?") {
    return String(config.igate_sts_interval);
  } else if (cmd.startsWith("AT+IGATE_STS_INTERVAL=")) {
    config.igate_sts_interval = cmd.substring(22).toInt();
    return "OK";
  }

  else if (cmd == "AT+IGATE_STATUS?") {
    return String(config.igate_status);
  } else if (cmd.startsWith("AT+IGATE_STATUS=")) {
    cmd.remove(0, 16);
    cmd.replace("\"", "");
    strncpy(config.igate_status, cmd.c_str(), sizeof(config.igate_status));
    return "OK";
  }

  else if (cmd == "AT+DIGI_EN?") {
    return String(config.digi_en ? "1" : "0");
  } else if (cmd == "AT+DIGI_EN=1") {
    config.digi_en = true;
    return "OK";
  } else if (cmd == "AT+DIGI_EN=0") {
    config.digi_en = false;
    return "OK";
  }

  else if (cmd == "AT+DIGI_AUTO?") {
    return String(config.digi_auto ? "1" : "0");
  } else if (cmd == "AT+DIGI_AUTO=1") {
    config.digi_auto = true;
    return "OK";
  } else if (cmd == "AT+DIGI_AUTO=0") {
    config.digi_auto = false;
    return "OK";
  }

  else if (cmd == "AT+DIGI_LOC2RF?") {
    return String(config.digi_loc2rf ? "1" : "0");
  } else if (cmd == "AT+DIGI_LOC2RF=1") {
    config.digi_loc2rf = true;
    return "OK";
  } else if (cmd == "AT+DIGI_LOC2RF=0") {
    config.digi_loc2rf = false;
    return "OK";
  }

  else if (cmd == "AT+DIGI_LOC2INET?") {
    return String(config.digi_loc2inet ? "1" : "0");
  } else if (cmd == "AT+DIGI_LOC2INET=1") {
    config.digi_loc2inet = true;
    return "OK";
  } else if (cmd == "AT+DIGI_LOC2INET=0") {
    config.digi_loc2inet = false;
    return "OK";
  }

  else if (cmd == "AT+DIGI_TIMESTAMP?") {
    return String(config.digi_timestamp ? "1" : "0");
  } else if (cmd == "AT+DIGI_TIMESTAMP=1") {
    config.digi_timestamp = true;
    return "OK";
  } else if (cmd == "AT+DIGI_TIMESTAMP=0") {
    config.digi_timestamp = false;
    return "OK";
  }

  else if (cmd == "AT+DIGI_SSID?") {
    return String(config.digi_ssid);
  } else if (cmd.startsWith("AT+DIGI_SSID=")) {
    config.digi_ssid = cmd.substring(13).toInt();
    return "OK";
  }

  else if (cmd == "AT+DIGI_MYCALL?") {
    return String(config.digi_mycall);
  } else if (cmd.startsWith("AT+DIGI_MYCALL=")) {
    cmd.remove(0, 15);
    cmd.replace("\"", "");
    strncpy(config.digi_mycall, cmd.c_str(), sizeof(config.digi_mycall));
    return "OK";
  }

  else if (cmd == "AT+DIGI_PATH?") {
    return String(config.digi_path);
  } else if (cmd.startsWith("AT+DIGI_PATH=")) {
    config.digi_path = cmd.substring(13).toInt();
    return "OK";
  }

  else if (cmd == "AT+DIGI_DELAY?") {
    return String(config.digi_delay);
  } else if (cmd.startsWith("AT+DIGI_DELAY=")) {
    config.digi_delay = cmd.substring(14).toInt();
    return "OK";
  }

  else if (cmd == "AT+DIGIFILTER?") {
    return String(config.digiFilter);
  } else if (cmd.startsWith("AT+DIGIFILTER=")) {
    config.digiFilter = cmd.substring(14).toInt();
    return "OK";
  }

  else if (cmd == "AT+DIGI_BCN?") {
    return String(config.digi_bcn ? "1" : "0");
  } else if (cmd == "AT+DIGI_BCN=1") {
    config.digi_bcn = true;
    return "OK";
  } else if (cmd == "AT+DIGI_BCN=0") {
    config.digi_bcn = false;
    return "OK";
  }

  else if (cmd == "AT+DIGI_GPS?") {
    return String(config.digi_gps ? "1" : "0");
  } else if (cmd == "AT+DIGI_GPS=1") {
    config.digi_gps = true;
    return "OK";
  } else if (cmd == "AT+DIGI_GPS=0") {
    config.digi_gps = false;
    return "OK";
  }

  else if (cmd == "AT+DIGI_LAT?") {
    return String(config.digi_lat, 6);
  } else if (cmd.startsWith("AT+DIGI_LAT=")) {
    config.digi_lat = cmd.substring(12).toFloat();
    return "OK";
  }

  else if (cmd == "AT+DIGI_LON?") {
    return String(config.digi_lon, 6);
  } else if (cmd.startsWith("AT+DIGI_LON=")) {
    config.digi_lon = cmd.substring(12).toFloat();
    return "OK";
  }

  else if (cmd == "AT+DIGI_ALT?") {
    return String(config.digi_alt, 6);
  } else if (cmd.startsWith("AT+DIGI_ALT=")) {
    config.digi_alt = cmd.substring(12).toFloat();
    return "OK";
  }

  else if (cmd == "AT+DIGI_INTERVAL?") {
    return String(config.digi_interval);
  } else if (cmd.startsWith("AT+DIGI_INTERVAL=")) {
    config.digi_interval = cmd.substring(17).toInt();
    return "OK";
  }

  else if (cmd == "AT+DIGI_PHG?") {
    return String(config.digi_phg);
  } else if (cmd.startsWith("AT+DIGI_PHG=")) {
    cmd.remove(0, 12);
    cmd.replace("\"", "");
    strncpy(config.digi_phg, cmd.c_str(), sizeof(config.digi_phg));
    return "OK";
  }

  else if (cmd == "AT+DIGI_COMMENT?") {
    return String(config.digi_comment);
  } else if (cmd.startsWith("AT+DIGI_COMMENT=")) {
    cmd.remove(0, 16);
    cmd.replace("\"", "");
    strncpy(config.digi_comment, cmd.c_str(), sizeof(config.digi_comment));
    return "OK";
  }

  else if (cmd == "AT+DIGI_STS_INTERVAL?") {
    return String(config.digi_sts_interval);
  } else if (cmd.startsWith("AT+DIGI_STS_INTERVAL=")) {
    config.digi_sts_interval = cmd.substring(21).toInt();
    return "OK";
  }

  else if (cmd == "AT+DIGI_STATUS?") {
    return String(config.digi_status);
  } else if (cmd.startsWith("AT+DIGI_STATUS=")) {
    cmd.remove(0, 15);
    cmd.replace("\"", "");
    strncpy(config.digi_status, cmd.c_str(), sizeof(config.digi_status));
    return "OK";
  }

  else if (cmd == "AT+TRK_EN?") {
    return String(config.trk_en ? "1" : "0");
  } else if (cmd == "AT+TRK_EN=1") {
    config.trk_en = true;
    return "OK";
  } else if (cmd == "AT+TRK_EN=0") {
    config.trk_en = false;
    return "OK";
  }

  else if (cmd == "AT+TRK_LOC2RF?") {
    return String(config.trk_loc2rf ? "1" : "0");
  } else if (cmd == "AT+TRK_LOC2RF=1") {
    config.trk_loc2rf = true;
    return "OK";
  } else if (cmd == "AT+TRK_LOC2RF=0") {
    config.trk_loc2rf = false;
    return "OK";
  }

  else if (cmd == "AT+TRK_LOC2INET?") {
    return String(config.trk_loc2inet ? "1" : "0");
  } else if (cmd == "AT+TRK_LOC2INET=1") {
    config.trk_loc2inet = true;
    return "OK";
  } else if (cmd == "AT+TRK_LOC2INET=0") {
    config.trk_loc2inet = false;
    return "OK";
  }

  else if (cmd == "AT+TRK_TIMESTAMP?") {
    return String(config.trk_timestamp ? "1" : "0");
  } else if (cmd == "AT+TRK_TIMESTAMP=1") {
    config.trk_timestamp = true;
    return "OK";
  } else if (cmd == "AT+TRK_TIMESTAMP=0") {
    config.trk_timestamp = false;
    return "OK";
  }

  else if (cmd == "AT+TRK_SSID?") {
    return String(config.trk_ssid);
  } else if (cmd.startsWith("AT+TRK_SSID=")) {
    config.trk_ssid = cmd.substring(12).toInt();
    return "OK";
  }

  else if (cmd == "AT+TRK_MYCALL?") {
    return String(config.trk_mycall);
  } else if (cmd.startsWith("AT+TRK_MYCALL=")) {
    cmd.remove(0, 14);
    cmd.replace("\"", "");
    strncpy(config.trk_mycall, cmd.c_str(), sizeof(config.trk_mycall));
    return "OK";
  }

  else if (cmd == "AT+TRK_PATH?") {
    return String(config.trk_path);
  } else if (cmd.startsWith("AT+TRK_PATH=")) {
    config.trk_path = cmd.substring(12).toInt();
    return "OK";
  }

  else if (cmd == "AT+TRK_GPS?") {
    return String(config.trk_gps ? "1" : "0");
  } else if (cmd == "AT+TRK_GPS=1") {
    config.trk_gps = true;
    return "OK";
  } else if (cmd == "AT+TRK_GPS=0") {
    config.trk_gps = false;
    return "OK";
  }

  else if (cmd == "AT+TRK_LAT?") {
    return String(config.trk_lat, 6);
  } else if (cmd.startsWith("AT+TRK_LAT=")) {
    config.trk_lat = cmd.substring(11).toFloat();
    return "OK";
  }

  else if (cmd == "AT+TRK_LON?") {
    return String(config.trk_lon, 6);
  } else if (cmd.startsWith("AT+TRK_LON=")) {
    config.trk_lon = cmd.substring(11).toFloat();
    return "OK";
  }

  else if (cmd == "AT+TRK_ALT?") {
    return String(config.trk_alt, 6);
  } else if (cmd.startsWith("AT+TRK_ALT=")) {
    config.trk_alt = cmd.substring(11).toFloat();
    return "OK";
  }

  else if (cmd == "AT+TRK_COMMENT?") {
    return String(config.trk_comment);
  } else if (cmd.startsWith("AT+TRK_COMMENT=")) {
    cmd.remove(0, 15);
    cmd.replace("\"", "");
    strncpy(config.trk_comment, cmd.c_str(), sizeof(config.trk_comment));
    return "OK";
  }

  else if (cmd == "AT+TRK_STS_INTERVAL?") {
    return String(config.trk_sts_interval);
  } else if (cmd.startsWith("AT+TRK_STS_INTERVAL=")) {
    config.trk_sts_interval = cmd.substring(20).toInt();
    return "OK";
  }

  else if (cmd == "AT+TRK_STATUS?") {
    return String(config.trk_status);
  } else if (cmd.startsWith("AT+TRK_STATUS=")) {
    cmd.remove(0, 14);
    cmd.replace("\"", "");
    strncpy(config.trk_status, cmd.c_str(), sizeof(config.trk_status));
    return "OK";
  }

  else if (cmd == "AT+WX_EN?") {
    return String(config.wx_en ? "1" : "0");
  } else if (cmd == "AT+WX_EN=1") {
    config.wx_en = true;
    return "OK";
  } else if (cmd == "AT+WX_EN=0") {
    config.wx_en = false;
    return "OK";
  }

  else if (cmd == "AT+WX_2RF?") {
    return String(config.wx_2rf ? "1" : "0");
  } else if (cmd == "AT+WX_2RF=1") {
    config.wx_2rf = true;
    return "OK";
  } else if (cmd == "AT+WX_2RF=0") {
    config.wx_2rf = false;
    return "OK";
  }

  else if (cmd == "AT+WX_2INET?") {
    return String(config.wx_2inet ? "1" : "0");
  } else if (cmd == "AT+WX_2INET=1") {
    config.wx_2inet = true;
    return "OK";
  } else if (cmd == "AT+WX_2INET=0") {
    config.wx_2inet = false;
    return "OK";
  }

  else if (cmd == "AT+WX_TIMESTAMP?") {
    return String(config.wx_timestamp ? "1" : "0");
  } else if (cmd == "AT+WX_TIMESTAMP=1") {
    config.wx_timestamp = true;
    return "OK";
  } else if (cmd == "AT+WX_TIMESTAMP=0") {
    config.wx_timestamp = false;
    return "OK";
  }

  else if (cmd == "AT+WX_SSID?") {
    return String(config.wx_ssid);
  } else if (cmd.startsWith("AT+WX_SSID=")) {
    config.wx_ssid = cmd.substring(11).toInt();
    return "OK";
  }

  else if (cmd == "AT+WX_MYCALL?") {
    return String(config.wx_mycall);
  } else if (cmd.startsWith("AT+WX_MYCALL=")) {
    cmd.remove(0, 13);
    cmd.replace("\"", "");
    strncpy(config.wx_mycall, cmd.c_str(), sizeof(config.wx_mycall));
    return "OK";
  }

  else if (cmd == "AT+WX_PATH?") {
    return String(config.wx_path);
  } else if (cmd.startsWith("AT+WX_PATH=")) {
    config.wx_path = cmd.substring(11).toInt();
    return "OK";
  }

  else if (cmd == "AT+WX_GPS?") {
    return String(config.wx_gps ? "1" : "0");
  } else if (cmd == "AT+WX_GPS=1") {
    config.wx_gps = true;
    return "OK";
  } else if (cmd == "AT+WX_GPS=0") {
    config.wx_gps = false;
    return "OK";
  }

  else if (cmd == "AT+WX_LAT?") {
    return String(config.wx_lat, 6);
  } else if (cmd.startsWith("AT+WX_LAT=")) {
    config.wx_lat = cmd.substring(10).toFloat();
    return "OK";
  }

  else if (cmd == "AT+WX_LON?") {
    return String(config.wx_lon, 6);
  } else if (cmd.startsWith("AT+WX_LON=")) {
    config.wx_lon = cmd.substring(10).toFloat();
    return "OK";
  }

  else if (cmd == "AT+WX_ALT?") {
    return String(config.wx_alt, 6);
  } else if (cmd.startsWith("AT+WX_ALT=")) {
    config.wx_alt = cmd.substring(10).toFloat();
    return "OK";
  }

  else if (cmd == "AT+WX_INTERVAL?") {
    return String(config.wx_interval);
  } else if (cmd.startsWith("AT+WX_INTERVAL=")) {
    config.wx_interval = cmd.substring(15).toInt();
    return "OK";
  }

  else if (cmd == "AT+WX_FLAGE?") {
    return String(config.wx_flage);
  } else if (cmd.startsWith("AT+WX_FLAGE=")) {
    config.wx_flage = cmd.substring(12).toInt();
    return "OK";
  }

  else if (cmd == "AT+WX_OBJECT?") {
    return String(config.wx_object);
  } else if (cmd.startsWith("AT+WX_OBJECT=")) {
    cmd.remove(0, 13);
    cmd.replace("\"", "");
    strncpy(config.wx_object, cmd.c_str(), sizeof(config.wx_object));
    return "OK";
  }

  else if (cmd == "AT+WX_COMMENT?") {
    return String(config.wx_comment);
  } else if (cmd.startsWith("AT+WX_COMMENT=")) {
    cmd.remove(0, 14);
    cmd.replace("\"", "");
    strncpy(config.wx_comment, cmd.c_str(), sizeof(config.wx_comment));
    return "OK";
  }

  else if (cmd == "AT+TLM0_EN?") {
    return String(config.tlm0_en ? "1" : "0");
  } else if (cmd == "AT+TLM0_EN=1") {
    config.tlm0_en = true;
    return "OK";
  } else if (cmd == "AT+TLM0_EN=0") {
    config.tlm0_en = false;
    return "OK";
  }

//   else if (cmd == "AT+TLM0_2RF?") {
//     return String(config.tlm0_2rf ? "1" : "0");
//   } else if (cmd == "AT+TLM0_2RF=1") {
//     config.tlm0_2rf = true;
//     return "OK";
//   } else if (cmd == "AT+TLM0_2RF=0") {
//     config.tlm0_2rf = false;
//     return "OK";
//   }

//   else if (cmd == "AT+TLM0_2INET?") {
//     return String(config.tlm0_2inet ? "1" : "0");
//   } else if (cmd == "AT+TLM0_2INET=1") {
//     config.tlm0_2inet = true;
//     return "OK";
//   } else if (cmd == "AT+TLM0_2INET=0") {
//     config.tlm0_2inet = false;
//     return "OK";
//   }

//   else if (cmd == "AT+TLM0_SSID?") {
//     return String(config.tlm0_ssid);
//   } else if (cmd.startsWith("AT+TLM0_SSID=")) {
//     config.tlm0_ssid = cmd.substring(13).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+TLM0_MYCALL?") {
//     return String(config.tlm0_mycall);
//   } else if (cmd.startsWith("AT+TLM0_MYCALL=")) {
//     cmd.remove(0, 15);
//     cmd.replace("\"", "");
//     strncpy(config.tlm0_mycall, cmd.c_str(), sizeof(config.tlm0_mycall));
//     return "OK";
//   }

//   else if (cmd == "AT+TLM0_PATH?") {
//     return String(config.tlm0_path);
//   } else if (cmd.startsWith("AT+TLM0_PATH=")) {
//     config.tlm0_path = cmd.substring(13).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+TLM0_DATA_INTERVAL?") {
//     return String(config.tlm0_data_interval);
//   } else if (cmd.startsWith("AT+TLM0_DATA_INTERVAL=")) {
//     config.tlm0_data_interval = cmd.substring(22).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+TLM0_INFO_INTERVAL?") {
//     return String(config.tlm0_info_interval);
//   } else if (cmd.startsWith("AT+TLM0_INFO_INTERVAL=")) {
//     config.tlm0_info_interval = cmd.substring(22).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+TLM0_BITS_ACTIVE?") {
//     return String(config.tlm0_BITS_Active);
//   } else if (cmd.startsWith("AT+TLM0_BITS_ACTIVE=")) {
//     config.tlm0_BITS_Active = cmd.substring(20).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+TLM0_COMMENT?") {
//     return String(config.tlm0_comment);
//   } else if (cmd.startsWith("AT+TLM0_COMMENT=")) {
//     cmd.remove(0, 16);
//     cmd.replace("\"", "");
//     strncpy(config.tlm0_comment, cmd.c_str(), sizeof(config.tlm0_comment));
//     return "OK";
//   }

//   else if (cmd == "AT+TML0_DATA_CHANNEL?") {
//     return String(config.tml0_data_channel);
//   } else if (cmd.startsWith("AT+TML0_DATA_CHANNEL=")) {
//     config.tml0_data_channel = cmd.substring(21).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+TLM1_EN?") {
//     return String(config.tlm1_en ? "1" : "0");
//   } else if (cmd == "AT+TLM1_EN=1") {
//     config.tlm1_en = true;
//     return "OK";
//   } else if (cmd == "AT+TLM1_EN=0") {
//     config.tlm1_en = false;
//     return "OK";
//   }

//   else if (cmd == "AT+TLM1_2RF?") {
//     return String(config.tlm1_2rf ? "1" : "0");
//   } else if (cmd == "AT+TLM1_2RF=1") {
//     config.tlm1_2rf = true;
//     return "OK";
//   } else if (cmd == "AT+TLM1_2RF=0") {
//     config.tlm1_2rf = false;
//     return "OK";
//   }

//   else if (cmd == "AT+TLM1_2INET?") {
//     return String(config.tlm1_2inet ? "1" : "0");
//   } else if (cmd == "AT+TLM1_2INET=1") {
//     config.tlm1_2inet = true;
//     return "OK";
//   } else if (cmd == "AT+TLM1_2INET=0") {
//     config.tlm1_2inet = false;
//     return "OK";
//   }

//   else if (cmd == "AT+TLM1_SSID?") {
//     return String(config.tlm1_ssid);
//   } else if (cmd.startsWith("AT+TLM1_SSID=")) {
//     config.tlm1_ssid = cmd.substring(13).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+TLM1_MYCALL?") {
//     return String(config.tlm1_mycall);
//   } else if (cmd.startsWith("AT+TLM1_MYCALL=")) {
//     cmd.remove(0, 15);
//     cmd.replace("\"", "");
//     strncpy(config.tlm1_mycall, cmd.c_str(), sizeof(config.tlm1_mycall));
//     return "OK";
//   }

//   else if (cmd == "AT+TLM1_PATH?") {
//     return String(config.tlm1_path);
//   } else if (cmd.startsWith("AT+TLM1_PATH=")) {
//     config.tlm1_path = cmd.substring(13).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+TLM1_DATA_INTERVAL?") {
//     return String(config.tlm1_data_interval);
//   } else if (cmd.startsWith("AT+TLM1_DATA_INTERVAL=")) {
//     config.tlm1_data_interval = cmd.substring(22).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+TLM1_INFO_INTERVAL?") {
//     return String(config.tlm1_info_interval);
//   } else if (cmd.startsWith("AT+TLM1_INFO_INTERVAL=")) {
//     config.tlm1_info_interval = cmd.substring(22).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+TLM1_BITS_ACTIVE?") {
//     return String(config.tlm1_BITS_Active);
//   } else if (cmd.startsWith("AT+TLM1_BITS_ACTIVE=")) {
//     config.tlm1_BITS_Active = cmd.substring(20).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+TLM1_COMMENT?") {
//     return String(config.tlm1_comment);
//   } else if (cmd.startsWith("AT+TLM1_COMMENT=")) {
//     cmd.remove(0, 16);
//     cmd.replace("\"", "");
//     strncpy(config.tlm1_comment, cmd.c_str(), sizeof(config.tlm1_comment));
//     return "OK";
//   }

//   else if (cmd == "AT+TML1_DATA_CHANNEL?") {
//     return String(config.tml1_data_channel);
//   } else if (cmd.startsWith("AT+TML1_DATA_CHANNEL=")) {
//     config.tml1_data_channel = cmd.substring(21).toInt();
//     return "OK";
//   }

  else if (cmd == "AT+OLED_ENABLE?") {
    return String(config.oled_enable ? "1" : "0");
  } else if (cmd == "AT+OLED_ENABLE=1") {
    config.oled_enable = true;
    return "OK";
  } else if (cmd == "AT+OLED_ENABLE=0") {
    config.oled_enable = false;
    return "OK";
  }

  else if (cmd == "AT+OLED_TIMEOUT?") {
    return String(config.oled_timeout);
  } else if (cmd.startsWith("AT+OLED_TIMEOUT=")) {
    config.oled_timeout = cmd.substring(16).toInt();
    return "OK";
  }

  else if (cmd == "AT+DIM?") {
    return String(config.dim);
  } else if (cmd.startsWith("AT+DIM=")) {
    config.dim = cmd.substring(7).toInt();
    return "OK";
  }

  else if (cmd == "AT+CONTRAST?") {
    return String(config.contrast);
  } else if (cmd.startsWith("AT+CONTRAST=")) {
    config.contrast = cmd.substring(12).toInt();
    return "OK";
  }

  else if (cmd == "AT+STARTUP?") {
    return String(config.startup);
  } else if (cmd.startsWith("AT+STARTUP=")) {
    config.startup = cmd.substring(11).toInt();
    return "OK";
  }

  else if (cmd == "AT+DISPFILTER?") {
    return String(config.dispFilter);
  } else if (cmd.startsWith("AT+DISPFILTER=")) {
    config.dispFilter = cmd.substring(14).toInt();
    return "OK";
  }

  else if (cmd == "AT+DISPRF?") {
    return String(config.dispRF ? "1" : "0");
  } else if (cmd == "AT+DISPRF=1") {
    config.dispRF = true;
    return "OK";
  } else if (cmd == "AT+DISPRF=0") {
    config.dispRF = false;
    return "OK";
  }

  else if (cmd == "AT+DISPINET?") {
    return String(config.dispINET ? "1" : "0");
  } else if (cmd == "AT+DISPINET=1") {
    config.dispINET = true;
    return "OK";
  } else if (cmd == "AT+DISPINET=0") {
    config.dispINET = false;
    return "OK";
  }

  else if (cmd == "AT+TX_TIMESLOT?") {
    return String(config.tx_timeslot);
  } else if (cmd.startsWith("AT+TX_TIMESLOT=")) {
    config.tx_timeslot = cmd.substring(15).toInt();
    return "OK";
  }

  else if (cmd == "AT+NTP_HOST?") {
    return String(config.ntp_host);
  } else if (cmd.startsWith("AT+NTP_HOST=")) {
    cmd.remove(0, 12);
    cmd.replace("\"", "");
    strncpy(config.ntp_host, cmd.c_str(), sizeof(config.ntp_host));
    return "OK";
  }

  else if (cmd == "AT+VPN?") {
    return String(config.vpn ? "1" : "0");
  } else if (cmd == "AT+VPN=1") {
    config.vpn = true;
    return "OK";
  } else if (cmd == "AT+VPN=0") {
    config.vpn = false;
    return "OK";
  }

  else if (cmd == "AT+MODEM?") {
    return String(config.modem ? "1" : "0");
  } else if (cmd == "AT+MODEM=1") {
    config.modem = true;
    return "OK";
  } else if (cmd == "AT+MODEM=0") {
    config.modem = false;
    return "OK";
  }

  else if (cmd == "AT+WG_PORT?") {
    return String(config.wg_port);
  } else if (cmd.startsWith("AT+WG_PORT=")) {
    config.wg_port = cmd.substring(11).toInt();
    return "OK";
  }

  else if (cmd == "AT+WG_PEER_ADDRESS?") {
    return String(config.wg_peer_address);
  } else if (cmd.startsWith("AT+WG_PEER_ADDRESS=")) {
    cmd.remove(0, 19);
    cmd.replace("\"", "");
    strncpy(config.wg_peer_address, cmd.c_str(), sizeof(config.wg_peer_address));
    return "OK";
  }

  else if (cmd == "AT+WG_LOCAL_ADDRESS?") {
    return String(config.wg_local_address);
  } else if (cmd.startsWith("AT+WG_LOCAL_ADDRESS=")) {
    cmd.remove(0, 20);
    cmd.replace("\"", "");
    strncpy(config.wg_local_address, cmd.c_str(), sizeof(config.wg_local_address));
    return "OK";
  }

  else if (cmd == "AT+WG_NETMASK_ADDRESS?") {
    return String(config.wg_netmask_address);
  } else if (cmd.startsWith("AT+WG_NETMASK_ADDRESS=")) {
    cmd.remove(0, 22);
    cmd.replace("\"", "");
    strncpy(config.wg_netmask_address, cmd.c_str(), sizeof(config.wg_netmask_address));
    return "OK";
  }

  else if (cmd == "AT+WG_GW_ADDRESS?") {
    return String(config.wg_gw_address);
  } else if (cmd.startsWith("AT+WG_GW_ADDRESS=")) {
    cmd.remove(0, 17);
    cmd.replace("\"", "");
    strncpy(config.wg_gw_address, cmd.c_str(), sizeof(config.wg_gw_address));
    return "OK";
  }

  else if (cmd == "AT+WG_PUBLIC_KEY?") {
    return String(config.wg_public_key);
  } else if (cmd.startsWith("AT+WG_PUBLIC_KEY=")) {
    cmd.remove(0, 17);
    cmd.replace("\"", "");
    strncpy(config.wg_public_key, cmd.c_str(), sizeof(config.wg_public_key));
    return "OK";
  }

  else if (cmd == "AT+WG_PRIVATE_KEY?") {
    return String(config.wg_private_key);
  } else if (cmd.startsWith("AT+WG_PRIVATE_KEY=")) {
    cmd.remove(0, 18);
    cmd.replace("\"", "");
    strncpy(config.wg_private_key, cmd.c_str(), sizeof(config.wg_private_key));
    return "OK";
  }

  else if (cmd == "AT+HTTP_USERNAME?") {
    return String(config.http_username);
  } else if (cmd.startsWith("AT+HTTP_USERNAME=")) {
    cmd.remove(0, 17);
    cmd.replace("\"", "");
    strncpy(config.http_username, cmd.c_str(), sizeof(config.http_username));
    return "OK";
  }

  else if (cmd == "AT+HTTP_PASSWORD?") {
    return String(config.http_password);
  } else if (cmd.startsWith("AT+HTTP_PASSWORD=")) {
    cmd.remove(0, 17);
    cmd.replace("\"", "");
    strncpy(config.http_password, cmd.c_str(), sizeof(config.http_password));
    return "OK";
  }

  else if (cmd == "AT+GNSS_ENABLE?") {
    return String(config.gnss_enable ? "1" : "0");
  } else if (cmd == "AT+GNSS_ENABLE=1") {
    config.gnss_enable = true;
    return "OK";
  } else if (cmd == "AT+GNSS_ENABLE=0") {
    config.gnss_enable = false;
    return "OK";
  }

  else if (cmd == "AT+GNSS_TCP_PORT?") {
    return String(config.gnss_tcp_port);
  } else if (cmd.startsWith("AT+GNSS_TCP_PORT=")) {
    config.gnss_tcp_port = cmd.substring(17).toInt();
    return "OK";
  }

  else if (cmd == "AT+GNSS_TCP_HOST?") {
    return String(config.gnss_tcp_host);
  } else if (cmd.startsWith("AT+GNSS_TCP_HOST=")) {
    cmd.remove(0, 17);
    cmd.replace("\"", "");
    strncpy(config.gnss_tcp_host, cmd.c_str(), sizeof(config.gnss_tcp_host));
    return "OK";
  }

  else if (cmd == "AT+GNSS_AT_COMMAND?") {
    return String(config.gnss_at_command);
  } else if (cmd.startsWith("AT+GNSS_AT_COMMAND=")) {
    cmd.remove(0, 19);
    cmd.replace("\"", "");
    strncpy(config.gnss_at_command, cmd.c_str(), sizeof(config.gnss_at_command));
    return "OK";
  }

  else if (cmd == "AT+I2C_ENABLE?") {
    return String(config.i2c_enable ? "1" : "0");
  } else if (cmd == "AT+I2C_ENABLE=1") {
    config.i2c_enable = true;
    return "OK";
  } else if (cmd == "AT+I2C_ENABLE=0") {
    config.i2c_enable = false;
    return "OK";
  }

  else if (cmd == "AT+I2C1_ENABLE?") {
    return String(config.i2c1_enable ? "1" : "0");
  } else if (cmd == "AT+I2C1_ENABLE=1") {
    config.i2c1_enable = true;
    return "OK";
  } else if (cmd == "AT+I2C1_ENABLE=0") {
    config.i2c1_enable = false;
    return "OK";
  }

  else if (cmd == "AT+PWR_EN?") {
    return String(config.pwr_en ? "1" : "0");
  } else if (cmd == "AT+PWR_EN=1") {
    config.pwr_en = true;
    return "OK";
  } else if (cmd == "AT+PWR_EN=0") {
    config.pwr_en = false;
    return "OK";
  }

  else if (cmd == "AT+PWR_MODE?") {
    return String(config.pwr_mode);
  } else if (cmd.startsWith("AT+PWR_MODE=")) {
    config.pwr_mode = cmd.substring(12).toInt();
    return "OK";
  }

  else if (cmd == "AT+PWR_SLEEP_INTERVAL?") {
    return String(config.pwr_sleep_interval);
  } else if (cmd.startsWith("AT+PWR_SLEEP_INTERVAL=")) {
    config.pwr_sleep_interval = cmd.substring(22).toInt();
    return "OK";
  }

  else if (cmd == "AT+PWR_STANBY_DELAY?") {
    return String(config.pwr_stanby_delay);
  } else if (cmd.startsWith("AT+PWR_STANBY_DELAY=")) {
    config.pwr_stanby_delay = cmd.substring(20).toInt();
    return "OK";
  }

  else if (cmd == "AT+PWR_SLEEP_ACTIVATE?") {
    return String(config.pwr_sleep_activate);
  } else if (cmd.startsWith("AT+PWR_SLEEP_ACTIVATE=")) {
    config.pwr_sleep_activate = cmd.substring(22).toInt();
    return "OK";
  }

  else if (cmd == "AT+DISP_FLIP?") {
    return String(config.disp_flip ? "1" : "0");
  } else if (cmd == "AT+DISP_FLIP=1") {
    config.disp_flip = true;
    return "OK";
  } else if (cmd == "AT+DISP_FLIP=0") {
    config.disp_flip = false;
    return "OK";
  }

  else if (cmd == "AT+DISP_BRIGHTNESS?") {
    return String(config.disp_brightness);
  } else if (cmd.startsWith("AT+DISP_BRIGHTNESS=")) {
    config.disp_brightness = cmd.substring(19).toInt();
    return "OK";
  }

//   else if (cmd == "AT+TRK_TLM_AVG?") {
//     return String(config.trk_tlm_avg ? "1" : "0");
//   } else if (cmd == "AT+TRK_TLM_AVG=1") {
//     config.trk_tlm_avg = true;
//     return "OK";
//   } else if (cmd == "AT+TRK_TLM_AVG=0") {
//     config.trk_tlm_avg = false;
//     return "OK";
//   }

//   else if (cmd == "AT+TRK_TLM_SENSOR?") {
//     return String(config.trk_tlm_sensor);
//   } else if (cmd.startsWith("AT+TRK_TLM_SENSOR=")) {
//     config.trk_tlm_sensor = cmd.substring(18).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+TRK_TLM_PRECISION?") {
//     return String(config.trk_tlm_precision);
//   } else if (cmd.startsWith("AT+TRK_TLM_PRECISION=")) {
//     config.trk_tlm_precision = cmd.substring(21).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+TRK_TLM_OFFSET?") {
//     return String(config.trk_tlm_offset, 6);
//   } else if (cmd.startsWith("AT+TRK_TLM_OFFSET=")) {
//     config.trk_tlm_offset = cmd.substring(18).toFloat();
//     return "OK";
//   }

//   else if (cmd == "AT+DIGI_TLM_AVG?") {
//     return String(config.digi_tlm_avg ? "1" : "0");
//   } else if (cmd == "AT+DIGI_TLM_AVG=1") {
//     config.digi_tlm_avg = true;
//     return "OK";
//   } else if (cmd == "AT+DIGI_TLM_AVG=0") {
//     config.digi_tlm_avg = false;
//     return "OK";
//   }

//   else if (cmd == "AT+DIGI_TLM_SENSOR?") {
//     return String(config.digi_tlm_sensor);
//   } else if (cmd.startsWith("AT+DIGI_TLM_SENSOR=")) {
//     config.digi_tlm_sensor = cmd.substring(19).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+DIGI_TLM_PRECISION?") {
//     return String(config.digi_tlm_precision);
//   } else if (cmd.startsWith("AT+DIGI_TLM_PRECISION=")) {
//     config.digi_tlm_precision = cmd.substring(22).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+DIGI_TLM_OFFSET?") {
//     return String(config.digi_tlm_offset, 6);
//   } else if (cmd.startsWith("AT+DIGI_TLM_OFFSET=")) {
//     config.digi_tlm_offset = cmd.substring(19).toFloat();
//     return "OK";
//   }

//   else if (cmd == "AT+IGATE_TLM_AVG?") {
//     return String(config.igate_tlm_avg ? "1" : "0");
//   } else if (cmd == "AT+IGATE_TLM_AVG=1") {
//     config.igate_tlm_avg = true;
//     return "OK";
//   } else if (cmd == "AT+IGATE_TLM_AVG=0") {
//     config.igate_tlm_avg = false;
//     return "OK";
//   }

//   else if (cmd == "AT+IGATE_TLM_SENSOR?") {
//     return String(config.igate_tlm_sensor);
//   } else if (cmd.startsWith("AT+IGATE_TLM_SENSOR=")) {
//     config.igate_tlm_sensor = cmd.substring(20).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+IGATE_TLM_PRECISION?") {
//     return String(config.igate_tlm_precision);
//   } else if (cmd.startsWith("AT+IGATE_TLM_PRECISION=")) {
//     config.igate_tlm_precision = cmd.substring(23).toInt();
//     return "OK";
//   }

//   else if (cmd == "AT+IGATE_TLM_OFFSET?") {
//     return String(config.igate_tlm_offset, 6);
//   } else if (cmd.startsWith("AT+IGATE_TLM_OFFSET=")) {
//     config.igate_tlm_offset = cmd.substring(20).toFloat();
//     return "OK";
//   }

//   else if (cmd == "AT+WX_SENSOR_ENABLE?") {
//     return String(config.wx_sensor_enable ? "1" : "0");
//   } else if (cmd == "AT+WX_SENSOR_ENABLE=1") {
//     config.wx_sensor_enable = true;
//     return "OK";
//   } else if (cmd == "AT+WX_SENSOR_ENABLE=0") {
//     config.wx_sensor_enable = false;
//     return "OK";
//   }

//   else if (cmd == "AT+WX_SENSOR_AVG?") {
//     return String(config.wx_sensor_avg ? "1" : "0");
//   } else if (cmd == "AT+WX_SENSOR_AVG=1") {
//     config.wx_sensor_avg = true;
//     return "OK";
//   } else if (cmd == "AT+WX_SENSOR_AVG=0") {
//     config.wx_sensor_avg = false;
//     return "OK";
//   }

//   else if (cmd == "AT+WX_SENSOR_CH?") {
//     return String(config.wx_sensor_ch);
//   } else if (cmd.startsWith("AT+WX_SENSOR_CH=")) {
//     config.wx_sensor_ch = cmd.substring(16).toInt();
//     return "OK";
//   }

  else if (cmd == "AT+PPP_APN?") {
    return String(config.ppp_apn);
  } else if (cmd.startsWith("AT+PPP_APN=")) {
    cmd.remove(0, 11);
    cmd.replace("\"", "");
    strncpy(config.ppp_apn, cmd.c_str(), sizeof(config.ppp_apn));
    return "OK";
  }

  else if (cmd == "AT+PPP_PIN?") {
    return String(config.ppp_pin);
  } else if (cmd.startsWith("AT+PPP_PIN=")) {
    cmd.remove(0, 11);
    cmd.replace("\"", "");
    strncpy(config.ppp_pin, cmd.c_str(), sizeof(config.ppp_pin));
    return "OK";
  }

  else if (cmd == "AT+EN_MQTT?") {
    return String(config.en_mqtt ? "1" : "0");
  } else if (cmd == "AT+EN_MQTT=1") {
    config.en_mqtt = true;
    return "OK";
  } else if (cmd == "AT+EN_MQTT=0") {
    config.en_mqtt = false;
    return "OK";
  }

  else if (cmd == "AT+MQTT_HOST?") {
    return String(config.mqtt_host);
  } else if (cmd.startsWith("AT+MQTT_HOST=")) {
    cmd.remove(0, 13);
    cmd.replace("\"", "");
    strncpy(config.mqtt_host, cmd.c_str(), sizeof(config.mqtt_host));
    return "OK";
  }

  else if (cmd == "AT+MQTT_TOPIC?") {
    return String(config.mqtt_topic);
  } else if (cmd.startsWith("AT+MQTT_TOPIC=")) {
    cmd.remove(0, 14);
    cmd.replace("\"", "");
    strncpy(config.mqtt_topic, cmd.c_str(), sizeof(config.mqtt_topic));
    return "OK";
  }

  else if (cmd == "AT+MQTT_SUBSCRIBE?") {
    return String(config.mqtt_subscribe);
  } else if (cmd.startsWith("AT+MQTT_SUBSCRIBE=")) {
    cmd.remove(0, 18);
    cmd.replace("\"", "");
    strncpy(config.mqtt_subscribe, cmd.c_str(), sizeof(config.mqtt_subscribe));
    return "OK";
  }

  else if (cmd == "AT+MQTT_USER?") {
    return String(config.mqtt_user);
  } else if (cmd.startsWith("AT+MQTT_USER=")) {
    cmd.remove(0, 13);
    cmd.replace("\"", "");
    strncpy(config.mqtt_user, cmd.c_str(), sizeof(config.mqtt_user));
    return "OK";
  }

  else if (cmd == "AT+MQTT_PASS?") {
    return String(config.mqtt_pass);
  } else if (cmd.startsWith("AT+MQTT_PASS=")) {
    cmd.remove(0, 13);
    cmd.replace("\"", "");
    strncpy(config.mqtt_pass, cmd.c_str(), sizeof(config.mqtt_pass));
    return "OK";
  }

  else if (cmd == "AT+MQTT_PORT?") {
    return String(config.mqtt_port);
  } else if (cmd.startsWith("AT+MQTT_PORT=")) {
    config.mqtt_port = cmd.substring(13).toInt();
    return "OK";
  }

  else if (cmd == "AT+MQTT_TOPIC_FLAG?") {
    return String(config.mqtt_topic_flag);
  } else if (cmd.startsWith("AT+MQTT_TOPIC_FLAG=")) {
    config.mqtt_topic_flag = cmd.substring(19).toInt();
    return "OK";
  }

  else if (cmd == "AT+MQTT_SUBSCRIBE_FLAG?") {
    return String(config.mqtt_subscribe_flag);
  } else if (cmd.startsWith("AT+MQTT_SUBSCRIBE_FLAG=")) {
    config.mqtt_subscribe_flag = cmd.substring(23).toInt();
    return "OK";
  }

  else if (cmd == "AT+TRK_MICE_TYPE?") {
    return String(config.trk_mice_type);
  } else if (cmd.startsWith("AT+TRK_MICE_TYPE=")) {
    config.trk_mice_type = cmd.substring(17).toInt();
    return "OK";
  }

  else if (cmd == "AT+TRK_TLM_INTERVAL?") {
    return String(config.trk_tlm_interval);
  } else if (cmd.startsWith("AT+TRK_TLM_INTERVAL=")) {
    config.trk_tlm_interval = cmd.substring(20).toInt();
    return "OK";
  }

  else if (cmd == "AT+DIGI_TLM_INTERVAL?") {
    return String(config.digi_tlm_interval);
  } else if (cmd.startsWith("AT+DIGI_TLM_INTERVAL=")) {
    config.digi_tlm_interval = cmd.substring(21).toInt();
    return "OK";
  }

  else if (cmd == "AT+IGATE_TLM_INTERVAL?") {
    return String(config.igate_tlm_interval);
  } else if (cmd.startsWith("AT+IGATE_TLM_INTERVAL=")) {
    config.igate_tlm_interval = cmd.substring(22).toInt();
    return "OK";
  }

  else if (cmd == "AT+WX_TLM_INTERVAL?") {
    return String(config.wx_tlm_interval);
  } else if (cmd.startsWith("AT+WX_TLM_INTERVAL=")) {
    config.wx_tlm_interval = cmd.substring(19).toInt();
    return "OK";
  }

  else if (cmd == "AT+HOST_NAME?") {
    return String(config.host_name);
  } else if (cmd.startsWith("AT+HOST_NAME=")) {
    cmd.remove(0, 13);
    cmd.replace("\"", "");
    strncpy(config.host_name, cmd.c_str(), sizeof(config.host_name));
    return "OK";
  }
  else {
    return "ERR";
  }
}

