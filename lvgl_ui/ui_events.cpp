/*
 * ui_events.cpp - SquareLine event handlers wired to the ESP32APRS_LoRa
 * runtime configuration (was NAWS_Display's flame/smoke alarm box config).
 */
#ifdef USE_LVGL_UI

#include <Arduino.h>
#include <string.h>
#include <stdio.h>

#include "ui.h"
#include "main.h"

extern Configuration config;

static void copy_text(char *dst, size_t size, lv_obj_t *ta)
{
    const char *txt = ta ? lv_textarea_get_text(ta) : "";
    if (txt == NULL)
        txt = "";
    snprintf(dst, size, "%s", txt);
}

void saveConfig(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED)
        return;

    copy_text(config.wifi_sta[0].wifi_ssid, sizeof(config.wifi_sta[0].wifi_ssid), ui_txtSSID);
    copy_text(config.wifi_sta[0].wifi_pass, sizeof(config.wifi_sta[0].wifi_pass), ui_txtPasword);
    copy_text(config.igate_mycall, sizeof(config.igate_mycall), ui_txtMyNode);
    copy_text(config.igate_filter, sizeof(config.igate_filter), ui_txtFilterNode);
#ifdef MQTT
    copy_text(config.mqtt_host, sizeof(config.mqtt_host), ui_txtMQTTHost);

    const char *port = ui_txtMQTTPort ? lv_textarea_get_text(ui_txtMQTTPort) : "0";
    config.mqtt_port = (uint16_t)atoi(port ? port : "0");
#endif

    bool ok = saveConfiguration("/default.cfg", config);
    lv_obj_t *box = lv_msgbox_create(NULL, "SAVE",
                                     ok ? "Configuration saved to /default.cfg"
                                        : "Failed to write /default.cfg",
                                     NULL, true);
    lv_obj_center(box);
}

void wifiEnable(lv_event_t *e)
{
    bool on = lv_obj_has_state(ui_swWiFi, LV_STATE_CHECKED);
    config.wifi_sta[0].enable = on;
    if (on)
        config.wifi_mode |= WIFI_STA_FIX;
    else
        config.wifi_mode &= ~WIFI_STA_FIX;
}

void alarmEnable(lv_event_t *e)
{
    config.igate_en = lv_obj_has_state(ui_swAlarm, LV_STATE_CHECKED);
}

void mqttEnable(lv_event_t *e)
{
#ifdef MQTT
    config.en_mqtt = lv_obj_has_state(ui_swMQTT, LV_STATE_CHECKED);
#else
    (void)e;
#endif
}

void logerEnable(lv_event_t *e)
{
    config.rf_en = lv_obj_has_state(ui_swLoger, LV_STATE_CHECKED);
}

#endif /* USE_LVGL_UI */
