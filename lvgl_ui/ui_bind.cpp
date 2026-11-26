/*
 * ui_bind.cpp - data binding between ESP32APRS_LoRa and the SquareLine UI
 * (dashboard / config / warning screens).
 *
 * WX shown on the dashboard comes from
 *   1. the latest WX packet decoded by the APRS task (aprs.wx_report), or
 *   2. the local weather sensors (WeatherData weather + weather.visable)
 * whichever is fresher.
 */
#ifdef USE_LVGL_UI

#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

#include <TinyGPSPlus.h>

#include "lvgl.h"
#include "main.h"
#include "pbuf.h"
#include "ui.h"
#include "ui_bind.h"

extern struct pbuf_t aprs;
extern TinyGPSPlus gps;
extern Configuration config;
extern uint8_t wifiStatus;

#if defined(ESP32_3248S035)
#define UI_SCR_W 480
#define UI_SCR_H 320
#else
#define UI_SCR_W 320
#define UI_SCR_H 240
#endif
#define UI_TOPBAR_H 34
#define UI_CHART_POINTS 30
#define UI_WX_PACKET_HOLD_MS (10UL * 60UL * 1000UL) /* show a packet for 10 min */
#define UI_ALERT_WIFI_MS (120UL * 1000UL)

/* ------------------------------------------------------------------ */
/* snapshot written by the APRS / RF tasks, read by the LVGL task      */
/* ------------------------------------------------------------------ */
typedef struct
{
    volatile uint32_t seq;
    uint32_t flags;
    float temp;
    float humidity;
    float pressure;
    float rain1h;
    float rain24h;
    float rainMid;
    float windSpeed; /* m/s */
    float windGust;  /* m/s */
    int windDir;
    uint32_t luminosity;
    uint8_t uv;
    uint32_t stamp; /* millis() at reception */
    bool valid;
    char src[12];
} wx_snapshot_t;

static wx_snapshot_t wxPkt;

static volatile uint32_t rxStamp = 0;
static volatile uint32_t txStamp = 0;

/* ------------------------------------------------------------------ */
/* UI state                                                            */
/* ------------------------------------------------------------------ */
static lv_coord_t chartTemp[UI_CHART_POINTS];
static lv_coord_t chartHum[UI_CHART_POINTS];
static lv_chart_series_t *serTemp = NULL;
static lv_chart_series_t *serHum = NULL;
static uint16_t chartIdx = 0;
static uint16_t chartCount = 0;
static char tableBuf[1024];
static bool tableEmpty = true;

static bool alertActive = false;
static bool alertShown = false;
static uint32_t wifiDownSince = 0;

static uint32_t lastTickMs = 0;
static uint32_t lastChartMs = 0;
static uint32_t lastTableMs = 0;
static lv_obj_t *curScreen = NULL;

/* ------------------------------------------------------------------ */
/* helpers                                                             */
/* ------------------------------------------------------------------ */
static void lbl_set(lv_obj_t *obj, const char *fmt, ...)
{
    if (obj == NULL)
        return;
    char buf[40];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    lv_label_set_text(obj, buf);
}

static float dew_point_c(float temp, float rh)
{
    if (rh <= 0.0f)
        rh = 0.1f;
    const float a = 17.27f, b = 237.7f;
    float g = (a * temp) / (b + temp) + logf(rh / 100.0f);
    return (b * g) / (a - g);
}

static float heat_index_c(float temp, float rh)
{
    if (temp < 26.7f)
        return temp;
    float tf = temp * 9.0f / 5.0f + 32.0f;
    float hi = -42.379f + 2.04901523f * tf + 10.14333127f * rh - 0.22475541f * tf * rh -
               0.00683783f * tf * tf - 0.05481717f * rh * rh + 0.00122874f * tf * tf * rh +
               0.00085282f * tf * rh * rh - 0.00000199f * tf * tf * rh * rh;
    return (hi - 32.0f) * 5.0f / 9.0f;
}

static void chart_reset(void)
{
    for (uint16_t i = 0; i < UI_CHART_POINTS; i++)
    {
        chartTemp[i] = LV_CHART_POINT_NONE;
        chartHum[i] = LV_CHART_POINT_NONE;
    }
    chartIdx = 0;
    chartCount = 0;
}

static void chart_push(int16_t temp, int16_t hum)
{
    if (serTemp == NULL || serHum == NULL)
        return;
    if (chartCount < UI_CHART_POINTS)
        chartIdx = chartCount;
    else
        chartIdx = (chartIdx + 1) % UI_CHART_POINTS;
    chartTemp[chartIdx] = temp;
    chartHum[chartIdx] = hum;
    if (chartCount < UI_CHART_POINTS)
        chartCount++;
    lv_chart_refresh(ui_ChartWeather);
}

static void table_append(const char *line)
{
    if (ui_txtTable == NULL)
        return;
    size_t used = tableEmpty ? 0 : strlen(tableBuf);
    size_t len = strlen(line);
    if (used + len + 2 > sizeof(tableBuf))
    {
        /* drop the oldest half of the log */
        char *nl = strchr(tableBuf + sizeof(tableBuf) / 3, '\n');
        char tmp[sizeof(tableBuf)];
        if (nl != NULL)
        {
            snprintf(tmp, sizeof(tmp), "%s", nl + 1);
            snprintf(tableBuf, sizeof(tableBuf), "%s", tmp);
            used = strlen(tableBuf);
        }
        else
        {
            tableBuf[0] = 0;
            used = 0;
        }
    }
    snprintf(tableBuf + used, sizeof(tableBuf) - used, "%s\n", line);
    tableEmpty = false;
    lv_textarea_set_text(ui_txtTable, tableBuf);
    lv_textarea_set_cursor_pos(ui_txtTable, LV_TEXTAREA_CURSOR_LAST);
}

static void switch_label(lv_obj_t *sw, const char *text)
{
    /* the generated screen puts the caption label just above each switch */
    lv_obj_t *par = lv_obj_get_parent(sw);
    if (par == NULL)
        return;
    uint32_t n = lv_obj_get_child_cnt(par);
    for (uint32_t i = 0; i < n; i++)
    {
        lv_obj_t *ch = lv_obj_get_child(par, i);
        if (ch == sw || lv_obj_check_type(ch, &lv_label_class) == false)
            continue;
        /* nearest label above the switch */
        if (lv_obj_get_y(ch) < lv_obj_get_y(sw))
        {
            lv_label_set_text(ch, text);
            return;
        }
    }
}

static void load_config_screen(void)
{
    if (ui_txtSSID)
        lv_textarea_set_text(ui_txtSSID, config.wifi_sta[0].wifi_ssid);
    if (ui_txtPasword)
        lv_textarea_set_text(ui_txtPasword, config.wifi_sta[0].wifi_pass);
    if (ui_txtMyNode)
        lv_textarea_set_text(ui_txtMyNode, config.igate_mycall);
    if (ui_txtFilterNode)
        lv_textarea_set_text(ui_txtFilterNode, config.igate_filter);
#ifdef MQTT
    if (ui_txtMQTTHost)
        lv_textarea_set_text(ui_txtMQTTHost, config.mqtt_host);
    if (ui_txtMQTTPort)
    {
        char port[8];
        snprintf(port, sizeof(port), "%u", (unsigned)config.mqtt_port);
        lv_textarea_set_text(ui_txtMQTTPort, port);
    }
#else
    if (ui_Panel15)
        lv_obj_add_flag(ui_Panel15, LV_OBJ_FLAG_HIDDEN);
    if (ui_txtMQTTHost)
        lv_textarea_set_text(ui_txtMQTTHost, "MQTT not built");
    if (ui_txtMQTTPort)
        lv_textarea_set_text(ui_txtMQTTPort, "-");
#endif

    if (ui_swWiFi)
    {
        if (config.wifi_sta[0].enable)
            lv_obj_add_state(ui_swWiFi, LV_STATE_CHECKED);
        else
            lv_obj_clear_state(ui_swWiFi, LV_STATE_CHECKED);
    }
    if (ui_swAlarm)
    {
        if (config.igate_en)
            lv_obj_add_state(ui_swAlarm, LV_STATE_CHECKED);
        else
            lv_obj_clear_state(ui_swAlarm, LV_STATE_CHECKED);
    }
    if (ui_swMQTT)
    {
#ifdef MQTT
        if (config.en_mqtt)
            lv_obj_add_state(ui_swMQTT, LV_STATE_CHECKED);
        else
            lv_obj_clear_state(ui_swMQTT, LV_STATE_CHECKED);
#else
        lv_obj_add_state(ui_swMQTT, LV_STATE_DISABLED);
#endif
    }
    if (ui_swLoger)
    {
        if (config.rf_en)
            lv_obj_add_state(ui_swLoger, LV_STATE_CHECKED);
        else
            lv_obj_clear_state(ui_swLoger, LV_STATE_CHECKED);
    }

    switch_label(ui_swWiFi, "WiFi STA");
    switch_label(ui_swAlarm, "iGate");
    switch_label(ui_swMQTT, "MQTT");
    switch_label(ui_swLoger, "LoRa RF");

    /* the flame / smoke threshold page belongs to the NAWS alarm box */
    if (ui_TabConfig)
    {
        lv_obj_t *btns = lv_tabview_get_tab_btns(ui_TabConfig);
        if (btns)
        {
            lv_obj_t *btn0 = lv_obj_get_child(btns, 0);
            if (btn0)
                lv_obj_add_flag(btn0, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (ui_TabPageThreadhold)
        lv_obj_add_flag(ui_TabPageThreadhold, LV_OBJ_FLAG_HIDDEN);
}

/* ------------------------------------------------------------------ */
/* public API                                                          */
/* ------------------------------------------------------------------ */
void ui_bind_init(void)
{
    wxPkt.valid = false;
    memset(chartTemp, 0, sizeof(chartTemp));
    memset(chartHum, 0, sizeof(chartHum));
    chart_reset();

    if (ui_ChartWeather)
    {
        serTemp = lv_chart_get_series_next(ui_ChartWeather, NULL);
        if (serTemp)
            serHum = lv_chart_get_series_next(ui_ChartWeather, serTemp);
        if (serTemp)
            lv_chart_set_ext_y_array(ui_ChartWeather, serTemp, chartTemp);
        if (serHum)
            lv_chart_set_ext_y_array(ui_ChartWeather, serHum, chartHum);
        lv_chart_set_range(ui_ChartWeather, LV_CHART_AXIS_PRIMARY_Y, -10, 50);
        lv_chart_set_range(ui_ChartWeather, LV_CHART_AXIS_SECONDARY_Y, 0, 100);
        lv_chart_refresh(ui_ChartWeather);
    }

    if (ui_txtTable)
        lv_textarea_set_text(ui_txtTable, "");

    /* the scaled 480x320 layout leaves a gap: let the tabview fill the panel */
    if (ui_tabHome)
        lv_obj_set_height(ui_tabHome, UI_SCR_H - UI_TOPBAR_H);
    (void)UI_SCR_W;

    load_config_screen();

    lbl_set(ui_lbTX, "TX:");
    lbl_set(ui_lbRX, "RX:");
    lbl_set(ui_lbTime, "--:--:--");
    lbl_set(ui_lbRSSI, "RSSI");
    lbl_set(ui_txtRSSI, "--dBm");

    curScreen = lv_scr_act();
}

void ui_bind_store_wx(void)
{
    wxPkt.flags = aprs.wx_report.flags;
    wxPkt.temp = (float)aprs.wx_report.temp;
    wxPkt.humidity = (float)aprs.wx_report.humidity;
    wxPkt.pressure = (float)aprs.wx_report.pressure;
    wxPkt.rain1h = (float)aprs.wx_report.rain_1h;
    wxPkt.rain24h = (float)aprs.wx_report.rain_24h;
    wxPkt.rainMid = (float)aprs.wx_report.rain_midnight;
    wxPkt.windSpeed = (float)aprs.wx_report.wind_speed;
    wxPkt.windGust = (float)aprs.wx_report.wind_gust;
    wxPkt.windDir = (int)aprs.wx_report.wind_dir;
    wxPkt.luminosity = aprs.wx_report.luminosity;
    wxPkt.uv = aprs.wx_report.uv;
    wxPkt.stamp = millis();
    wxPkt.valid = (aprs.wx_report.flags != 0);

    memset(wxPkt.src, 0, sizeof(wxPkt.src));
    if (aprs.srcname != NULL && aprs.srcname_len > 0)
    {
        size_t n = aprs.srcname_len;
        if (n > sizeof(wxPkt.src) - 1)
            n = sizeof(wxPkt.src) - 1;
        memcpy(wxPkt.src, aprs.srcname, n);
    }
    wxPkt.seq++;
}

void ui_bind_note_rx(void) { rxStamp = millis(); }
void ui_bind_note_tx(void) { txStamp = millis(); }

void ui_bind_show_alert(const char *title, const char *detail)
{
    if (ui_Label14 && title)
        lv_label_set_text(ui_Label14, title);
    if (ui_lbDetection && detail)
        lv_label_set_text(ui_lbDetection, detail);
    alertActive = true;
    /* jump to the warning screen once per alert episode so the user can
       dismiss it with the home button and stay on the dashboard */
    if (!alertShown && ui_scnWaring && lv_scr_act() == ui_scnDashboard)
    {
        alertShown = true;
        _ui_screen_change(&ui_scnWaring, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, &ui_scnWaring_screen_init);
    }
}

void ui_bind_clear_alert(void)
{
    if (!alertActive)
        return;
    alertActive = false;
    alertShown = false;
    if (ui_scnWaring && lv_scr_act() == ui_scnWaring)
        _ui_screen_change(&ui_scnDashboard, LV_SCR_LOAD_ANIM_FADE_ON, 300, 0, &ui_scnDashboard_screen_init);
}

/* ------------------------------------------------------------------ */
/* periodic refresh                                                    */
/* ------------------------------------------------------------------ */
static void update_clock(void)
{
    time_t now = time(NULL);
    struct tm tmv;
    if (now < 1600000000 || localtime_r(&now, &tmv) == NULL)
    {
        lbl_set(ui_lbTime, "--:--:--");
        return;
    }
    lbl_set(ui_lbTime, "%02d:%02d:%02d", tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
}

static void update_status(void)
{
    uint32_t now = millis();
    bool wifiUp = (WiFi.status() == WL_CONNECTED);

    lv_obj_set_style_opa(ui_dotTX, (now - txStamp < 1500) ? 255 : 60, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_dotRX, (now - rxStamp < 1500) ? 255 : 60, LV_PART_MAIN | LV_STATE_DEFAULT);

    if (wifiUp)
        lbl_set(ui_txtRSSI, "%ddBm", (int)WiFi.RSSI());
    else
        lbl_set(ui_txtRSSI, "--dBm");

    lv_obj_set_style_opa(ui_imgWiFI, wifiUp ? 255 : 90, LV_PART_MAIN | LV_STATE_DEFAULT);
#ifdef MQTT
    lv_obj_set_style_opa(ui_imgMQTT, (config.en_mqtt && wifiUp) ? 255 : 90, LV_PART_MAIN | LV_STATE_DEFAULT);
#else
    lv_obj_set_style_opa(ui_imgMQTT, 60, LV_PART_MAIN | LV_STATE_DEFAULT);
#endif
    lv_obj_set_style_opa(ui_imgIGATE, (config.igate_en && wifiUp) ? 255 : 90, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_opa(ui_imgLoRa, config.rf_en ? 255 : 90, LV_PART_MAIN | LV_STATE_DEFAULT);
}

static void update_weather(void)
{
    bool usePkt = wxPkt.valid && (millis() - wxPkt.stamp < UI_WX_PACKET_HOLD_MS);

    uint32_t flags = 0;
    float t = 0, h = 0, p = 0, r1 = 0, r24 = 0, rmid = 0, ws = 0, wg = 0;
    int wd = 0;
    uint32_t lux = 0;
    int uv = 0;
    const char *src = "";

    if (usePkt)
    {
        flags = wxPkt.flags;
        t = wxPkt.temp;
        h = wxPkt.humidity;
        p = wxPkt.pressure;
        r1 = wxPkt.rain1h;
        r24 = wxPkt.rain24h;
        rmid = wxPkt.rainMid;
        ws = wxPkt.windSpeed * 3.6f;
        wg = wxPkt.windGust * 3.6f;
        wd = wxPkt.windDir;
        lux = wxPkt.luminosity;
        uv = wxPkt.uv;
        src = wxPkt.src;
    }
    else if (config.wx_en)
    {
        flags = weather.visable;
        t = weather.temperature;
        h = weather.humidity;
        p = weather.barometric;
        r1 = weather.rain;
        r24 = weather.rain24hr;
        rmid = weather.rainmidnight;
        ws = weather.windspeed;
        wg = weather.windgust;
        wd = weather.winddirection;
        lux = weather.solar;
        uv = weather.uv;
        src = config.wx_mycall;
    }

    if (flags & W_TEMP)
        lbl_set(ui_lbTemperature, "%0.1f", t);
    if (flags & W_HUM)
        lbl_set(ui_lbHumidity, "%0.0f%%", h);
    if ((flags & W_TEMP) && (flags & W_HUM))
    {
        lbl_set(ui_lbDewpoint, "%0.1f", dew_point_c(t, h));
        lbl_set(ui_lbHeatindex, "%0.1f", heat_index_c(t, h));
    }
    if (flags & W_BAR)
        lbl_set(ui_lbBaro, "%0.1f", p);
    if (flags & W_R1H)
        lbl_set(ui_lbRain1hr, "%0.1f", r1);
    if (flags & W_R24H)
        lbl_set(ui_lbRain24hr, "%0.1f", r24);
    if (flags & W_RMN)
        lbl_set(ui_lbRainGMT, "%0.1f", rmid);
    if (flags & W_PAR)
        lbl_set(ui_lbPyrano, "%u", (unsigned)lux);
    if (flags & W_WS)
        lbl_set(ui_lbWindSpeed, "%0.1f", ws);
    if (flags & W_WG)
        lbl_set(ui_lbWindGust, "%0.1f", wg);
    if (flags & W_WD)
    {
        lbl_set(ui_lbWindCorse, "%d", wd);
        if (ui_imgArrow)
            lv_img_set_angle(ui_imgArrow, (int16_t)(wd * 10));
    }
    (void)uv;
    (void)src;
}

static void history_step(void)
{
    uint32_t now = millis();
    bool usePkt = wxPkt.valid && (now - wxPkt.stamp < UI_WX_PACKET_HOLD_MS);
    uint32_t flags = usePkt ? wxPkt.flags : (config.wx_en ? weather.visable : 0);
    if (!(flags & W_TEMP))
        return;

    int16_t t = (int16_t)lroundf(usePkt ? wxPkt.temp : weather.temperature);
    int16_t h = (flags & W_HUM) ? (int16_t)lroundf(usePkt ? wxPkt.humidity : weather.humidity) : (int16_t)LV_CHART_POINT_NONE;

    if (now - lastChartMs >= 60000UL)
    {
        lastChartMs = now;
        chart_push(t, h);
    }
    if (now - lastTableMs >= 300000UL)
    {
        lastTableMs = now;
        time_t tm = time(NULL);
        struct tm tmv;
        char stamp[16] = "--:--";
        if (tm >= 1600000000 && localtime_r(&tm, &tmv) != NULL)
            snprintf(stamp, sizeof(stamp), "%02d:%02d", tmv.tm_hour, tmv.tm_min);
        char line[64];
        snprintf(line, sizeof(line), "%s T=%dC H=%d%% P=%.1fhPa",
                 stamp, (int)t, (int)(h == LV_CHART_POINT_NONE ? 0 : h),
                 (double)(usePkt ? wxPkt.pressure : weather.barometric));
        table_append(line);
    }
}

static void check_alerts(void)
{
    uint32_t now = millis();
    bool wifiUp = (WiFi.status() == WL_CONNECTED);

    if (config.wifi_sta[0].enable && !wifiUp)
    {
        if (wifiDownSince == 0)
            wifiDownSince = now ? now : 1;
        else if (now - wifiDownSince > UI_ALERT_WIFI_MS)
        {
            ui_bind_show_alert("WiFi Lost !", "Cannot connect to the WiFi station");
            return;
        }
    }
    else
    {
        wifiDownSince = 0;
    }

    if (config.trk_en && config.trk_gps && !gps.location.isValid())
    {
        ui_bind_show_alert("GPS Lost !", "Waiting for a valid GPS fix");
        return;
    }

    ui_bind_clear_alert();
}

void ui_bind_tick(void)
{
    uint32_t now = millis();
    if (now - lastTickMs < 500)
        return;
    lastTickMs = now;

    if (curScreen != lv_scr_act())
        curScreen = lv_scr_act();

    update_clock();
    update_status();
    update_weather();
    history_step();

    static uint32_t lastAlertMs = 0;
    if (now - lastAlertMs > 2000)
    {
        lastAlertMs = now;
        check_alerts();
    }
}

#endif /* USE_LVGL_UI */
