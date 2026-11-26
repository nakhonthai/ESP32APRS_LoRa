/*
 * ui_bind.h - bridge between the ESP32APRS_LoRa firmware and the SquareLine UI
 *
 * Threading rules:
 *   - ui_bind_store_wx() / ui_bind_note_rx() / ui_bind_note_tx() may be called from
 *     any task. They only copy data into a static snapshot.
 *   - ui_bind_init(), ui_bind_tick() and every ui_* LVGL access must run in the
 *     task that calls lv_timer_handler() (the Arduino loop task).
 */
#ifndef UI_BIND_H
#define UI_BIND_H

#ifdef __cplusplus
extern "C" {
#endif

/** Fill the config screen from the runtime configuration and prepare the widgets. */
void ui_bind_init(void);

/** Periodic refresh (clock, RSSI, status icons, weather values, chart/table history). */
void ui_bind_tick(void);

/** Snapshot the WX report of the packet just parsed in aprs.wx_report (any task). */
void ui_bind_store_wx(void);

/** Activity markers used by the TX/RX dots in the top bar (any task). */
void ui_bind_note_rx(void);
void ui_bind_note_tx(void);

/** Force the warning screen (NULL clears it). */
void ui_bind_show_alert(const char *title, const char *detail);
void ui_bind_clear_alert(void);

#ifdef __cplusplus
}
#endif

#endif /* UI_BIND_H */
