// ESP32-S3 Smartwatch - Clock Screen
// LVGL version: 9.3.0

#include "ui.h"

void ui_ScreenClock_screen_init(void)
{
    ui_ScreenClock = lv_obj_create(NULL);
    lv_obj_remove_flag(ui_ScreenClock, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(ui_ScreenClock, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_ScreenClock, 255, LV_PART_MAIN);

    // ===== TIJD (HH:MM) - groot, midden-boven =====
    ui_time_label = lv_label_create(ui_ScreenClock);
    lv_obj_align(ui_time_label, LV_ALIGN_CENTER, 0, -100);
    lv_label_set_text(ui_time_label, "00:00");
    lv_obj_set_style_text_color(ui_time_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(ui_time_label, &lv_font_montserrat_48, 0);

    // ===== SECONDEN - onder de tijd =====
    ui_seconds_label = lv_label_create(ui_ScreenClock);
    lv_obj_align(ui_seconds_label, LV_ALIGN_CENTER, 0, -30);
    lv_label_set_text(ui_seconds_label, "00");
    lv_obj_set_style_text_color(ui_seconds_label, lv_color_hex(0x808080), 0);
    lv_obj_set_style_text_font(ui_seconds_label, &lv_font_montserrat_32, 0);

    // ===== DATUM =====
    ui_date_label = lv_label_create(ui_ScreenClock);
    lv_obj_align(ui_date_label, LV_ALIGN_CENTER, 0, 40);
    lv_label_set_text(ui_date_label, "1 Januari 2026");
    lv_obj_set_style_text_color(ui_date_label, lv_color_hex(0xA0A0A0), 0);
    lv_obj_set_style_text_font(ui_date_label, &lv_font_montserrat_24, 0);

    // ===== DAGNAAM - onder de datum =====
    ui_day_label = lv_label_create(ui_ScreenClock);
    lv_obj_align(ui_day_label, LV_ALIGN_CENTER, 0, 90);
    lv_label_set_text(ui_day_label, "Vrijdag");
    lv_obj_set_style_text_color(ui_day_label, lv_color_hex(0x00C8FF), 0);
    lv_obj_set_style_text_font(ui_day_label, &lv_font_montserrat_24, 0);
}
