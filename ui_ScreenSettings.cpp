// ESP32-S3 Smartwatch - Settings Screen
// LVGL version: 9.3.0

#include "ui.h"

void ui_ScreenSettings_screen_init(void)
{
    ui_ScreenSettings = lv_obj_create(NULL);
    lv_obj_remove_flag(ui_ScreenSettings, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(ui_ScreenSettings, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_ScreenSettings, 255, LV_PART_MAIN);

    // ===== TITEL =====
    ui_settings_title = lv_label_create(ui_ScreenSettings);
    lv_obj_align(ui_settings_title, LV_ALIGN_TOP_MID, 0, 30);
    lv_label_set_text(ui_settings_title, "INSTELLINGEN");
    lv_obj_set_style_text_color(ui_settings_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(ui_settings_title, &lv_font_montserrat_32, 0);

    // ===== HELDERHEID SECTIE =====
    ui_brightness_label = lv_label_create(ui_ScreenSettings);
    lv_obj_align(ui_brightness_label, LV_ALIGN_TOP_LEFT, 30, 100);
    lv_label_set_text(ui_brightness_label, "Helderheid");
    lv_obj_set_style_text_color(ui_brightness_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(ui_brightness_label, &lv_font_montserrat_24, 0);

    // Slider
    ui_brightness_slider = lv_slider_create(ui_ScreenSettings);
    lv_slider_set_range(ui_brightness_slider, 10, 255);
    lv_slider_set_value(ui_brightness_slider, 200, LV_ANIM_OFF);
    lv_obj_set_width(ui_brightness_slider, 350);
    lv_obj_set_height(ui_brightness_slider, 30);
    lv_obj_align(ui_brightness_slider, LV_ALIGN_TOP_MID, 0, 150);
    lv_obj_set_style_bg_color(ui_brightness_slider, lv_color_hex(0x404040), LV_PART_MAIN);
    lv_obj_set_style_bg_color(ui_brightness_slider, lv_color_hex(0xFFA500), LV_PART_INDICATOR);
    lv_obj_set_style_bg_color(ui_brightness_slider, lv_color_hex(0xFFFFFF), LV_PART_KNOB);
    lv_obj_add_event_cb(ui_brightness_slider, ui_event_brightness_changed, LV_EVENT_VALUE_CHANGED, NULL);

    // ===== BATTERIJ SECTIE =====
    ui_battery_label = lv_label_create(ui_ScreenSettings);
    lv_obj_align(ui_battery_label, LV_ALIGN_TOP_LEFT, 30, 220);
    lv_label_set_text(ui_battery_label, "Batterij");
    lv_obj_set_style_text_color(ui_battery_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(ui_battery_label, &lv_font_montserrat_24, 0);

    ui_battery_value = lv_label_create(ui_ScreenSettings);
    lv_obj_align(ui_battery_value, LV_ALIGN_TOP_RIGHT, -30, 220);
    lv_label_set_text(ui_battery_value, "100%");
    lv_obj_set_style_text_color(ui_battery_value, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_text_font(ui_battery_value, &lv_font_montserrat_24, 0);

    // ===== WIFI SECTIE =====
    ui_wifi_label = lv_label_create(ui_ScreenSettings);
    lv_obj_align(ui_wifi_label, LV_ALIGN_TOP_LEFT, 30, 280);
    lv_label_set_text(ui_wifi_label, "WiFi");
    lv_obj_set_style_text_color(ui_wifi_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(ui_wifi_label, &lv_font_montserrat_24, 0);

    ui_wifi_status = lv_label_create(ui_ScreenSettings);
    lv_obj_align(ui_wifi_status, LV_ALIGN_TOP_RIGHT, -30, 280);
    lv_label_set_text(ui_wifi_status, "Verbonden");
    lv_obj_set_style_text_color(ui_wifi_status, lv_color_hex(0x00C8FF), 0);
    lv_obj_set_style_text_font(ui_wifi_status, &lv_font_montserrat_24, 0);

    // ===== VERSIE =====
    ui_version_label = lv_label_create(ui_ScreenSettings);
    lv_obj_align(ui_version_label, LV_ALIGN_BOTTOM_MID, 0, -30);
    lv_label_set_text(ui_version_label, "LVGL v1.2.0");
    lv_obj_set_style_text_color(ui_version_label, lv_color_hex(0x606060), 0);
    lv_obj_set_style_text_font(ui_version_label, &lv_font_montserrat_18, 0);
}
