// ESP32-S3 Smartwatch - Steps Screen
// LVGL version: 9.3.0

#include "ui.h"

void ui_ScreenSteps_screen_init(void)
{
    ui_ScreenSteps = lv_obj_create(NULL);
    lv_obj_remove_flag(ui_ScreenSteps, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(ui_ScreenSteps, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_ScreenSteps, 255, LV_PART_MAIN);

    // ===== TITEL =====
    ui_steps_title = lv_label_create(ui_ScreenSteps);
    lv_obj_align(ui_steps_title, LV_ALIGN_TOP_MID, 0, 60);
    lv_label_set_text(ui_steps_title, "STAPPEN");
    lv_obj_set_style_text_color(ui_steps_title, lv_color_hex(0xC8C8C8), 0);
    lv_obj_set_style_text_font(ui_steps_title, &lv_font_montserrat_28, 0);

    // ===== STAPPEN TELLER (groot, midden) =====
    ui_steps_count = lv_label_create(ui_ScreenSteps);
    lv_obj_align(ui_steps_count, LV_ALIGN_CENTER, 0, -20);
    lv_label_set_text(ui_steps_count, "0");
    lv_obj_add_flag(ui_steps_count, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_text_color(ui_steps_count, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_text_font(ui_steps_count, &lv_font_montserrat_48, 0);

    // Event handlers
    lv_obj_add_event_cb(ui_steps_count, ui_event_steps_count, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_steps_count, ui_event_steps_count_long_press, LV_EVENT_LONG_PRESSED, NULL);

    // ===== STATUS =====
    ui_steps_status = lv_label_create(ui_ScreenSteps);
    lv_obj_align(ui_steps_status, LV_ALIGN_CENTER, 0, 80);
    lv_label_set_text(ui_steps_status, "ACTIEF");
    lv_obj_set_style_text_color(ui_steps_status, lv_color_hex(0x00C800), 0);
    lv_obj_set_style_text_font(ui_steps_status, &lv_font_montserrat_20, 0);
}
