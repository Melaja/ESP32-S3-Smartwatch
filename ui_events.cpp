// ESP32-S3 Smartwatch - UI Events
// LVGL version: 9.3.0

#include "ui.h"

// External variables - implement in main code
extern bool stepCountingEnabled;
extern uint32_t stepCount;

// Statische tekst buffers voor stabiele weergave
static const char* STATUS_ACTIVE = "ACTIEF";
static const char* STATUS_PAUSED = "GEPAUZEERD";

// Event handler: Short tap on step count - toggle counting on/off
void ui_event_steps_count(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        // Toggle step counting
        stepCountingEnabled = !stepCountingEnabled;

        if(stepCountingEnabled) {
            // Active - green color
            lv_obj_set_style_text_color(ui_steps_count, lv_color_hex(0x00FF00), 0);
            lv_label_set_text(ui_steps_status, STATUS_ACTIVE);
            lv_obj_set_style_text_color(ui_steps_status, lv_color_hex(0x00C800), 0);
        } else {
            // Paused - red color
            lv_obj_set_style_text_color(ui_steps_count, lv_color_hex(0xFF0000), 0);
            lv_label_set_text(ui_steps_status, STATUS_PAUSED);
            lv_obj_set_style_text_color(ui_steps_status, lv_color_hex(0xC80000), 0);
        }
    }
}

// Event handler: Long press on step count - reset to 0
void ui_event_steps_count_long_press(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_LONG_PRESSED) {
        // Reset step count
        stepCount = 0;
        lv_label_set_text(ui_steps_count, "0");
    }
}

// Event handler: Brightness slider changed
void ui_event_brightness_changed(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_VALUE_CHANGED) {
        lv_obj_t * slider = (lv_obj_t *)lv_event_get_target(e);
        int32_t value = lv_slider_get_value(slider);

        // External function to set brightness (defined in main .ino)
        extern void setDisplayBrightness(uint8_t brightness);
        setDisplayBrightness((uint8_t)value);

        // Register activity to prevent sleep
        extern void registerActivity(void);
        registerActivity();
    }
}
