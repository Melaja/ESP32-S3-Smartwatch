// ESP32-S3 Smartwatch - Files Screen (File Browser)
// LVGL version: 9.3.0
// Auteur: JWP van Renen
// Datum: 18 januari 2026

#include "ui.h"

// Forward declaration voor file selectie event handler
void ui_event_file_selected(lv_event_t * e);

void ui_ScreenFiles_screen_init(void)
{
    // ===== SCHERM AANMAKEN =====
    // Maak het hoofd scherm object
    ui_ScreenFiles = lv_obj_create(NULL);

    // Verwijder scrollable flag van het scherm zelf (de lijst scrollt)
    lv_obj_remove_flag(ui_ScreenFiles, LV_OBJ_FLAG_SCROLLABLE);

    // Zwarte achtergrond (AMOLED - pixels UIT = zuinig)
    lv_obj_set_style_bg_color(ui_ScreenFiles, lv_color_hex(0x000000), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_ScreenFiles, 255, LV_PART_MAIN);

    // ===== TITEL =====
    // Toont "BESTANDEN" bovenaan het scherm
    ui_files_title = lv_label_create(ui_ScreenFiles);
    lv_obj_align(ui_files_title, LV_ALIGN_TOP_MID, 0, 20);
    lv_label_set_text(ui_files_title, "BESTANDEN");
    lv_obj_set_style_text_color(ui_files_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(ui_files_title, &lv_font_montserrat_28, 0);

    // ===== SD KAART STATUS =====
    // Toont of SD kaart beschikbaar is
    ui_files_sd_status = lv_label_create(ui_ScreenFiles);
    lv_obj_align(ui_files_sd_status, LV_ALIGN_TOP_MID, 0, 55);
    lv_label_set_text(ui_files_sd_status, "SD Kaart: Controleren...");
    lv_obj_set_style_text_color(ui_files_sd_status, lv_color_hex(0x808080), 0);
    lv_obj_set_style_text_font(ui_files_sd_status, &lv_font_montserrat_16, 0);

    // ===== BESTANDENLIJST =====
    // Scrollbare lijst met bestanden van de SD kaart
    ui_files_list = lv_list_create(ui_ScreenFiles);

    // Positie en grootte - onder de titel, vult bijna het hele scherm
    lv_obj_set_size(ui_files_list, 380, 380);
    lv_obj_align(ui_files_list, LV_ALIGN_TOP_MID, 0, 85);

    // Stijl: donkere achtergrond voor contrast
    lv_obj_set_style_bg_color(ui_files_list, lv_color_hex(0x1A1A1A), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(ui_files_list, 255, LV_PART_MAIN);

    // Geen rand
    lv_obj_set_style_border_width(ui_files_list, 0, LV_PART_MAIN);

    // Ronde hoeken
    lv_obj_set_style_radius(ui_files_list, 10, LV_PART_MAIN);

    // Padding voor items
    lv_obj_set_style_pad_all(ui_files_list, 5, LV_PART_MAIN);

    // Scrollbar stijl
    lv_obj_set_style_bg_color(ui_files_list, lv_color_hex(0x505050), LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(ui_files_list, 150, LV_PART_SCROLLBAR);

    // ===== INSTRUCTIE LABEL (onderaan) =====
    ui_files_info = lv_label_create(ui_ScreenFiles);
    lv_obj_align(ui_files_info, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_label_set_text(ui_files_info, "Tik op bestand om te openen");
    lv_obj_set_style_text_color(ui_files_info, lv_color_hex(0x606060), 0);
    lv_obj_set_style_text_font(ui_files_info, &lv_font_montserrat_14, 0);
}
