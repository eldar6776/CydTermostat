// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.2
// LVGL version: 8.3.6
// Project name: termostat_V1

#include "../ui.h"

void ui_Settings_Wifi_Bus_screen_init(void)
{
    ui_Settings_Wifi_Bus = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Settings_Wifi_Bus, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Settings_Wifi_Bus, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Settings_Wifi_Bus, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_Settings_Wifi_Bus, &ui_img_settings_bg_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Group_Header1 = lv_obj_create(ui_Settings_Wifi_Bus);
    lv_obj_set_height(ui_Group_Header1, 24);
    lv_obj_set_width(ui_Group_Header1, lv_pct(100));
    lv_obj_set_align(ui_Group_Header1, LV_ALIGN_TOP_MID);
    lv_obj_set_flex_flow(ui_Group_Header1, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(ui_Group_Header1, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_clear_flag(ui_Group_Header1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Group_Header1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Group_Header1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Head_Group2 = lv_obj_create(ui_Group_Header1);
    lv_obj_set_height(ui_Head_Group2, lv_pct(110));
    lv_obj_set_flex_grow(ui_Head_Group2, 1);
    lv_obj_set_align(ui_Head_Group2, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Head_Group2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Head_Group2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Head_Group2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui_Head_Group2, 12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui_Head_Group2, 55, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui_Head_Group2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui_Head_Group2, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label_Time2 = lv_label_create(ui_Head_Group2);
    lv_obj_set_width(ui_Label_Time2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label_Time2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label_Time2, -28);
    lv_obj_set_y(ui_Label_Time2, 0);
    lv_obj_set_align(ui_Label_Time2, LV_ALIGN_RIGHT_MID);
    lv_label_set_text(ui_Label_Time2, "11:45");
    lv_obj_set_style_text_color(ui_Label_Time2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_Label_Time2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_Label_Time2, &ui_font_Medium, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabeL_PM1 = lv_label_create(ui_Head_Group2);
    lv_obj_set_width(ui_LabeL_PM1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabeL_PM1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabeL_PM1, -7);
    lv_obj_set_y(ui_LabeL_PM1, 0);
    lv_obj_set_align(ui_LabeL_PM1, LV_ALIGN_RIGHT_MID);
    lv_label_set_text(ui_LabeL_PM1, "PM");
    lv_obj_set_style_text_color(ui_LabeL_PM1, lv_color_hex(0x94AEB4), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabeL_PM1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabeL_PM1, &ui_font_Small, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Chart_Group = lv_obj_create(ui_Settings_Wifi_Bus);
    lv_obj_set_height(ui_Chart_Group, 248);
    lv_obj_set_width(ui_Chart_Group, lv_pct(98));
    lv_obj_set_x(ui_Chart_Group, 0);
    lv_obj_set_y(ui_Chart_Group, 7);
    lv_obj_set_align(ui_Chart_Group, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Chart_Group, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Chart_Group, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Chart_Group, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Checkbox1 = lv_checkbox_create(ui_Chart_Group);
    lv_checkbox_set_text(ui_Checkbox1, " TH 1");
    lv_obj_set_width(ui_Checkbox1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Checkbox1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Checkbox1, 75);
    lv_obj_set_y(ui_Checkbox1, -70);
    lv_obj_set_align(ui_Checkbox1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Checkbox1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags

    ui_Checkbox2 = lv_checkbox_create(ui_Chart_Group);
    lv_checkbox_set_text(ui_Checkbox2, " TH 2");
    lv_obj_set_width(ui_Checkbox2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Checkbox2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Checkbox2, 76);
    lv_obj_set_y(ui_Checkbox2, -46);
    lv_obj_set_align(ui_Checkbox2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Checkbox2, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags

    ui_Checkbox3 = lv_checkbox_create(ui_Chart_Group);
    lv_checkbox_set_text(ui_Checkbox3, " TH 3");
    lv_obj_set_width(ui_Checkbox3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Checkbox3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Checkbox3, 76);
    lv_obj_set_y(ui_Checkbox3, -22);
    lv_obj_set_align(ui_Checkbox3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Checkbox3, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags

    ui_Checkbox4 = lv_checkbox_create(ui_Chart_Group);
    lv_checkbox_set_text(ui_Checkbox4, " TH 4");
    lv_obj_set_width(ui_Checkbox4, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Checkbox4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Checkbox4, 77);
    lv_obj_set_y(ui_Checkbox4, 2);
    lv_obj_set_align(ui_Checkbox4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Checkbox4, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags

    ui_Checkbox5 = lv_checkbox_create(ui_Chart_Group);
    lv_checkbox_set_text(ui_Checkbox5, " TH 5");
    lv_obj_set_width(ui_Checkbox5, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Checkbox5, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Checkbox5, 76);
    lv_obj_set_y(ui_Checkbox5, 26);
    lv_obj_set_align(ui_Checkbox5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Checkbox5, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags

    ui_Checkbox6 = lv_checkbox_create(ui_Chart_Group);
    lv_checkbox_set_text(ui_Checkbox6, " TH 6");
    lv_obj_set_width(ui_Checkbox6, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Checkbox6, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Checkbox6, 77);
    lv_obj_set_y(ui_Checkbox6, 50);
    lv_obj_set_align(ui_Checkbox6, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Checkbox6, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags

    ui_Checkbox7 = lv_checkbox_create(ui_Chart_Group);
    lv_checkbox_set_text(ui_Checkbox7, " TH 7");
    lv_obj_set_width(ui_Checkbox7, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Checkbox7, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Checkbox7, 76);
    lv_obj_set_y(ui_Checkbox7, 74);
    lv_obj_set_align(ui_Checkbox7, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Checkbox7, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags

    ui_Checkbox8 = lv_checkbox_create(ui_Chart_Group);
    lv_checkbox_set_text(ui_Checkbox8, " TH 8");
    lv_obj_set_width(ui_Checkbox8, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Checkbox8, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Checkbox8, 77);
    lv_obj_set_y(ui_Checkbox8, 98);
    lv_obj_set_align(ui_Checkbox8, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Checkbox8, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags

    ui_Dropdown1 = lv_dropdown_create(ui_Chart_Group);
    lv_dropdown_set_options(ui_Dropdown1, "12VAC\n16VAC\n24VAC\n36VAC");
    lv_obj_set_width(ui_Dropdown1, 70);
    lv_obj_set_height(ui_Dropdown1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Dropdown1, 190);
    lv_obj_set_y(ui_Dropdown1, -69);
    lv_obj_set_align(ui_Dropdown1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Dropdown1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags



    ui_Dropdown2 = lv_dropdown_create(ui_Chart_Group);
    lv_dropdown_set_options(ui_Dropdown2, "12VAC\n16VAC\n24VAC\n36VAC");
    lv_obj_set_width(ui_Dropdown2, 70);
    lv_obj_set_height(ui_Dropdown2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Dropdown2, 190);
    lv_obj_set_y(ui_Dropdown2, -45);
    lv_obj_set_align(ui_Dropdown2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Dropdown2, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags



    ui_Dropdown3 = lv_dropdown_create(ui_Chart_Group);
    lv_dropdown_set_options(ui_Dropdown3, "12VAC\n16VAC\n24VAC\n36VAC");
    lv_obj_set_width(ui_Dropdown3, 70);
    lv_obj_set_height(ui_Dropdown3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Dropdown3, 190);
    lv_obj_set_y(ui_Dropdown3, -21);
    lv_obj_set_align(ui_Dropdown3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Dropdown3, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags



    ui_Dropdown4 = lv_dropdown_create(ui_Chart_Group);
    lv_dropdown_set_options(ui_Dropdown4, "12VAC\n16VAC\n24VAC\n36VAC");
    lv_obj_set_width(ui_Dropdown4, 70);
    lv_obj_set_height(ui_Dropdown4, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Dropdown4, 190);
    lv_obj_set_y(ui_Dropdown4, 3);
    lv_obj_set_align(ui_Dropdown4, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Dropdown4, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags



    ui_Dropdown5 = lv_dropdown_create(ui_Chart_Group);
    lv_dropdown_set_options(ui_Dropdown5, "12VAC\n16VAC\n24VAC\n36VAC");
    lv_obj_set_width(ui_Dropdown5, 70);
    lv_obj_set_height(ui_Dropdown5, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Dropdown5, 190);
    lv_obj_set_y(ui_Dropdown5, 27);
    lv_obj_set_align(ui_Dropdown5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Dropdown5, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags



    ui_Dropdown6 = lv_dropdown_create(ui_Chart_Group);
    lv_dropdown_set_options(ui_Dropdown6, "12VAC\n16VAC\n24VAC\n36VAC");
    lv_obj_set_width(ui_Dropdown6, 70);
    lv_obj_set_height(ui_Dropdown6, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Dropdown6, 190);
    lv_obj_set_y(ui_Dropdown6, 51);
    lv_obj_set_align(ui_Dropdown6, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Dropdown6, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags



    ui_Dropdown7 = lv_dropdown_create(ui_Chart_Group);
    lv_dropdown_set_options(ui_Dropdown7, "12VAC\n16VAC\n24VAC\n36VAC");
    lv_obj_set_width(ui_Dropdown7, 70);
    lv_obj_set_height(ui_Dropdown7, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Dropdown7, 190);
    lv_obj_set_y(ui_Dropdown7, 75);
    lv_obj_set_align(ui_Dropdown7, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Dropdown7, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags



    ui_Dropdown8 = lv_dropdown_create(ui_Chart_Group);
    lv_dropdown_set_options(ui_Dropdown8, "12VAC\n16VAC\n24VAC\n36VAC");
    lv_obj_set_width(ui_Dropdown8, 70);
    lv_obj_set_height(ui_Dropdown8, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Dropdown8, 190);
    lv_obj_set_y(ui_Dropdown8, 99);
    lv_obj_set_align(ui_Dropdown8, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Dropdown8, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags



    ui_lblRSBUSSETTINGS = lv_label_create(ui_Chart_Group);
    lv_obj_set_width(ui_lblRSBUSSETTINGS, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblRSBUSSETTINGS, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lblRSBUSSETTINGS, -7);
    lv_obj_set_y(ui_lblRSBUSSETTINGS, 20);
    lv_obj_set_align(ui_lblRSBUSSETTINGS, LV_ALIGN_TOP_RIGHT);
    lv_label_set_text(ui_lblRSBUSSETTINGS, "RS485 Bus Settings");
    lv_obj_set_style_text_color(ui_lblRSBUSSETTINGS, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_lblRSBUSSETTINGS, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_lblRSBUSSETTINGS, &ui_font_Big, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Keyboard1 = lv_keyboard_create(ui_Settings_Wifi_Bus);
    lv_obj_set_width(ui_Keyboard1, 468);
    lv_obj_set_height(ui_Keyboard1, 118);
    lv_obj_set_x(ui_Keyboard1, 2);
    lv_obj_set_y(ui_Keyboard1, 67);
    lv_obj_set_align(ui_Keyboard1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Keyboard1, LV_OBJ_FLAG_HIDDEN);     /// Flags
    lv_obj_set_style_bg_color(ui_Keyboard1, lv_color_hex(0x4D1A1A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Keyboard1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_lblCONNECTED = lv_label_create(ui_Settings_Wifi_Bus);
    lv_obj_set_width(ui_lblCONNECTED, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblCONNECTED, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lblCONNECTED, -64);
    lv_obj_set_y(ui_lblCONNECTED, 30);
    lv_obj_set_align(ui_lblCONNECTED, LV_ALIGN_CENTER);
    lv_label_set_text(ui_lblCONNECTED, "NOT CONNECTED");

    ui_Checkbox9 = lv_checkbox_create(ui_Settings_Wifi_Bus);
    lv_checkbox_set_text(ui_Checkbox9, " WEB SERVER");
    lv_obj_set_width(ui_Checkbox9, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Checkbox9, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Checkbox9, -151);
    lv_obj_set_y(ui_Checkbox9, 75);
    lv_obj_set_align(ui_Checkbox9, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Checkbox9, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags

    ui_lblWIFISETTINGS = lv_label_create(ui_Settings_Wifi_Bus);
    lv_obj_set_width(ui_lblWIFISETTINGS, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblWIFISETTINGS, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lblWIFISETTINGS, -342);
    lv_obj_set_y(ui_lblWIFISETTINGS, 38);
    lv_obj_set_align(ui_lblWIFISETTINGS, LV_ALIGN_TOP_RIGHT);
    lv_label_set_text(ui_lblWIFISETTINGS, "WiFi Settings");
    lv_obj_set_style_text_color(ui_lblWIFISETTINGS, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_lblWIFISETTINGS, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_lblWIFISETTINGS, &ui_font_Big, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_btnCONNECT = lv_btn_create(ui_Settings_Wifi_Bus);
    lv_obj_set_width(ui_btnCONNECT, 74);
    lv_obj_set_height(ui_btnCONNECT, 31);
    lv_obj_set_x(ui_btnCONNECT, -175);
    lv_obj_set_y(ui_btnCONNECT, 39);
    lv_obj_set_align(ui_btnCONNECT, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_btnCONNECT, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_btnCONNECT, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_lblCONNECT = lv_label_create(ui_Settings_Wifi_Bus);
    lv_obj_set_width(ui_lblCONNECT, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblCONNECT, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lblCONNECT, -176);
    lv_obj_set_y(ui_lblCONNECT, 39);
    lv_obj_set_align(ui_lblCONNECT, LV_ALIGN_CENTER);
    lv_label_set_text(ui_lblCONNECT, "CONNECT");

    ui_lblPASSWORD = lv_label_create(ui_Settings_Wifi_Bus);
    lv_obj_set_width(ui_lblPASSWORD, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblPASSWORD, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lblPASSWORD, -180);
    lv_obj_set_y(ui_lblPASSWORD, -37);
    lv_obj_set_align(ui_lblPASSWORD, LV_ALIGN_CENTER);
    lv_label_set_text(ui_lblPASSWORD, "Password:");

    ui_lblNAME = lv_label_create(ui_Settings_Wifi_Bus);
    lv_obj_set_width(ui_lblNAME, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblNAME, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lblNAME, -195);
    lv_obj_set_y(ui_lblNAME, -64);
    lv_obj_set_align(ui_lblNAME, LV_ALIGN_CENTER);
    lv_label_set_text(ui_lblNAME, "Name:");

    ui_txtPASSWORD = lv_textarea_create(ui_Settings_Wifi_Bus);
    lv_obj_set_width(ui_txtPASSWORD, 120);
    lv_obj_set_height(ui_txtPASSWORD, LV_SIZE_CONTENT);    /// 19
    lv_obj_set_x(ui_txtPASSWORD, -70);
    lv_obj_set_y(ui_txtPASSWORD, -38);
    lv_obj_set_align(ui_txtPASSWORD, LV_ALIGN_CENTER);
    lv_textarea_set_max_length(ui_txtPASSWORD, 16);
    lv_textarea_set_placeholder_text(ui_txtPASSWORD, "Password");
    lv_textarea_set_one_line(ui_txtPASSWORD, true);
    lv_textarea_set_password_mode(ui_txtPASSWORD, true);
    lv_obj_set_style_bg_color(ui_txtPASSWORD, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_txtPASSWORD, 20, LV_PART_MAIN | LV_STATE_DEFAULT);



    ui_txtUSERNAME = lv_textarea_create(ui_Settings_Wifi_Bus);
    lv_obj_set_width(ui_txtUSERNAME, 119);
    lv_obj_set_height(ui_txtUSERNAME, LV_SIZE_CONTENT);    /// 19
    lv_obj_set_x(ui_txtUSERNAME, -71);
    lv_obj_set_y(ui_txtUSERNAME, -63);
    lv_obj_set_align(ui_txtUSERNAME, LV_ALIGN_CENTER);
    lv_textarea_set_max_length(ui_txtUSERNAME, 16);
    lv_textarea_set_placeholder_text(ui_txtUSERNAME, "Username");
    lv_textarea_set_one_line(ui_txtUSERNAME, true);
    lv_obj_set_style_bg_color(ui_txtUSERNAME, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_txtUSERNAME, 20, LV_PART_MAIN | LV_STATE_DEFAULT);



    ui_btnSEARCH = lv_btn_create(ui_Settings_Wifi_Bus);
    lv_obj_set_width(ui_btnSEARCH, 74);
    lv_obj_set_height(ui_btnSEARCH, 31);
    lv_obj_set_x(ui_btnSEARCH, -175);
    lv_obj_set_y(ui_btnSEARCH, -1);
    lv_obj_set_align(ui_btnSEARCH, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_btnSEARCH, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_btnSEARCH, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_lblSEARCH = lv_label_create(ui_Settings_Wifi_Bus);
    lv_obj_set_width(ui_lblSEARCH, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblSEARCH, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lblSEARCH, -175);
    lv_obj_set_y(ui_lblSEARCH, 0);
    lv_obj_set_align(ui_lblSEARCH, LV_ALIGN_CENTER);
    lv_label_set_text(ui_lblSEARCH, "SEARCH");

    ui_dplWIFILIST = lv_dropdown_create(ui_Settings_Wifi_Bus);
    lv_dropdown_set_options(ui_dplWIFILIST, "WiFi0\nMojaTV\nJaskoWiFi");
    lv_obj_set_width(ui_dplWIFILIST, 119);
    lv_obj_set_height(ui_dplWIFILIST, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_dplWIFILIST, -70);
    lv_obj_set_y(ui_dplWIFILIST, 1);
    lv_obj_set_align(ui_dplWIFILIST, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_dplWIFILIST, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags



    ui_Menu1 = lv_img_create(ui_Settings_Wifi_Bus);
    lv_img_set_src(ui_Menu1, &ui_img_settings_corner_png);
    lv_obj_set_width(ui_Menu1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Menu1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Menu1, LV_ALIGN_TOP_RIGHT);
    lv_obj_add_flag(ui_Menu1, LV_OBJ_FLAG_ADV_HITTEST | LV_OBJ_FLAG_FLOATING);     /// Flags
    lv_obj_clear_flag(ui_Menu1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_BTN_Settings1 = lv_btn_create(ui_Menu1);
    lv_obj_set_width(ui_BTN_Settings1, 35);
    lv_obj_set_height(ui_BTN_Settings1, 30);
    lv_obj_set_align(ui_BTN_Settings1, LV_ALIGN_TOP_RIGHT);
    lv_obj_add_flag(ui_BTN_Settings1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_BTN_Settings1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_BTN_Settings1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BTN_Settings1, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_img_src(ui_BTN_Settings1, &ui_img_icn_x2_png, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label2 = lv_label_create(ui_Settings_Wifi_Bus);
    lv_obj_set_width(ui_Label2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label2, -80);
    lv_obj_set_y(ui_Label2, 50);
    lv_obj_set_align(ui_Label2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label2, "IP: 192.168.0.2");

    ui_Dropdown9 = lv_dropdown_create(ui_Settings_Wifi_Bus);
    lv_dropdown_set_options(ui_Dropdown9, "0,5\n1\n1,5\n2\n2,5\n3\n3,5\n4\n4,5\n5");
    lv_obj_set_width(ui_Dropdown9, 54);
    lv_obj_set_height(ui_Dropdown9, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Dropdown9, -182);
    lv_obj_set_y(ui_Dropdown9, 104);
    lv_obj_set_align(ui_Dropdown9, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Dropdown9, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags



    ui_Label1 = lv_label_create(ui_Settings_Wifi_Bus);
    lv_obj_set_width(ui_Label1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label1, -113);
    lv_obj_set_y(ui_Label1, 103);
    lv_obj_set_align(ui_Label1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label1, "Hysteresis");

    ui_Dropdown10 = lv_dropdown_create(ui_Settings_Wifi_Bus);
    lv_dropdown_set_options(ui_Dropdown10,
                            "-5\n-4.5\n-4\n-3.5\n-3\n-2.5\n-2\n-1.5\n-1\n-0.5\n0\n0.5\n1\n1.5\n2\n2.5\n3\n3.5\n4\n4.5\n5");
    lv_obj_set_width(ui_Dropdown10, 57);
    lv_obj_set_height(ui_Dropdown10, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Dropdown10, -41);
    lv_obj_set_y(ui_Dropdown10, 105);
    lv_obj_set_align(ui_Dropdown10, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Dropdown10, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags



    ui_Label3 = lv_label_create(ui_Settings_Wifi_Bus);
    lv_obj_set_width(ui_Label3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label3, 15);
    lv_obj_set_y(ui_Label3, 105);
    lv_obj_set_align(ui_Label3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label3, "Offset");

    lv_obj_add_event_cb(ui_Checkbox1, ui_event_Checkbox1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Checkbox2, ui_event_Checkbox2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Checkbox3, ui_event_Checkbox3, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Checkbox4, ui_event_Checkbox4, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Checkbox5, ui_event_Checkbox5, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Checkbox6, ui_event_Checkbox6, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Checkbox7, ui_event_Checkbox7, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Checkbox8, ui_event_Checkbox8, LV_EVENT_ALL, NULL);
    lv_keyboard_set_textarea(ui_Keyboard1, ui_txtUSERNAME);
    lv_obj_add_event_cb(ui_Keyboard1, ui_event_Keyboard1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Checkbox9, ui_event_Checkbox9, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_btnCONNECT, ui_event_btnCONNECT, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_txtPASSWORD, ui_event_txtPASSWORD, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_txtUSERNAME, ui_event_txtUSERNAME, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_btnSEARCH, ui_event_btnSEARCH, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_BTN_Settings1, ui_event_BTN_Settings1, LV_EVENT_ALL, NULL);

}
