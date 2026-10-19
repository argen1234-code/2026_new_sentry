//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 4
#define TOTAL_STRING 10

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];
ui_interface_string_t ui_g_now_strings[TOTAL_STRING];
uint8_t ui_g_dirty_string[TOTAL_STRING];

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_last_strings[TOTAL_STRING];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_g(void) {
    ui_g_Ungroup_super_capacity_num->figure_type = 6;
    ui_g_Ungroup_super_capacity_num->operate_type = 1;
    ui_g_Ungroup_super_capacity_num->layer = 0;
    ui_g_Ungroup_super_capacity_num->color = 0;
    ui_g_Ungroup_super_capacity_num->start_x = 1630;
    ui_g_Ungroup_super_capacity_num->start_y = 800;
    ui_g_Ungroup_super_capacity_num->width = 2;
    ui_g_Ungroup_super_capacity_num->font_size = 20;
    ui_g_Ungroup_super_capacity_num->number = 1;

    ui_g_Ungroup_shoot_speed_number->figure_type = 6;
    ui_g_Ungroup_shoot_speed_number->operate_type = 1;
    ui_g_Ungroup_shoot_speed_number->layer = 0;
    ui_g_Ungroup_shoot_speed_number->color = 0;
    ui_g_Ungroup_shoot_speed_number->start_x = 1630;
    ui_g_Ungroup_shoot_speed_number->start_y = 650;
    ui_g_Ungroup_shoot_speed_number->width = 2;
    ui_g_Ungroup_shoot_speed_number->font_size = 20;
    ui_g_Ungroup_shoot_speed_number->number = 0;

    ui_g_Ungroup_auto_aim->figure_type = 1;
    ui_g_Ungroup_auto_aim->operate_type = 1;
    ui_g_Ungroup_auto_aim->layer = 0;
    ui_g_Ungroup_auto_aim->color = 0;
    ui_g_Ungroup_auto_aim->start_x = 649;
    ui_g_Ungroup_auto_aim->start_y = 203;
    ui_g_Ungroup_auto_aim->width = 1;
    ui_g_Ungroup_auto_aim->end_x = 1249;
    ui_g_Ungroup_auto_aim->end_y = 803;

    ui_g_Ungroup_Crosshair->figure_type = 2;
    ui_g_Ungroup_Crosshair->operate_type = 1;
    ui_g_Ungroup_Crosshair->layer = 0;
    ui_g_Ungroup_Crosshair->color = 0;
    ui_g_Ungroup_Crosshair->start_x = 958;
    ui_g_Ungroup_Crosshair->start_y = 441;
    ui_g_Ungroup_Crosshair->width = 5;
    ui_g_Ungroup_Crosshair->r = 15;

    ui_g_Ungroup_friction->figure_type = 7;
    ui_g_Ungroup_friction->operate_type = 1;
    ui_g_Ungroup_friction->layer = 0;
    ui_g_Ungroup_friction->color = 0;
    ui_g_Ungroup_friction->start_x = 1529;
    ui_g_Ungroup_friction->start_y = 847;
    ui_g_Ungroup_friction->width = 2;
    ui_g_Ungroup_friction->font_size = 20;
    ui_g_Ungroup_friction->str_length = 4;
    strcpy(ui_g_Ungroup_friction->string, "FRI:");

    ui_g_Ungroup_gimbal_mode->figure_type = 7;
    ui_g_Ungroup_gimbal_mode->operate_type = 1;
    ui_g_Ungroup_gimbal_mode->layer = 0;
    ui_g_Ungroup_gimbal_mode->color = 0;
    ui_g_Ungroup_gimbal_mode->start_x = 1489;
    ui_g_Ungroup_gimbal_mode->start_y = 697;
    ui_g_Ungroup_gimbal_mode->width = 2;
    ui_g_Ungroup_gimbal_mode->font_size = 20;
    ui_g_Ungroup_gimbal_mode->str_length = 7;
    strcpy(ui_g_Ungroup_gimbal_mode->string, "gimbal:");

    ui_g_Ungroup_shoot_speed_text->figure_type = 7;
    ui_g_Ungroup_shoot_speed_text->operate_type = 1;
    ui_g_Ungroup_shoot_speed_text->layer = 0;
    ui_g_Ungroup_shoot_speed_text->color = 0;
    ui_g_Ungroup_shoot_speed_text->start_x = 1384;
    ui_g_Ungroup_shoot_speed_text->start_y = 647;
    ui_g_Ungroup_shoot_speed_text->width = 2;
    ui_g_Ungroup_shoot_speed_text->font_size = 20;
    ui_g_Ungroup_shoot_speed_text->str_length = 12;
    strcpy(ui_g_Ungroup_shoot_speed_text->string, "shoot_speed:");

    ui_g_Ungroup_friction_mode->figure_type = 7;
    ui_g_Ungroup_friction_mode->operate_type = 1;
    ui_g_Ungroup_friction_mode->layer = 0;
    ui_g_Ungroup_friction_mode->color = 0;
    ui_g_Ungroup_friction_mode->start_x = 1630;
    ui_g_Ungroup_friction_mode->start_y = 850;
    ui_g_Ungroup_friction_mode->width = 2;
    ui_g_Ungroup_friction_mode->font_size = 20;
    ui_g_Ungroup_friction_mode->str_length = 4;
    strcpy(ui_g_Ungroup_friction_mode->string, "mode");

    ui_g_Ungroup_aim_mode->figure_type = 7;
    ui_g_Ungroup_aim_mode->operate_type = 1;
    ui_g_Ungroup_aim_mode->layer = 0;
    ui_g_Ungroup_aim_mode->color = 0;
    ui_g_Ungroup_aim_mode->start_x = 1630;
    ui_g_Ungroup_aim_mode->start_y = 900;
    ui_g_Ungroup_aim_mode->width = 2;
    ui_g_Ungroup_aim_mode->font_size = 20;
    ui_g_Ungroup_aim_mode->str_length = 2;
    strcpy(ui_g_Ungroup_aim_mode->string, "on");

    ui_g_Ungroup_chassis_dis->figure_type = 7;
    ui_g_Ungroup_chassis_dis->operate_type = 1;
    ui_g_Ungroup_chassis_dis->layer = 0;
    ui_g_Ungroup_chassis_dis->color = 0;
    ui_g_Ungroup_chassis_dis->start_x = 1636;
    ui_g_Ungroup_chassis_dis->start_y = 750;
    ui_g_Ungroup_chassis_dis->width = 2;
    ui_g_Ungroup_chassis_dis->font_size = 20;
    ui_g_Ungroup_chassis_dis->str_length = 6;
    strcpy(ui_g_Ungroup_chassis_dis->string, "gimbal");

    ui_g_Ungroup_gimbal_dis->figure_type = 7;
    ui_g_Ungroup_gimbal_dis->operate_type = 1;
    ui_g_Ungroup_gimbal_dis->layer = 0;
    ui_g_Ungroup_gimbal_dis->color = 0;
    ui_g_Ungroup_gimbal_dis->start_x = 1630;
    ui_g_Ungroup_gimbal_dis->start_y = 700;
    ui_g_Ungroup_gimbal_dis->width = 2;
    ui_g_Ungroup_gimbal_dis->font_size = 20;
    ui_g_Ungroup_gimbal_dis->str_length = 6;
    strcpy(ui_g_Ungroup_gimbal_dis->string, "encode");

    ui_g_Ungroup_superCap_text->figure_type = 7;
    ui_g_Ungroup_superCap_text->operate_type = 1;
    ui_g_Ungroup_superCap_text->layer = 0;
    ui_g_Ungroup_superCap_text->color = 0;
    ui_g_Ungroup_superCap_text->start_x = 1509;
    ui_g_Ungroup_superCap_text->start_y = 797;
    ui_g_Ungroup_superCap_text->width = 2;
    ui_g_Ungroup_superCap_text->font_size = 20;
    ui_g_Ungroup_superCap_text->str_length = 6;
    strcpy(ui_g_Ungroup_superCap_text->string, "super:");

    ui_g_Ungroup_aim_switch->figure_type = 7;
    ui_g_Ungroup_aim_switch->operate_type = 1;
    ui_g_Ungroup_aim_switch->layer = 0;
    ui_g_Ungroup_aim_switch->color = 0;
    ui_g_Ungroup_aim_switch->start_x = 1529;
    ui_g_Ungroup_aim_switch->start_y = 897;
    ui_g_Ungroup_aim_switch->width = 2;
    ui_g_Ungroup_aim_switch->font_size = 20;
    ui_g_Ungroup_aim_switch->str_length = 5;
    strcpy(ui_g_Ungroup_aim_switch->string, "aim: ");

    ui_g_Ungroup_chassis_mode->figure_type = 7;
    ui_g_Ungroup_chassis_mode->operate_type = 1;
    ui_g_Ungroup_chassis_mode->layer = 0;
    ui_g_Ungroup_chassis_mode->color = 0;
    ui_g_Ungroup_chassis_mode->start_x = 1469;
    ui_g_Ungroup_chassis_mode->start_y = 747;
    ui_g_Ungroup_chassis_mode->width = 2;
    ui_g_Ungroup_chassis_mode->font_size = 20;
    ui_g_Ungroup_chassis_mode->str_length = 8;
    strcpy(ui_g_Ungroup_chassis_mode->string, "chassis:");

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_figures[i] = ui_g_now_figures[i];
#endif
        ui_g_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_g_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_strings[i].operate_type = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_strings[i] = ui_g_now_strings[i];
#endif
        ui_g_dirty_string[i] = 1;
        idx++;
    }

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].operate_type = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].operate_type = 2;
    }
}

void ui_update_g(void) {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0) {
            ui_g_dirty_figure[i] = 1;
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_g_now_strings[i], &ui_g_last_strings[i], sizeof(ui_g_now_strings[i])) != 0) {
            ui_g_dirty_string[i] = 1;
            ui_g_last_strings[i] = ui_g_now_strings[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
