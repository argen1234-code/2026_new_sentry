//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 2
#define TOTAL_STRING 0

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];

#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
#endif

#define SCAN_AND_SEND() ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, NULL, NULL, TOTAL_FIGURE, TOTAL_STRING)

void ui_init_g() {
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

    SCAN_AND_SEND();

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].operate_type = 2;
    }
}

void ui_update_g() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0) {
            ui_g_dirty_figure[i] = 1;
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }
#endif
    SCAN_AND_SEND();
}
