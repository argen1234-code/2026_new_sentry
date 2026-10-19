//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_g_now_figures[2];
extern uint8_t ui_g_dirty_figure[2];

#define ui_g_Ungroup_auto_aim ((ui_interface_rect_t*)&(ui_g_now_figures[0]))
#define ui_g_Ungroup_Crosshair ((ui_interface_round_t*)&(ui_g_now_figures[1]))


#ifdef MANUAL_DIRTY
#define ui_g_Ungroup_auto_aim_dirty (ui_g_dirty_figure[0])
#define ui_g_Ungroup_Crosshair_dirty (ui_g_dirty_figure[1])

#endif

void ui_init_g();
void ui_update_g();

#endif // UI_g_H
