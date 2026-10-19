//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_g_now_figures[4];
extern uint8_t ui_g_dirty_figure[4];
extern ui_interface_string_t ui_g_now_strings[10];
extern uint8_t ui_g_dirty_string[10];

#define ui_g_Ungroup_super_capacity_num ((ui_interface_number_t*)&(ui_g_now_figures[0]))
#define ui_g_Ungroup_shoot_speed_number ((ui_interface_number_t*)&(ui_g_now_figures[1]))
#define ui_g_Ungroup_auto_aim ((ui_interface_rect_t*)&(ui_g_now_figures[2]))
#define ui_g_Ungroup_Crosshair ((ui_interface_round_t*)&(ui_g_now_figures[3]))

#define ui_g_Ungroup_friction (&(ui_g_now_strings[0]))
#define ui_g_Ungroup_gimbal_mode (&(ui_g_now_strings[1]))
#define ui_g_Ungroup_shoot_speed_text (&(ui_g_now_strings[2]))
#define ui_g_Ungroup_friction_mode (&(ui_g_now_strings[3]))
#define ui_g_Ungroup_aim_mode (&(ui_g_now_strings[4]))
#define ui_g_Ungroup_chassis_dis (&(ui_g_now_strings[5]))
#define ui_g_Ungroup_gimbal_dis (&(ui_g_now_strings[6]))
#define ui_g_Ungroup_superCap_text (&(ui_g_now_strings[7]))
#define ui_g_Ungroup_aim_switch (&(ui_g_now_strings[8]))
#define ui_g_Ungroup_chassis_mode (&(ui_g_now_strings[9]))

#ifdef MANUAL_DIRTY
#define ui_g_Ungroup_super_capacity_num_dirty (ui_g_dirty_figure[0])
#define ui_g_Ungroup_shoot_speed_number_dirty (ui_g_dirty_figure[1])
#define ui_g_Ungroup_auto_aim_dirty (ui_g_dirty_figure[2])
#define ui_g_Ungroup_Crosshair_dirty (ui_g_dirty_figure[3])

#define ui_g_Ungroup_friction_dirty (ui_g_dirty_string[0])
#define ui_g_Ungroup_gimbal_mode_dirty (ui_g_dirty_string[1])
#define ui_g_Ungroup_shoot_speed_text_dirty (ui_g_dirty_string[2])
#define ui_g_Ungroup_friction_mode_dirty (ui_g_dirty_string[3])
#define ui_g_Ungroup_aim_mode_dirty (ui_g_dirty_string[4])
#define ui_g_Ungroup_chassis_dis_dirty (ui_g_dirty_string[5])
#define ui_g_Ungroup_gimbal_dis_dirty (ui_g_dirty_string[6])
#define ui_g_Ungroup_superCap_text_dirty (ui_g_dirty_string[7])
#define ui_g_Ungroup_aim_switch_dirty (ui_g_dirty_string[8])
#define ui_g_Ungroup_chassis_mode_dirty (ui_g_dirty_string[9])
#endif

void ui_init_g(void);
void ui_update_g(void);

#endif // UI_g_H
