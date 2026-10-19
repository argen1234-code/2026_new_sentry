#ifndef _OMNI_CHASSIS_H
#define _OMNI_CHASSIS_H
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "robot_param.h"
#if ROBOT_CHASSIS == Omni_wheel  

#define MOTOR_DISTANCE_TO_CENTER      0.22f

#define OMNI_WHEEL_NUM   4          // 全向轮数量
#define OMNI_RADIUS      0.0603f    // 全向轮半径

extern void chassis_init(chassis_move_t *chassis_move_init);
extern void chassis_set_mode(chassis_move_t *chassis_move_mode);
extern void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
extern void chassis_feedback_update(chassis_move_t *chassis_move_update);
extern void chassis_set_contorl(chassis_move_t *chassis_move_control);
extern void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
extern void chassis_send_cmd(chassis_move_t *chassis_send_cmd);
extern void observer(chassis_move_t *chassis_move, KalmanFilter_t *vxEstimateKF, KalmanFilter_t *vyEstimateKF);

#endif

#endif
