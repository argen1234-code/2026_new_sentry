/**
 ******************************************************************************
 * @file    task_main.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   应用任务启动入口实现。
 *
 * @note    本文件只负责启动 app，不承载控制逻辑、设备解析或状态估计。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "task_main.h"

#include "robot.h"

#if ROBOT_APP_ENABLE_BOARD_TEST != 0U
#include "app_board_test.h"
#endif
#if ROBOT_APP_ENABLE_CAN_TEST != 0U
#include "app_can_test.h"
#endif
#if ROBOT_APP_ENABLE_ALARM != 0U
#include "app_alarm.h"
#endif
#if ROBOT_APP_ENABLE_CHASSIS != 0U
#include "app_chassis.h"
#endif
#if ROBOT_APP_ENABLE_CALIBRATION != 0U
#include "app_calibration.h"
#endif
#if ROBOT_APP_ENABLE_BOARD_LINK != 0U
#include "app_board_link.h"
#endif
#if ROBOT_APP_ENABLE_COMMUNICATION != 0U
#include "app_communication.h"
#endif
#if ROBOT_APP_ENABLE_DAEMON != 0U
#include "app_daemon.h"
#endif
#if ROBOT_APP_ENABLE_GIMBAL != 0U
#include "app_gimbal.h"
#endif
#if ROBOT_APP_ENABLE_INPUT != 0U
#include "app_input.h"
#endif
#if ROBOT_APP_ENABLE_MAGAZINE != 0U
#include "app_magazine.h"
#endif
#if ROBOT_APP_ENABLE_OBSERVE != 0U
#include "app_observe.h"
#endif
#if ROBOT_APP_ENABLE_REFEREE != 0U
#include "app_referee.h"
#endif
#if ROBOT_APP_ENABLE_SHOOT != 0U
#include "app_shoot.h"
#endif

uint8_t task_main_init(void)
{
#if ROBOT_APP_ENABLE_BOARD_TEST != 0U
    if (app_board_test_init() == 0U) {
        return 0U;
    }
#endif

#if ROBOT_APP_ENABLE_CAN_TEST != 0U
    app_can_test_init();
#endif

#if ROBOT_APP_ENABLE_COMMUNICATION != 0U
    if (app_communication_init() == 0U) {
        return 0U;
    }
#endif

#if ROBOT_APP_ENABLE_CALIBRATION != 0U
    if (app_calibration_init() == 0U) {
        return 0U;
    }
#endif

#if ROBOT_APP_ENABLE_OBSERVE != 0U
    if (app_observe_init() == 0U) {
        return 0U;
    }
#endif

#if ROBOT_APP_ENABLE_INPUT != 0U
    if (app_input_init() == 0U) {
        return 0U;
    }
#endif

#if ROBOT_APP_ENABLE_MAGAZINE != 0U
    if (app_magazine_init() == 0U) {
        return 0U;
    }
#endif

#if ROBOT_APP_ENABLE_GIMBAL != 0U
    if (app_gimbal_init() == 0U) {
        return 0U;
    }
#endif

#if (ROBOT_APP_ENABLE_CALIBRATION != 0U) && (ROBOT_APP_ENABLE_GIMBAL != 0U)
    (void)app_calibration_apply_gimbal();
#endif

#if ROBOT_APP_ENABLE_CHASSIS != 0U
    if (app_chassis_init() == 0U) {
        return 0U;
    }
#endif

#if ROBOT_APP_ENABLE_SHOOT != 0U
    if (app_shoot_init() == 0U) {
        return 0U;
    }
#endif

#if ROBOT_APP_ENABLE_REFEREE != 0U
    if (app_referee_init() == 0U) {
        return 0U;
    }
#endif

#if ROBOT_APP_ENABLE_BOARD_LINK != 0U
    if (app_board_link_init() == 0U) {
        return 0U;
    }
#endif

#if ROBOT_APP_ENABLE_DAEMON != 0U
    if (app_daemon_init() == 0U) {
        return 0U;
    }
#endif

#if ROBOT_APP_ENABLE_ALARM != 0U
    if (app_alarm_init() == 0U) {
        return 0U;
    }
#endif

    return 1U;
}
