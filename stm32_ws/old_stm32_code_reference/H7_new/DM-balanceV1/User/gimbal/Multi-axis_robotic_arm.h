#ifndef MULTI_AXIS_ROBOTIC_ARM_H
#define MULTI_AXIS_ROBOTIC_ARM_H
#include "struct_typedef.h"
#include "robot_param.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#if ROBOT_GIMBAL == multi_axis_robotic_arm
#include "can_bsp.h"
#include "pid.h"
#include "remote_control.h"

#include "INS_task.h"
//#ifndef ARM_JOINT_NUM
//#define ARM_JOINT_NUM MOTOR_NUM
//#endif
#ifndef ARM_JOINT_NUM
#define ARM_JOINT_NUM 6
#endif
// 电机校准使用
#ifndef ARM_KP_DEFAULT
#define ARM_KP_DEFAULT (20.0f)
#endif
#ifndef ARM_KD_DEFAULT
#define ARM_KD_DEFAULT (1.5f)
#endif

//电机PID参数
//注意：在电机设置的POS不为0时，KD也应不为0.否则会导致电机震荡

#define J0_MAX_ANGLE (0.0f)
#define J1_MAX_ANGLE (5.5f)
#define J2_MAX_ANGLE (0.0f)
#define J3_MAX_ANGLE (0.0f)
#define J4_MAX_ANGLE (0.0f)
#define J5_MAX_ANGLE (0.0f)

#define J0_MIN_ANLE (0.9f)
#define J1_MIN_ANLE (2.9f)
#define J2_MIN_ANLE (0.9f)
#define J3_MIN_ANLE (0.9f)
#define J4_MIN_ANLE (0.9f)
#define J5_MIN_ANLE (0.9f)

#ifndef J0_KP
#define J0_KP (0.6f)
#endif

#ifndef J1_KP
#define J1_KP (2.0f)
#endif

#ifndef J2_KP
#define J2_KP (5.0f)
#endif

#ifndef J3_KP
#define J3_KP (0.6f)
#endif

#ifndef J4_KP
#define J4_KP (0.6f)
#endif

#ifndef J5_KP
#define J5_KP (0.6f)
#endif

#ifndef J0_KD
#define J0_KD (0.6f)
#endif

#ifndef J1_KD
#define J1_KD (0.3f)
#endif

#ifndef J2_KD
#define J2_KD (0.3f)
#endif

#ifndef J3_KD
#define J3_KD (0.6f)
#endif

#ifndef J4_KD
#define J4_KD (0.6f)
#endif

#ifndef J5_KD
#define J5_KD (0.6f)
#endif
//超时时间
#define TIMEOUT 100
//机械臂电机初始化速度
#define INIT_VEL 2.0f
//机械臂电机片选控制通道
#define ARM_RC_MODE_CHANNEL         1
//机械臂电机控制通道
#define HORI_CHANNEL                2
#define VERT_CHANNEL                3
typedef enum
{
  ARM_NONE,
  ARM_HOLD,
  ARM_RC,
  ARM_SELF,
  ARM_INIT
}arm_joint_e;

extern void gimbal_init(gimbal_control_t *init);		
extern void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change);
extern void gimbal_set_mode(gimbal_control_t *set_mode);
extern void gimbal_feedback_update(gimbal_control_t *feedback_update);
extern void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change);
extern void gimbal_set_control(gimbal_control_t *set_control);
extern void gimbal_control_loop(gimbal_control_t *control_loop);
extern void gimbal_send_cmd(gimbal_control_t *control_send);

/**
  * @brief          锟斤拷台校准锟斤拷锟姐，锟斤拷校准锟斤拷录锟斤拷锟斤拷值,锟斤拷锟?锟斤拷小值锟斤拷锟斤拷
  * @param[out]     yaw 锟斤拷值 指锟斤拷
  * @param[out]     pitch 锟斤拷值 指锟斤拷
  * @param[out]     yaw 锟斤拷锟斤拷锟皆角讹拷 指锟斤拷
  * @param[out]     yaw 锟斤拷小锟斤拷越嵌锟?指锟斤拷
  * @param[out]     pitch 锟斤拷锟斤拷锟皆角讹拷 指锟斤拷
  * @param[out]     pitch 锟斤拷小锟斤拷越嵌锟?指锟斤拷
  * @retval         锟斤拷锟斤拷1 锟斤拷锟斤拷锟缴癸拷校准锟斤拷希锟?锟斤拷锟斤拷0 锟斤拷锟斤拷未校准锟斤拷
  * @waring         锟斤拷锟斤拷锟斤拷锟绞癸拷玫锟絞imbal_control 锟斤拷态锟斤拷锟斤拷锟斤拷锟铰猴拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷通锟斤拷指锟诫复锟斤拷
  */
extern bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);

/**
  * @brief          gimbal cali data, set motor offset encode, max and min relative angle
  * @param[in]      yaw_offse:yaw middle place encode
  * @param[in]      pitch_offset:pitch place encode
  * @param[in]      max_yaw:yaw max relative angle
  * @param[in]      min_yaw:yaw min relative angle
  * @param[in]      max_yaw:pitch max relative angle
  * @param[in]      min_yaw:pitch min relative angle
  * @retval         none
  */
/**
  * @brief          锟斤拷台校准锟斤拷锟矫ｏ拷锟斤拷校准锟斤拷锟斤拷台锟斤拷值锟皆硷拷锟斤拷小锟斤拷锟斤拷械锟斤拷越嵌锟?
  * @param[in]      yaw_offse:yaw 锟斤拷值
  * @param[in]      pitch_offset:pitch 锟斤拷值
  * @param[in]      max_yaw:max_yaw:yaw 锟斤拷锟斤拷锟皆角讹拷
  * @param[in]      min_yaw:yaw 锟斤拷小锟斤拷越嵌锟?
  * @param[in]      max_yaw:pitch 锟斤拷锟斤拷锟皆角讹拷
  * @param[in]      min_yaw:pitch 锟斤拷小锟斤拷越嵌锟?
  * @retval         锟斤拷锟截匡拷
  * @waring         锟斤拷锟斤拷锟斤拷锟绞癸拷玫锟絞imbal_control 锟斤拷态锟斤拷锟斤拷锟斤拷锟铰猴拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷通锟斤拷指锟诫复锟斤拷
  */
extern void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch);
#endif


#endif
