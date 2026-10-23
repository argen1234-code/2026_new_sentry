/**
  * @file       host_cmd.h
  * @brief      上位机下发的底盘速度指令
  * @details    legacy_bridge 解析旧协议 UnifiedCmd 后写入该结构，行为层桩
  *             直接读取。单位与 ROS /cmd_vel 一致：m/s、rad/s。
  *
  *             坐标系严格按主机侧（ROS REP-103）：x 向前、y 向左、z 向上，
  *             wz 逆时针为正。上游底盘运动学的 vx/vy 与此同向，但 wz
  *             正方向相反；换算只在两处使用 HOST_CHASSIS_WZ_SIGN
  *             （stubs.c 写 setpoint、legacy_bridge.c 写 0x81 回传），
  *             不要在这里改符号。
  */
#ifndef HOST_CMD_H
#define HOST_CMD_H

#include <stdint.h>

/* 主机与底盘运动学之间只剩 wz 一个符号。
 *
 * 2026-10-23：chas_inv_cal 的平移项已按实车标定改成“相对上游内部系顺时针
 * 90°”，vx/vy 不再需要其它修正；wz 的自转方向实测与 /cmd_vel 一致，故取 +1。
 *   rear = -vx + r*wz, right = +vy - r*wz, front = -vx - r*wz, left = +vy + r*wz
 *
 * 该系数对两个方向都适用：主机 wz -> 内部 setpoint 与
 * 内部反馈 -> 主机 0x81 都乘它。若以后换底盘后自转又反了，只改这一个值。 */
#define HOST_CHASSIS_WZ_SIGN (1.0f)

typedef struct
{
    float vx_m_s;
    float vy_m_s;
    float wz_rad_s;
    uint8_t valid;
} host_chassis_cmd_t;

#endif /* HOST_CMD_H */
