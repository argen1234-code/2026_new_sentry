/**
  * @file       hwt_imu.h
  * @brief      HWT906 类型桩（本工程不含 IMU）
  * @note       只保留底盘代码引用到的字段；getter 返回常量零值。
  */
#ifndef HWT_IMU_H
#define HWT_IMU_H

#include <stdint.h>

#define HWT_AXIS_ROLL   0u
#define HWT_AXIS_PITCH  1u
#define HWT_AXIS_YAW    2u

typedef struct
{
    int16_t raw[3];
    float deg[3];
    float rad[3];
    float yaw_total_deg;
    float yaw_total_rad;
    float last_yaw_rad;
    uint8_t yaw_init_flag;
    uint16_t version;
    uint8_t updated;
} hwt_angle_info_t;

typedef struct
{
    int16_t raw[3];
    float dps[3];
    float radps[3];
    uint16_t voltage_raw;
    float voltage;
    uint8_t updated;
} hwt_gyro_info_t;

typedef struct
{
    int16_t raw[3];
    float g[3];
    float mps2[3];
    int16_t temp_raw;
    float temp_c;
    uint8_t updated;
} hwt_accel_info_t;

typedef struct
{
    hwt_angle_info_t angle;
    hwt_gyro_info_t gyro;
    hwt_accel_info_t accel;
    uint32_t ok_frame_count;
    uint32_t bad_frame_count;
} hwt_imu_info_t;

const hwt_imu_info_t *hwt906_get_info(void);
/* [yaw, pitch, roll]，与上游约定一致 */
const float *hwt906_get_gimbal_angle_point(void);

#endif
