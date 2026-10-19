#ifndef _BALANCE_H
#define _BALANCE_H

#define PITCH_TURN                    0
#define YAW_TURN                      1



/* ================= ���̲��� ================= */
#define CHASSIS_TASK_INIT_TIME        357
#define CHASSIS_X_CHANNEL             1
#define CHASSIS_Y_CHANNEL             0
#define CHASSIS_WZ_CHANNEL            2
#define CHASSIS_MODE_CHANNEL          0
#define CHASSIS_FOLLOW_CHANNEL        1
#define CHASSIS_VX_RC_SEN             0.006f
#define CHASSIS_VY_RC_SEN             0.005f
#define CHASSIS_ANGLE_Z_RC_SEN        0.000002f
#define CHASSIS_WZ_RC_SEN             0.01f
#define CHASSIS_ACCEL_X_NUM           0.1666666667f
#define CHASSIS_ACCEL_Y_NUM           0.3333333333f
#define CHASSIS_RC_DEADLINE           25
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f
#define CHASSIS_CONTROL_TIME_MS       2
#define CHASSIS_CONTROL_TIME          0.002f
#define CHASSIS_CONTROL_FREQUENCE     500.0f
#define MAX_MOTOR_CAN_CURRENT         16000.0f
#define SWING_KEY                     KEY_PRESSED_OFFSET_CTRL
#define CHASSIS_FRONT_KEY             KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY              KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY              KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY             KEY_PRESSED_OFFSET_D
#define M3508_MOTOR_RPM_TO_VECTOR     0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR
#define MAX_WHEEL_SPEED               4.0f
#define NORMAL_MAX_CHASSIS_SPEED_X    3.0f
#define NORMAL_MAX_CHASSIS_SPEED_Y    3.0f
#define CHASSIS_WZ_SET_SCALE          0.1f
#define SWING_NO_MOVE_ANGLE           0.7f
#define SWING_MOVE_ANGLE              0.31415926535897932384626433832795f

// #define M3505_MOTOR_SPEED_PID_KP      15000.0f
// #define M3505_MOTOR_SPEED_PID_KI      10.0f
// #define M3505_MOTOR_SPEED_PID_KD      0.0f
// #define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
// #define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

// // 6020 ������غ꣨�� F4 ������ֲ��
// #define GM6020_Angle_Ratio            1303.63813886f   // ������ת��Ϊ GM6020 ����ֵ: 8191 / (2 * PI)
// #define MPS_to_RPM                    3234.16461897f   // m/s -> r/min: 60.0f * ������ / ���ܳ�

// // 6020 �ǶȻ� PID
// #define GM6020_MOTOR_ANGLE_PID_KP        100.0f
// #define GM6020_MOTOR_ANGLE_PID_KI        0.01f
// #define GM6020_MOTOR_ANGLE_PID_KD        0.0f
// #define GM6020_MOTOR_ANGLE_PID_MAX_OUT   120.0f
// #define GM6020_MOTOR_ANGLE_PID_MAX_IOUT  10.0f

// // ���ֻ���ģʽ PID
// #define YAW_RETURN_PID_KP                450.0f
// #define YAW_RETURN_PID_KI                0.021f
// #define YAW_RETURN_PID_KD                20.0f
// #define YAW_RETURN_PID_MAX_OUT           600.0f
// #define YAW_RETURN_PID_MAX_IOUT          28.0f

// // 6020 �ٶȻ� PID
// #define GM6020_MOTOR_SPEED_PID_KP        55.0f
// #define GM6020_MOTOR_SPEED_PID_KI        2.4f
// #define GM6020_MOTOR_SPEED_PID_KD        80.0f
// #define GM6020_MOTOR_SPEED_PID_MAX_OUT   8000.0f
// #define GM6020_MOTOR_SPEED_PID_MAX_IOUT  4000.0f

// // ���� 6020 ��ʼ�Ƕȣ���λ��rad��
// #define CHASSIS_6020_INIT_ANGLE_0        ( 1.95f)
// #define CHASSIS_6020_INIT_ANGLE_1        (-1.80f)
// #define CHASSIS_6020_INIT_ANGLE_2        ( 0.90f)
// #define CHASSIS_6020_INIT_ANGLE_3        ( 3.30f)

#define CHASSIS_FOLLOW_GIMBAL_PID_KP  10.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI  0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD  0.1f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

#define CHASSIS_SPIN_SPEED SPIN_SPEED
#define CHASSIS_SPIN_FACTOR SPIN_FACTOR

/* ================= ��̨���� ================= */
#define FEEDFORWARD_GAIN              0.00f
#define PITCH_SPEED_PID_KP            3000.0f
#define PITCH_SPEED_PID_KI            80.0f
#define PITCH_SPEED_PID_KD            0.0f
#define PITCH_SPEED_PID_MAX_OUT       30000.0f
#define PITCH_SPEED_PID_MAX_IOUT      10000.0f

#define YAW_SPEED_PID_KP              3600.0f
#define YAW_SPEED_PID_KI              20.0f
#define YAW_SPEED_PID_KD              0.0f
#define YAW_SPEED_PID_MAX_OUT         30000.0f
#define YAW_SPEED_PID_MAX_IOUT        5000.0f



#define PITCH_GYRO_ABSOLUTE_PID_KP    15.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI    0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD    0.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f
				
#define YAW_GYRO_ABSOLUTE_PID_KP      28.0f
#define YAW_GYRO_ABSOLUTE_PID_KI      0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD      0.5f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

#define PITCH_ENCODE_RELATIVE_PID_KP  80.0f
#define PITCH_ENCODE_RELATIVE_PID_KI  0.0f
#define PITCH_ENCODE_RELATIVE_PID_KD  0.5f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f
			
#define YAW_ENCODE_RELATIVE_PID_KP    28.0f
#define YAW_ENCODE_RELATIVE_PID_KI    0.0f
#define YAW_ENCODE_RELATIVE_PID_KD    0.5f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f

//解决电机颤抖
#define YAW_MOTOR_GYRO_POLARITY					( 1)
#define PITCH_MOTOR_GYRO_POLARITY				( 1)


#define GIMBAL_ANGLE_Z_RC_SEN         0.000002f
#define GIMBAL_TASK_INIT_TIME         200
#define YAW_CHANNEL                   2
#define PITCH_CHANNEL                 3
#define GIMBAL_MODE_CHANNEL           0
#define WZ_CHANNEL                    2
#define TURN_KEYBOARD                 KEY_PRESSED_OFFSET_F
#define TURN_SPEED                    0.04f
#define TEST_KEYBOARD                 KEY_PRESSED_OFFSET_R
#define RC_DEADBAND                   10
#define YAW_RC_SEN                    -0.000005f
#define PITCH_RC_SEN                  -0.000006f
#define YAW_MOUSE_SEN                 0.00005f
#define PITCH_MOUSE_SEN               0.00015f
#define YAW_ENCODE_SEN                0.01f
#define PITCH_ENCODE_SEN              0.01f
#define GIMBAL_CONTROL_TIME           1
#define GIMBAL_TEST_MODE              0
#define HALF_ECD_RANGE                4096
#define ECD_RANGE                     8191
#define GIMBAL_INIT_ANGLE_ERROR       0.1f
#define GIMBAL_INIT_STOP_TIME         100
#define GIMBAL_INIT_TIME              6000
#define GIMBAL_CALI_REDUNDANT_ANGLE   0.1f
#define GIMBAL_INIT_PITCH_SPEED       0.004f
#define GIMBAL_INIT_YAW_SPEED         0.005f
#define GIMBAL_CALI_MOTOR_SET         8000
#define GIMBAL_CALI_STEP_TIME         2000
#define GIMBAL_CALI_GYRO_LIMIT        0.1f
#define GIMBAL_CALI_PITCH_MAX_STEP    1
#define GIMBAL_CALI_PITCH_MIN_STEP    2
#define GIMBAL_CALI_YAW_MAX_STEP      3
#define GIMBAL_CALI_YAW_MIN_STEP      4
#define GIMBAL_CALI_START_STEP        GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP          5
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD              0.000766990394f
#endif

#define ecd_format(ecd)               \
    do {                              \
        if ((ecd) > ECD_RANGE)        \
            (ecd) -= ECD_RANGE;       \
        else if ((ecd) < 0)           \
            (ecd) += ECD_RANGE;       \
    } while (0)

#define gimbal_total_pid_clear(gimbal_clear)                                                                    \
    do {                                                                                                        \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);                    \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);                    \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                                     \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid);                  \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid);                  \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                                   \
    } while (0)

/* ================= ����������� ================= */
#define SHOOT_RC_MODE_CHANNEL         1
#define SHOOT_CONTROL_TIME            GIMBAL_CONTROL_TIME
#define SHOOT_FRIC_PWM_ADD_VALUE      100.0f
#define SHOOT_ON_KEYBOARD             KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD            KEY_PRESSED_OFFSET_E
#define SHOOT_DONE_KEY_OFF_TIME       10
#define PRESS_LONG_TIME               2
#define RC_S_LONG_TIME                2000
#define UP_ADD_TIME                   80
#define HALF_ECD_RANGE                4096
#define ECD_RANGE                     8191
#define MOTOR_RPM_TO_SPEED            0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE            0.000021305288720633905968306772076277f
#define FULL_COUNT                    18
#define TRIGGER_SPEED                 -10.0f
#define CONTINUE_TRIGGER_SPEED        -20.0f
#define READY_TRIGGER_SPEED           -10.0f
#define KEY_OFF_JUGUE_TIME            500
#define SWITCH_TRIGGER_ON             0
#define SWITCH_TRIGGER_OFF            1
#define BLOCK_TRIGGER_SPEED           1.0f
#define BLOCK_TIME                    700
#define REVERSE_TIME                  500
#define REVERSE_SPEED_LIMIT           13.0f
#define PI_FOUR                       0.78539816339744830961566084581988f
#define PI_TEN                        0.31415f
#define TRIGGER_ANGLE_PID_KP          5000.0f
#define TRIGGER_ANGLE_PID_KI          2.0f
#define TRIGGER_ANGLE_PID_KD          0.0f
#define TRIGGER_BULLET_PID_MAX_OUT    10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT   9000.0f
#define TRIGGER_READY_PID_MAX_OUT     10000.0f
#define TRIGGER_READY_PID_MAX_IOUT    7000.0f
#define FRIC_SPEED_PID_KP             15000.0f
#define FRIC_SPEED_PID_KI             10.0f
#define FRIC_SPEED_PID_KD             0.0f
#define FRIC_MAX_OUT                  10000.0f
#define FRIC_MAX_IOUT                 2000.0f
#define FRIC_UP                       10
#define FRIC_DOWN                     0
#define SHOOT_HEAT_REMAIN_VALUE       80

#define SHOOT_LIMIT										230
#define SHOOT_COOL										14
#define DAMN_CALORIES									10

#define POWER_LIMIT         90.0f

#endif
