/**
 ******************************************************************************
 * @file    sentry_2026_map.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   2026 哨兵硬件 map 与设备默认策略。
 *
 * @note    本文件只描述当前实车接线、BSP 启动策略、设备容量和电机用途映射。
 * @note    控制器 PID、输入灵敏度、自瞄参数等运行时参数不放在这里。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_ROBOT_SENTRY_2026_MAP_H
#define PRINTK_ROBOT_SENTRY_2026_MAP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * BSP 启动策略
 * --------------------------------------------------------------------------
 * 描述 bsp_init() 在当前 target 下默认启动哪些底层设备。
 * ========================================================================== */

#define ROBOT_BSP_INIT_FDCAN                      1U                                 /**< 启动 FDCAN 默认接收。 */
#define ROBOT_BSP_INIT_MOTOR                      1U                                 /**< 启动电机设备绑定。 */
#define ROBOT_BSP_INIT_FLASH                      0U                                 /**< 默认不探测外部 Flash。 */
#define ROBOT_BSP_INIT_IMU                        1U                                 /**< 启动板载 IMU。 */
#define ROBOT_BSP_IMU_CALI                        0U                                 /**< 默认不执行 IMU 在线标定。 */
#define ROBOT_BSP_INIT_TFMINI                     0U                                 /**< 默认不启动 TFmini。 */
#define ROBOT_BSP_INIT_RC                         1U                                 /**< 启动遥控器接收。 */
#define ROBOT_BSP_INIT_BUZZER                     1U                                 /**< 启动蜂鸣器 PWM。 */
#define ROBOT_BSP_INIT_MAGAZINE                   0U                                 /**< 默认不启动弹仓舵机。 */
#define ROBOT_BSP_INIT_CDC                        1U                                 /**< 启动 USB CDC 调试输出。 */
#define ROBOT_BSP_INIT_POWER                      1U                                 /**< 初始化可控电源输出。 */

/* ==========================================================================
 * UART 与通讯映射
 * --------------------------------------------------------------------------
 * 描述当前 target 的串口设备绑定；设备实现只消费 robot_board.h 展开的短宏。
 * ========================================================================== */

#define ROBOT_TARGET_RC_UART_ID                   BSP_UART_ID_UART5                  /**< DBUS 遥控器 UART。 */
#define ROBOT_TARGET_REFEREE_UART_ID              BSP_UART_ID_USART1                 /**< 裁判系统 UART。 */
#define ROBOT_TARGET_COMM_HOST_UART_ID            BSP_UART_ID_USART2                 /**< Host 485 通讯 UART。 */
#define ROBOT_TARGET_COMM_BOARD_UART_ID           BSP_UART_ID_USART3                 /**< 板间通讯 UART，默认未启用。 */
#define ROBOT_TARGET_COMM_BOARD_ENABLE            0U                                 /**< 当前 target 不占用板间通讯 UART。 */
#define ROBOT_TARGET_TFMINI_UART_ID               BSP_UART_ID_USART10                /**< TFmini UART。 */
#define ROBOT_RC_RX_BUF_SIZE                      ((uint16_t)18U)                    /**< 遥控器接收帧长度，单位 byte。 */
#define ROBOT_RC_DMA_BUF_BYTES                    ((uint16_t)32U)                    /**< 遥控器 DMA 缓存长度，单位 byte。 */
#define ROBOT_RC_UART_BAUD                        100000U                            /**< DBUS UART 波特率。 */
#define ROBOT_REFEREE_UART_BAUD                   115200U                            /**< 裁判系统 UART 波特率。 */
#define ROBOT_REFEREE_RX_BUF_SIZE                 ((uint16_t)512U)                   /**< 裁判系统接收缓存长度，单位 byte。 */
#define ROBOT_REFEREE_TX_TIMEOUT_MS               20U                                /**< 裁判系统发送超时，单位 ms。 */
#define ROBOT_TFMINI_USE_TIMEBASE                 1U                                 /**< TFmini 使用 bsp_timebase 时间戳。 */
#define ROBOT_TFMINI_USE_DWT                      0U                                 /**< TFmini 不使用 DWT 时间戳。 */
#define ROBOT_TFMINI_TX_TIMEOUT_MS                100U                               /**< TFmini 发送超时，单位 ms。 */

/* ==========================================================================
 * SPI、OSPI 与片选映射
 * --------------------------------------------------------------------------
 * 描述当前 target 的 SPI/OSPI 设备绑定。
 * ========================================================================== */

#define ROBOT_TARGET_WS2812_SPI_ID                BSP_SPI_ID_SPI6                    /**< WS2812 灯带 SPI。 */
#define ROBOT_TARGET_IMU_SPI_ID                   BSP_SPI_ID_SPI2                    /**< BMI088 IMU SPI。 */
#define ROBOT_TARGET_IMU_ACC_CS_GPIO_ID           BSP_GPIO_ID_ACC_CS                 /**< BMI088 加速度计片选。 */
#define ROBOT_TARGET_IMU_GYRO_CS_GPIO_ID          BSP_GPIO_ID_GYRO_CS                /**< BMI088 陀螺仪片选。 */
#define ROBOT_TARGET_FLASH_OSPI_ID                BSP_OSPI_ID_FLASH                  /**< 外部 Flash OSPI bus。 */
#define ROBOT_TARGET_FLASH_OSPI_FLASH_ID          BSP_OSPI_FLASH_ID_1                /**< 外部 Flash OSPI flash id。 */
#define ROBOT_WS2812_LED_MAX                      ((uint16_t)1U)                     /**< WS2812 最大灯珠数量。 */
#define ROBOT_WS2812_RESET_WORDS                  ((uint16_t)64U)                    /**< WS2812 复位码长度。 */
#define ROBOT_WS2812_TX_TIMEOUT_MS                20U                                /**< WS2812 发送超时，单位 ms。 */
#define ROBOT_IMU_SPI_TIMEOUT_MS                  1U                                 /**< BMI088 SPI 阻塞超时，单位 ms。 */
#define ROBOT_FLASH_DEVICE_BASE                   0x000000UL                         /**< 外部 Flash 器件内部起点。 */
#define ROBOT_FLASH_DEVICE_SIZE                   (8UL * 1024UL * 1024UL)            /**< 外部 Flash 总容量，单位 byte。 */
#define ROBOT_FLASH_DEVICE_PAGE_SIZE              256U                               /**< 外部 Flash 页大小，单位 byte。 */
#define ROBOT_FLASH_DEVICE_ERASE_SIZE             4096U                              /**< 外部 Flash 擦除粒度，单位 byte。 */
#define ROBOT_FLASH_CMD_TIMEOUT_MS                1000UL                             /**< 外部 Flash 普通命令超时，单位 ms。 */

/* ==========================================================================
 * GPIO 与 PWM 映射
 * --------------------------------------------------------------------------
 * 描述当前 target 的电源输出、蜂鸣器、弹仓舵机和 IMU 加热片资源。
 * ========================================================================== */

#define ROBOT_TARGET_POWER_OUT1_GPIO_ID           BSP_GPIO_ID_PC14                   /**< XT30 VCC_OUT1 使能 GPIO。 */
#define ROBOT_TARGET_POWER_OUT2_GPIO_ID           BSP_GPIO_ID_PC13                   /**< XT30 VCC_OUT2 使能 GPIO。 */
#define ROBOT_TARGET_POWER_5V_GPIO_ID             BSP_GPIO_ID_PWR_EN_5V              /**< 对外 5V 使能 GPIO。 */
#define ROBOT_POWER_OUT1_DEFAULT_ON               0U                                 /**< VCC_OUT1 默认关闭。 */
#define ROBOT_POWER_OUT2_DEFAULT_ON               0U                                 /**< VCC_OUT2 默认关闭。 */
#define ROBOT_POWER_5V_DEFAULT_ON                 1U                                 /**< 对外 5V 默认打开。 */
#define ROBOT_POWER_ON_LEVEL                      GPIO_PIN_SET                       /**< 电源打开 GPIO 电平。 */
#define ROBOT_POWER_OFF_LEVEL                     GPIO_PIN_RESET                     /**< 电源关闭 GPIO 电平。 */
#define ROBOT_TARGET_BUZZER_TIM_ID                BSP_TIM_ID_TIM12                   /**< 蜂鸣器 PWM 定时器。 */
#define ROBOT_TARGET_BUZZER_TIM_CHANNEL           TIM_CHANNEL_2                      /**< 蜂鸣器 PWM 通道。 */
#define ROBOT_BUZZER_DUTY_DEFAULT                 ((uint16_t)500U)                   /**< 蜂鸣器默认占空比，千分比。 */
#define ROBOT_TARGET_MAGAZINE_TIM_ID              BSP_TIM_ID_TIM2                    /**< 弹仓舵机 PWM 定时器。 */
#define ROBOT_TARGET_MAGAZINE_TIM_CHANNEL         TIM_CHANNEL_1                      /**< 弹仓舵机 PWM 通道。 */
#define ROBOT_MAGAZINE_COMPARE_CLOSED             500U                               /**< 弹仓关闭 compare。 */
#define ROBOT_MAGAZINE_COMPARE_OPEN               1300U                              /**< 弹仓打开 compare。 */
#define ROBOT_TARGET_IMU_HEAT_TIM_ID              BSP_TIM_ID_TIM3                    /**< IMU 加热 PWM 定时器。 */
#define ROBOT_TARGET_IMU_HEAT_TIM_CHANNEL         TIM_CHANNEL_4                      /**< IMU 加热 PWM 通道。 */
#define ROBOT_TARGET_TIMEBASE_TIM_ID              BSP_TIM_ID_TIM24                   /**< BSP 64 位时间基准定时器。 */

/* ==========================================================================
 * FDCAN 与电机用途映射
 * --------------------------------------------------------------------------
 * 描述当前 target 的 FDCAN 默认启动槽、电机接收槽和用途到总线/ID 的绑定。
 * ========================================================================== */

#define ROBOT_TARGET_FDCAN_DEFAULT0_ID            BSP_FDCAN_ID_FDCAN1                /**< 默认启动槽 0 FDCAN。 */
#define ROBOT_TARGET_FDCAN_DEFAULT0_RX_FIFO       FDCAN_RX_FIFO0                     /**< 默认启动槽 0 RX FIFO。 */
#define ROBOT_TARGET_FDCAN_DEFAULT0_FILTER        0U                                 /**< 默认启动槽 0 过滤器。 */
#define ROBOT_TARGET_FDCAN_DEFAULT1_ID            BSP_FDCAN_ID_FDCAN2                /**< 默认启动槽 1 FDCAN。 */
#define ROBOT_TARGET_FDCAN_DEFAULT1_RX_FIFO       FDCAN_RX_FIFO1                     /**< 默认启动槽 1 RX FIFO。 */
#define ROBOT_TARGET_FDCAN_DEFAULT1_FILTER        0U                                 /**< 默认启动槽 1 过滤器。 */
#define ROBOT_TARGET_FDCAN_DEFAULT2_ID            BSP_FDCAN_ID_FDCAN3                /**< 默认启动槽 2 FDCAN。 */
#define ROBOT_TARGET_FDCAN_DEFAULT2_RX_FIFO       FDCAN_RX_FIFO0                     /**< 默认启动槽 2 RX FIFO。 */
#define ROBOT_TARGET_FDCAN_DEFAULT2_FILTER        0U                                 /**< 默认启动槽 2 过滤器。 */
#define ROBOT_TARGET_MOTOR_BUS0_ID                BSP_FDCAN_ID_FDCAN1                /**< 电机接收槽 0 FDCAN。 */
#define ROBOT_TARGET_MOTOR_BUS0_RX_FIFO           FDCAN_RX_FIFO0                     /**< 电机接收槽 0 RX FIFO。 */
#define ROBOT_TARGET_MOTOR_BUS1_ID                BSP_FDCAN_ID_FDCAN2                /**< 电机接收槽 1 FDCAN。 */
#define ROBOT_TARGET_MOTOR_BUS1_RX_FIFO           FDCAN_RX_FIFO1                     /**< 电机接收槽 1 RX FIFO。 */
#define ROBOT_TARGET_MOTOR_BUS2_ID                BSP_FDCAN_ID_FDCAN3                /**< 电机接收槽 2 FDCAN。 */
#define ROBOT_TARGET_MOTOR_BUS2_RX_FIFO           FDCAN_RX_FIFO0                     /**< 电机接收槽 2 RX FIFO。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_BUS            BSP_FDCAN_ID_FDCAN1                /**< 底盘电机 FDCAN。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_STEER_BUS      ROBOT_TARGET_MOTOR_CHASSIS_BUS     /**< 底盘舵向电机 FDCAN。 */
#define ROBOT_TARGET_MOTOR_FRIC_BUS               BSP_FDCAN_ID_FDCAN1                /**< 摩擦轮电机 FDCAN。 */
#define ROBOT_TARGET_MOTOR_GIMBAL_BUS             BSP_FDCAN_ID_FDCAN2                /**< 云台电机 FDCAN。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_TX_ID          0x200U                             /**< 底盘 DJI 电机发送 ID。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_STEER_TX_ID    0x1FFU                             /**< 底盘舵向 DJI 电机发送 ID。 */
#define ROBOT_TARGET_MOTOR_FRIC_TX_ID             0x200U                             /**< 摩擦轮 DJI 电机发送 ID。 */
#define ROBOT_TARGET_MOTOR_GIMBAL_TX_ID           0x1FFU                             /**< 云台 DJI 电机发送 ID。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_DRIVE_0_ID     0x201U                             /**< 底盘驱动 0 反馈 ID。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_DRIVE_1_ID     0x202U                             /**< 底盘驱动 1 反馈 ID。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_DRIVE_2_ID     0x203U                             /**< 底盘驱动 2 反馈 ID。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_DRIVE_3_ID     0x204U                             /**< 底盘驱动 3 反馈 ID。 */

/*
 * DMH6215 原生速度模式映射。速度命令 ID = 基础 ID + 0x200；
 * 反馈 ID 保持 0x51..0x54。摩擦轮暂未接入，后续接入前必须迁移 ESC_ID
 * 或把摩擦轮移到独立 FDCAN，避免 0x201/0x202 冲突。
 */
#define ROBOT_TARGET_MOTOR_CHASSIS_DM_BIND        1U                                 /**< 启用 DM6215 反馈量程绑定。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_DM_ID_0        1U                                 /**< RR 基础 ID。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_DM_ID_1        2U                                 /**< FL 基础 ID。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_DM_ID_2        3U                                 /**< RL 基础 ID。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_DM_ID_3        4U                                 /**< FR 基础 ID。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_DM_FB_ID_0     0x51U                              /**< RR 反馈 ID。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_DM_FB_ID_1     0x52U                              /**< FL 反馈 ID。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_DM_FB_ID_2     0x53U                              /**< RL 反馈 ID。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_DM_FB_ID_3     0x54U                              /**< FR 反馈 ID。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_STEER_0_ID     0x205U                             /**< 底盘舵向 0 反馈 ID。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_STEER_1_ID     0x206U                             /**< 底盘舵向 1 反馈 ID。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_STEER_2_ID     0x207U                             /**< 底盘舵向 2 反馈 ID。 */
#define ROBOT_TARGET_MOTOR_CHASSIS_STEER_3_ID     0x208U                             /**< 底盘舵向 3 反馈 ID。 */
#define ROBOT_TARGET_MOTOR_FRIC_0_ID              0x201U                             /**< 摩擦轮 0 反馈 ID。 */
#define ROBOT_TARGET_MOTOR_FRIC_1_ID              0x202U                             /**< 摩擦轮 1 反馈 ID。 */
#define ROBOT_TARGET_MOTOR_FRIC_2_ID              0x203U                             /**< 摩擦轮 2 反馈 ID。 */
#define ROBOT_TARGET_MOTOR_FRIC_3_ID              0x204U                             /**< 摩擦轮 3 反馈 ID。 */
#define ROBOT_TARGET_MOTOR_GIMBAL_YAW_ID          0x205U                             /**< 云台 yaw 电机反馈 ID。 */
#define ROBOT_TARGET_MOTOR_GIMBAL_PITCH_ID        0x206U                             /**< 云台 pitch 电机反馈 ID。 */
#define ROBOT_TARGET_MOTOR_TRIGGER_ID             0x207U                             /**< 拨弹电机反馈 ID。 */

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_ROBOT_SENTRY_2026_MAP_H */
