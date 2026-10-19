/**
 ******************************************************************************
 * @file    robot_board_map.h
 * @author  GPT
 * @version V1.1.0
 * @date    2026-05-20
 * @brief   当前 target 的板级硬件映射展开层。
 *
 * @note    具体车辆事实由 robot.h 选择的 Robot target 文件提供。
 * @note    本文件只把 target 宏展开成 Bsp/Devices 使用的板级短宏。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_CONFIG_ROBOT_BOARD_MAP_H
#define PRINTK_CONFIG_ROBOT_BOARD_MAP_H

#include "bsp.h"
#include "robot.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 板级目标
 * --------------------------------------------------------------------------
 * 当前工程只服务 DM02 STM32H723；具体车辆 target 仍由 robot.h 决定。
 * ========================================================================== */

#define BOARDS_DM02_STM32_H723                    1U                                 /**< 当前板级目标名称。 */

/* ==========================================================================
 * UART 设备映射
 * --------------------------------------------------------------------------
 * 把 target 中的串口接线事实展开为设备层使用的板级映射宏。
 * ========================================================================== */

#define ROBOT_BOARD_RC_UART_ID                    ROBOT_TARGET_RC_UART_ID            /**< 遥控器接收 UART。 */
#define ROBOT_BOARD_REFEREE_UART_ID               ROBOT_TARGET_REFEREE_UART_ID       /**< 裁判系统 UART。 */
#define ROBOT_BOARD_COMM_HOST_UART_ID             ROBOT_TARGET_COMM_HOST_UART_ID     /**< Host 485 通讯 UART。 */
#define ROBOT_BOARD_COMM_BOARD_UART_ID            ROBOT_TARGET_COMM_BOARD_UART_ID    /**< 板间通讯 UART。 */
#define ROBOT_BOARD_COMM_BOARD_ENABLE             ROBOT_TARGET_COMM_BOARD_ENABLE     /**< 板间通讯默认启用状态。 */
#define ROBOT_BOARD_TFMINI_UART_ID                ROBOT_TARGET_TFMINI_UART_ID        /**< TFmini 测距模块 UART。 */

/* ==========================================================================
 * SPI 与片选映射
 * --------------------------------------------------------------------------
 * 把 target 中的 SPI/片选事实展开为设备层使用的板级映射宏。
 * ========================================================================== */

#define ROBOT_BOARD_WS2812_SPI_ID                 ROBOT_TARGET_WS2812_SPI_ID         /**< WS2812 灯带 SPI。 */
#define ROBOT_BOARD_IMU_SPI_ID                    ROBOT_TARGET_IMU_SPI_ID            /**< 板载 BMI088 IMU SPI。 */
#define ROBOT_BOARD_IMU_ACC_CS_GPIO_ID            ROBOT_TARGET_IMU_ACC_CS_GPIO_ID    /**< BMI088 加速度计片选 GPIO。 */
#define ROBOT_BOARD_IMU_GYRO_CS_GPIO_ID           ROBOT_TARGET_IMU_GYRO_CS_GPIO_ID   /**< BMI088 陀螺仪片选 GPIO。 */

/* ==========================================================================
 * OSPI Flash 映射
 * --------------------------------------------------------------------------
 * 把 target 中的外部 Flash 接线事实展开为设备层使用的板级映射宏。
 * ========================================================================== */

#define ROBOT_BOARD_FLASH_OSPI_ID                 ROBOT_TARGET_FLASH_OSPI_ID         /**< 板载外部 Flash OSPI bus。 */
#define ROBOT_BOARD_FLASH_OSPI_FLASH_ID           ROBOT_TARGET_FLASH_OSPI_FLASH_ID   /**< 板载外部 Flash OSPI flash id。 */

/* ==========================================================================
 * 可控电源 GPIO 映射
 * --------------------------------------------------------------------------
 * 对应原理图中三路电源输出使能脚：XT30 OUT1、XT30 OUT2 和对外 5V。
 * ========================================================================== */

#define ROBOT_BOARD_POWER_OUT1_GPIO_ID            ROBOT_TARGET_POWER_OUT1_GPIO_ID    /**< XT30 VCC_OUT1 使能 GPIO。 */
#define ROBOT_BOARD_POWER_OUT2_GPIO_ID            ROBOT_TARGET_POWER_OUT2_GPIO_ID    /**< XT30 VCC_OUT2 使能 GPIO。 */
#define ROBOT_BOARD_POWER_5V_GPIO_ID              ROBOT_TARGET_POWER_5V_GPIO_ID      /**< 对外 5V 使能 GPIO。 */

/* ==========================================================================
 * PWM 设备映射
 * --------------------------------------------------------------------------
 * 统一展开蜂鸣器、弹仓舵机和 IMU 加热片使用的 TIM/channel 资源。
 * ========================================================================== */

#define ROBOT_BOARD_BUZZER_TIM_ID                 ROBOT_TARGET_BUZZER_TIM_ID         /**< 蜂鸣器 PWM 定时器。 */
#define ROBOT_BOARD_BUZZER_TIM_CHANNEL            ROBOT_TARGET_BUZZER_TIM_CHANNEL    /**< 蜂鸣器 PWM 通道。 */
#define ROBOT_BOARD_MAGAZINE_TIM_ID               ROBOT_TARGET_MAGAZINE_TIM_ID       /**< 弹仓舵机 PWM 定时器。 */
#define ROBOT_BOARD_MAGAZINE_TIM_CHANNEL          ROBOT_TARGET_MAGAZINE_TIM_CHANNEL  /**< 弹仓舵机 PWM 通道。 */
#define ROBOT_BOARD_IMU_HEAT_TIM_ID               ROBOT_TARGET_IMU_HEAT_TIM_ID       /**< IMU 加热片 PWM 定时器。 */
#define ROBOT_BOARD_IMU_HEAT_TIM_CHANNEL          ROBOT_TARGET_IMU_HEAT_TIM_CHANNEL  /**< IMU 加热片 PWM 通道。 */

/* ==========================================================================
 * BSP 时间基准映射
 * --------------------------------------------------------------------------
 * 选择 bsp_timebase 使用的 1 MHz 基准定时器。
 * ========================================================================== */

#define ROBOT_BOARD_TIMEBASE_TIM_ID               ROBOT_TARGET_TIMEBASE_TIM_ID       /**< BSP 64 位时间基准定时器。 */

/* ==========================================================================
 * FDCAN 默认启动映射
 * --------------------------------------------------------------------------
 * 描述 bsp_fdcan_start_default() 启动哪些 FDCAN，以及每路使用的 RX FIFO/过滤器索引。
 * ========================================================================== */

#define ROBOT_BOARD_FDCAN_DEFAULT0_ID             ROBOT_TARGET_FDCAN_DEFAULT0_ID     /**< 默认启动槽 0 使用的 FDCAN 总线。 */
#define ROBOT_BOARD_FDCAN_DEFAULT0_RX_FIFO        ROBOT_TARGET_FDCAN_DEFAULT0_RX_FIFO /**< 默认启动槽 0 使用的 RX FIFO。 */
#define ROBOT_BOARD_FDCAN_DEFAULT0_FILTER         ROBOT_TARGET_FDCAN_DEFAULT0_FILTER /**< 默认启动槽 0 使用的过滤器索引。 */
#define ROBOT_BOARD_FDCAN_DEFAULT1_ID             ROBOT_TARGET_FDCAN_DEFAULT1_ID     /**< 默认启动槽 1 使用的 FDCAN 总线。 */
#define ROBOT_BOARD_FDCAN_DEFAULT1_RX_FIFO        ROBOT_TARGET_FDCAN_DEFAULT1_RX_FIFO /**< 默认启动槽 1 使用的 RX FIFO。 */
#define ROBOT_BOARD_FDCAN_DEFAULT1_FILTER         ROBOT_TARGET_FDCAN_DEFAULT1_FILTER /**< 默认启动槽 1 使用的过滤器索引。 */
#define ROBOT_BOARD_FDCAN_DEFAULT2_ID             ROBOT_TARGET_FDCAN_DEFAULT2_ID     /**< 默认启动槽 2 使用的 FDCAN 总线。 */
#define ROBOT_BOARD_FDCAN_DEFAULT2_RX_FIFO        ROBOT_TARGET_FDCAN_DEFAULT2_RX_FIFO /**< 默认启动槽 2 使用的 RX FIFO。 */
#define ROBOT_BOARD_FDCAN_DEFAULT2_FILTER         ROBOT_TARGET_FDCAN_DEFAULT2_FILTER /**< 默认启动槽 2 使用的过滤器索引。 */

/* ==========================================================================
 * 电机总线接收映射
 * --------------------------------------------------------------------------
 * 描述电机设备层需要订阅的 FDCAN 总线和 RX FIFO，用于初始化反馈接收。
 * ========================================================================== */

#define ROBOT_BOARD_MOTOR_BUS0_ID                 ROBOT_TARGET_MOTOR_BUS0_ID         /**< 电机接收槽 0 使用的 FDCAN 总线。 */
#define ROBOT_BOARD_MOTOR_BUS0_RX_FIFO            ROBOT_TARGET_MOTOR_BUS0_RX_FIFO    /**< 电机接收槽 0 使用的 FDCAN RX FIFO。 */
#define ROBOT_BOARD_MOTOR_BUS1_ID                 ROBOT_TARGET_MOTOR_BUS1_ID         /**< 电机接收槽 1 使用的 FDCAN 总线。 */
#define ROBOT_BOARD_MOTOR_BUS1_RX_FIFO            ROBOT_TARGET_MOTOR_BUS1_RX_FIFO    /**< 电机接收槽 1 使用的 FDCAN RX FIFO。 */
#define ROBOT_BOARD_MOTOR_BUS2_ID                 ROBOT_TARGET_MOTOR_BUS2_ID         /**< 电机接收槽 2 使用的 FDCAN 总线。 */
#define ROBOT_BOARD_MOTOR_BUS2_RX_FIFO            ROBOT_TARGET_MOTOR_BUS2_RX_FIFO    /**< 电机接收槽 2 使用的 FDCAN RX FIFO。 */

/* ==========================================================================
 * 电机用途映射
 * --------------------------------------------------------------------------
 * 把机器人用途映射到 FDCAN 总线、发送 ID 和反馈 ID；协议细节仍由电机协议层负责。
 * ========================================================================== */

#define ROBOT_BOARD_MOTOR_CHASSIS_BUS             ROBOT_TARGET_MOTOR_CHASSIS_BUS     /**< 底盘 DJI 电机 FDCAN 总线。 */
#define ROBOT_BOARD_MOTOR_CHASSIS_STEER_BUS       ROBOT_TARGET_MOTOR_CHASSIS_STEER_BUS /**< 底盘舵向 DJI 电机 FDCAN 总线。 */
#define ROBOT_BOARD_MOTOR_FRIC_BUS                ROBOT_TARGET_MOTOR_FRIC_BUS        /**< 摩擦轮 DJI 电机 FDCAN 总线。 */
#define ROBOT_BOARD_MOTOR_GIMBAL_BUS              ROBOT_TARGET_MOTOR_GIMBAL_BUS      /**< 云台 DJI 电机 FDCAN 总线。 */
#define ROBOT_BOARD_MOTOR_CHASSIS_TX_ID           ROBOT_TARGET_MOTOR_CHASSIS_TX_ID   /**< 底盘 DJI 电机发送 ID。 */
#define ROBOT_BOARD_MOTOR_CHASSIS_STEER_TX_ID     ROBOT_TARGET_MOTOR_CHASSIS_STEER_TX_ID /**< 底盘舵向 DJI 电机发送 ID。 */
#define ROBOT_BOARD_MOTOR_FRIC_TX_ID              ROBOT_TARGET_MOTOR_FRIC_TX_ID      /**< 摩擦轮 DJI 电机发送 ID。 */
#define ROBOT_BOARD_MOTOR_GIMBAL_TX_ID            ROBOT_TARGET_MOTOR_GIMBAL_TX_ID    /**< 云台 DJI 电机发送 ID。 */
#define ROBOT_BOARD_MOTOR_CHASSIS_DRIVE_0_ID      ROBOT_TARGET_MOTOR_CHASSIS_DRIVE_0_ID /**< 底盘驱动 0 反馈 ID。 */
#define ROBOT_BOARD_MOTOR_CHASSIS_DRIVE_1_ID      ROBOT_TARGET_MOTOR_CHASSIS_DRIVE_1_ID /**< 底盘驱动 1 反馈 ID。 */
#define ROBOT_BOARD_MOTOR_CHASSIS_DRIVE_2_ID      ROBOT_TARGET_MOTOR_CHASSIS_DRIVE_2_ID /**< 底盘驱动 2 反馈 ID。 */
#define ROBOT_BOARD_MOTOR_CHASSIS_DRIVE_3_ID      ROBOT_TARGET_MOTOR_CHASSIS_DRIVE_3_ID /**< 底盘驱动 3 反馈 ID。 */
#define ROBOT_BOARD_MOTOR_CHASSIS_STEER_0_ID      ROBOT_TARGET_MOTOR_CHASSIS_STEER_0_ID /**< 底盘舵向 0 反馈 ID。 */
#define ROBOT_BOARD_MOTOR_CHASSIS_STEER_1_ID      ROBOT_TARGET_MOTOR_CHASSIS_STEER_1_ID /**< 底盘舵向 1 反馈 ID。 */
#define ROBOT_BOARD_MOTOR_CHASSIS_STEER_2_ID      ROBOT_TARGET_MOTOR_CHASSIS_STEER_2_ID /**< 底盘舵向 2 反馈 ID。 */
#define ROBOT_BOARD_MOTOR_CHASSIS_STEER_3_ID      ROBOT_TARGET_MOTOR_CHASSIS_STEER_3_ID /**< 底盘舵向 3 反馈 ID。 */
#define ROBOT_BOARD_MOTOR_FRIC_0_ID               ROBOT_TARGET_MOTOR_FRIC_0_ID       /**< 摩擦轮 0 反馈 ID。 */
#define ROBOT_BOARD_MOTOR_FRIC_1_ID               ROBOT_TARGET_MOTOR_FRIC_1_ID       /**< 摩擦轮 1 反馈 ID。 */
#define ROBOT_BOARD_MOTOR_FRIC_2_ID               ROBOT_TARGET_MOTOR_FRIC_2_ID       /**< 摩擦轮 2 反馈 ID。 */
#define ROBOT_BOARD_MOTOR_FRIC_3_ID               ROBOT_TARGET_MOTOR_FRIC_3_ID       /**< 摩擦轮 3 反馈 ID。 */
#define ROBOT_BOARD_MOTOR_GIMBAL_YAW_ID           ROBOT_TARGET_MOTOR_GIMBAL_YAW_ID   /**< 云台 yaw 电机反馈 ID。 */
#define ROBOT_BOARD_MOTOR_GIMBAL_PITCH_ID         ROBOT_TARGET_MOTOR_GIMBAL_PITCH_ID /**< 云台 pitch 电机反馈 ID。 */
#define ROBOT_BOARD_MOTOR_TRIGGER_ID              ROBOT_TARGET_MOTOR_TRIGGER_ID      /**< 拨弹电机反馈 ID。 */

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_CONFIG_ROBOT_BOARD_MAP_H */
