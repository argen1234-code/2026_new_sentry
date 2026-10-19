/**
 ******************************************************************************
 * @file    robot_board.h
 * @author  GPT
 * @version V1.1.0
 * @date    2026-05-20
 * @brief   当前 target 的 BSP 与设备默认配置展开层。
 *
 * @note    Bsp/Devices 只 include 本文件，不直接 include 具体 Robot target。
 * @note    本文件把 robot_board_map.h 和 target 宏展开成底层使用的短配置宏。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_CONFIG_ROBOT_BOARD_H
#define PRINTK_CONFIG_ROBOT_BOARD_H

#include <stdint.h>

#include "robot_board_map.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * BSP 启动策略
 * --------------------------------------------------------------------------
 * 控制 bsp_init() 是否启动对应板载设备或基础外设能力，不在工程宏里散落覆盖。
 * ========================================================================== */

#define BSP_INIT_FDCAN                            ROBOT_BSP_INIT_FDCAN              /**< 启动默认 FDCAN 过滤器和接收通知。 */
#define BSP_INIT_MOTOR                            ROBOT_BSP_INIT_MOTOR              /**< 绑定板端电机 FDCAN 接收回调。 */
#define BSP_INIT_FLASH                            ROBOT_BSP_INIT_FLASH              /**< 探测并初始化板载外部 Flash。 */
#define BSP_INIT_IMU                              ROBOT_BSP_INIT_IMU                /**< 初始化板载 IMU。 */
#define BSP_IMU_CALI                              ROBOT_BSP_IMU_CALI                /**< 板载 IMU 初始化时是否执行在线标定。 */
#define BSP_INIT_TFMINI                           ROBOT_BSP_INIT_TFMINI             /**< 初始化 TFmini 接收。 */
#define BSP_INIT_RC                               ROBOT_BSP_INIT_RC                 /**< 初始化板载遥控器接收。 */
#define BSP_INIT_BUZZER                           ROBOT_BSP_INIT_BUZZER             /**< 初始化蜂鸣器 PWM。 */
#define BSP_INIT_MAGAZINE                         ROBOT_BSP_INIT_MAGAZINE           /**< 初始化弹仓舵机 PWM。 */
#define BSP_INIT_CDC                              ROBOT_BSP_INIT_CDC                /**< 启动 USB CDC 调试输出。 */
#define BSP_INIT_POWER                            ROBOT_BSP_INIT_POWER              /**< 初始化板载可控电源输出。 */

/* ==========================================================================
 * 遥控器设备默认配置
 * --------------------------------------------------------------------------
 * 描述遥控器设备接收缓存、DMA 对齐缓存、串口和波特率默认值。
 * ========================================================================== */

#define RC_RX_BUF_SIZE                            ROBOT_RC_RX_BUF_SIZE              /**< 遥控器 DMA 接收缓存长度，单位 byte。 */
#define RC_DMA_BUF_BYTES                          ROBOT_RC_DMA_BUF_BYTES            /**< 遥控器 DMA 缓存行对齐长度，单位 byte。 */
#define RC_UART_BAUD                              ROBOT_RC_UART_BAUD                /**< DBUS 遥控器 UART 波特率。 */
#define RC_UART_ID                                ROBOT_BOARD_RC_UART_ID            /**< 遥控器接收 UART。 */

/* ==========================================================================
 * 裁判系统设备默认配置
 * --------------------------------------------------------------------------
 * 描述裁判系统串口、波特率、接收缓存和发送超时。
 * ========================================================================== */

#define REFEREE_UART_ID                           ROBOT_BOARD_REFEREE_UART_ID       /**< 裁判系统 UART。 */
#define REFEREE_UART_BAUD                         ROBOT_REFEREE_UART_BAUD           /**< 裁判系统 UART 波特率。 */
#define REFEREE_RX_BUF_SIZE                       ROBOT_REFEREE_RX_BUF_SIZE         /**< 裁判系统 DMA idle 接收缓存长度，单位 byte。 */
#define REFEREE_TX_TIMEOUT_MS                     ROBOT_REFEREE_TX_TIMEOUT_MS       /**< 裁判系统发送超时时间，单位 ms。 */

/* ==========================================================================
 * 通讯与测距设备默认配置
 * --------------------------------------------------------------------------
 * 描述 Host 485 通讯和 TFmini 测距设备使用的串口与时间戳来源。
 * ========================================================================== */

#define COMM_HOST_UART_ID                         ROBOT_BOARD_COMM_HOST_UART_ID     /**< Host 485 通讯 UART。 */
#define COMM_BOARD_UART_ID                        ROBOT_BOARD_COMM_BOARD_UART_ID    /**< 板间通讯 UART。 */
#define COMM_BOARD_ENABLE_DEFAULT                 ROBOT_BOARD_COMM_BOARD_ENABLE     /**< 板间通讯默认启用状态。 */
#define COMM_HOST_RX_SIZE                         ROBOT_COMM_HOST_RX_BUF_SIZE       /**< Host 485 接收缓存长度。 */
#define COMM_BOARD_RX_SIZE                        ROBOT_COMM_BOARD_RX_BUF_SIZE      /**< 板间通讯接收缓存长度。 */
#define COMM_VOFA_TX_SIZE                         ROBOT_COMM_VOFA_TX_BUF_SIZE       /**< VOFA 发送缓存长度。 */
#define TFMINI_UART_ID                            ROBOT_BOARD_TFMINI_UART_ID        /**< TFmini UART。 */
#define TFMINI_USE_TIMEBASE                       ROBOT_TFMINI_USE_TIMEBASE         /**< 非 0 表示 TFmini 使用 bsp_timebase 时间戳。 */
#define TFMINI_USE_DWT                            ROBOT_TFMINI_USE_DWT              /**< 非 0 表示 TFmini 使用 DWT 时间戳。 */
#define TFMINI_UART_TX_TIMEOUT_MS                 ROBOT_TFMINI_TX_TIMEOUT_MS        /**< TFmini UART 发送默认超时时间，单位 ms。 */

/* ==========================================================================
 * BSP 时间基准默认配置
 * --------------------------------------------------------------------------
 * 描述 bsp_timebase 使用的定时器资源；CubeMX 需要保证该定时器计数频率为 1 MHz。
 * ========================================================================== */

#define BSP_TIMEBASE_TIM_ID                       ROBOT_BOARD_TIMEBASE_TIM_ID       /**< BSP 64 位时间基准定时器。 */

/* ==========================================================================
 * FDCAN 默认启动配置
 * --------------------------------------------------------------------------
 * 把板级 FDCAN 默认启动表转成 BSP 使用的短宏。
 * ========================================================================== */

#define BSP_FDCAN_DEFAULT0_ID                     ROBOT_BOARD_FDCAN_DEFAULT0_ID     /**< 默认启动槽 0 使用的 FDCAN 总线。 */
#define BSP_FDCAN_DEFAULT0_RX_FIFO                ROBOT_BOARD_FDCAN_DEFAULT0_RX_FIFO /**< 默认启动槽 0 使用的 RX FIFO。 */
#define BSP_FDCAN_DEFAULT0_FILTER                 ROBOT_BOARD_FDCAN_DEFAULT0_FILTER /**< 默认启动槽 0 使用的过滤器索引。 */
#define BSP_FDCAN_DEFAULT1_ID                     ROBOT_BOARD_FDCAN_DEFAULT1_ID     /**< 默认启动槽 1 使用的 FDCAN 总线。 */
#define BSP_FDCAN_DEFAULT1_RX_FIFO                ROBOT_BOARD_FDCAN_DEFAULT1_RX_FIFO /**< 默认启动槽 1 使用的 RX FIFO。 */
#define BSP_FDCAN_DEFAULT1_FILTER                 ROBOT_BOARD_FDCAN_DEFAULT1_FILTER /**< 默认启动槽 1 使用的过滤器索引。 */
#define BSP_FDCAN_DEFAULT2_ID                     ROBOT_BOARD_FDCAN_DEFAULT2_ID     /**< 默认启动槽 2 使用的 FDCAN 总线。 */
#define BSP_FDCAN_DEFAULT2_RX_FIFO                ROBOT_BOARD_FDCAN_DEFAULT2_RX_FIFO /**< 默认启动槽 2 使用的 RX FIFO。 */
#define BSP_FDCAN_DEFAULT2_FILTER                 ROBOT_BOARD_FDCAN_DEFAULT2_FILTER /**< 默认启动槽 2 使用的过滤器索引。 */

/* ==========================================================================
 * WS2812 设备默认配置
 * --------------------------------------------------------------------------
 * 描述 WS2812 使用的 SPI、最大灯珠数量、复位码长度和发送超时。
 * ========================================================================== */

#define WS2812_SPI_ID                             ROBOT_BOARD_WS2812_SPI_ID         /**< WS2812 使用的 SPI。 */
#define WS2812_LED_MAX                            ROBOT_WS2812_LED_MAX              /**< WS2812 最大灯珠数量。 */
#define WS2812_RESET_WORDS                        ROBOT_WS2812_RESET_WORDS          /**< WS2812 复位低电平对应的 SPI 4-bit 帧数量。 */
#define WS2812_TX_TIMEOUT_MS                      ROBOT_WS2812_TX_TIMEOUT_MS        /**< WS2812 发送超时时间，单位 ms。 */

/* ==========================================================================
 * IMU 设备默认配置
 * --------------------------------------------------------------------------
 * 描述板载 BMI088 的 SPI、片选 GPIO 和阻塞传输超时。
 * ========================================================================== */

#define IMU_SPI_ID                                ROBOT_BOARD_IMU_SPI_ID            /**< 板载 IMU 使用的 SPI。 */
#define IMU_ACC_CS_GPIO_ID                        ROBOT_BOARD_IMU_ACC_CS_GPIO_ID    /**< 板载 IMU 加速度计片选 GPIO。 */
#define IMU_GYRO_CS_GPIO_ID                       ROBOT_BOARD_IMU_GYRO_CS_GPIO_ID   /**< 板载 IMU 陀螺仪片选 GPIO。 */
#define IMU_SPI_TIMEOUT_MS                        ROBOT_IMU_SPI_TIMEOUT_MS          /**< BMI088 SPI 阻塞传输超时时间，单位 ms。 */

/* ==========================================================================
 * 外部 Flash 设备默认配置
 * --------------------------------------------------------------------------
 * 描述板载外部 Flash 的 OSPI 绑定、器件几何信息和普通命令超时。
 * ========================================================================== */

#define FLASH_OSPI_ID                             ROBOT_BOARD_FLASH_OSPI_ID         /**< 板载外部 Flash OSPI bus。 */
#define FLASH_OSPI_FLASH_ID                       ROBOT_BOARD_FLASH_OSPI_FLASH_ID   /**< 板载外部 Flash OSPI flash id。 */
#define FLASH_DEVICE_BASE                         ROBOT_FLASH_DEVICE_BASE           /**< 外部 Flash 器件内部地址起点。 */
#define FLASH_DEVICE_SIZE                         ROBOT_FLASH_DEVICE_SIZE           /**< 外部 Flash 总容量，单位 byte。 */
#define FLASH_DEVICE_PAGE_SIZE                    ROBOT_FLASH_DEVICE_PAGE_SIZE      /**< 外部 Flash 页大小，单位 byte。 */
#define FLASH_DEVICE_ERASE_SIZE                   ROBOT_FLASH_DEVICE_ERASE_SIZE     /**< 外部 Flash 最小擦除粒度，单位 byte。 */
#define FLASH_DEVICE_END                          (FLASH_DEVICE_BASE + FLASH_DEVICE_SIZE) /**< 外部 Flash 器件内部结束地址。 */
#define FLASH_CMD_TIMEOUT_MS                      ROBOT_FLASH_CMD_TIMEOUT_MS        /**< 外部 Flash 普通命令默认超时时间，单位 ms。 */

/* ==========================================================================
 * 可控电源设备默认配置
 * --------------------------------------------------------------------------
 * 描述三路电源输出 GPIO、初始化默认状态和有效电平。
 * ========================================================================== */

#define POWER_OUT1_GPIO_ID                        ROBOT_BOARD_POWER_OUT1_GPIO_ID    /**< VCC_OUT1 使能 GPIO。 */
#define POWER_OUT2_GPIO_ID                        ROBOT_BOARD_POWER_OUT2_GPIO_ID    /**< VCC_OUT2 使能 GPIO。 */
#define POWER_5V_GPIO_ID                          ROBOT_BOARD_POWER_5V_GPIO_ID      /**< 对外 5V 使能 GPIO。 */
#define POWER_OUT1_DEFAULT_ON                     ROBOT_POWER_OUT1_DEFAULT_ON       /**< VCC_OUT1 初始化默认使能状态。 */
#define POWER_OUT2_DEFAULT_ON                     ROBOT_POWER_OUT2_DEFAULT_ON       /**< VCC_OUT2 初始化默认使能状态。 */
#define POWER_5V_DEFAULT_ON                       ROBOT_POWER_5V_DEFAULT_ON         /**< 对外 5V 初始化默认使能状态。 */
#define POWER_ON_LEVEL                            ROBOT_POWER_ON_LEVEL              /**< 电源打开对应 GPIO 电平。 */
#define POWER_OFF_LEVEL                           ROBOT_POWER_OFF_LEVEL             /**< 电源关闭对应 GPIO 电平。 */

/* ==========================================================================
 * PWM 设备默认配置
 * --------------------------------------------------------------------------
 * 描述蜂鸣器、弹仓舵机和 IMU 加热片的 PWM 资源与默认 compare 参数。
 * ========================================================================== */

#define BUZZER_TIM_ID                             ROBOT_BOARD_BUZZER_TIM_ID         /**< 蜂鸣器 PWM 定时器。 */
#define BUZZER_TIM_CHANNEL                        ROBOT_BOARD_BUZZER_TIM_CHANNEL    /**< 蜂鸣器 PWM 通道。 */
#define BUZZER_DUTY_DEFAULT                       ROBOT_BUZZER_DUTY_DEFAULT         /**< 蜂鸣器默认占空比，单位为千分比。 */
#define MAGAZINE_TIM_ID                           ROBOT_BOARD_MAGAZINE_TIM_ID       /**< 弹仓舵机 PWM 定时器。 */
#define MAGAZINE_TIM_CHANNEL                      ROBOT_BOARD_MAGAZINE_TIM_CHANNEL  /**< 弹仓舵机 PWM 通道。 */
#define MAGAZINE_COMPARE_CLOSED                   ROBOT_MAGAZINE_COMPARE_CLOSED     /**< 弹仓关闭位置对应的 PWM compare 值。 */
#define MAGAZINE_COMPARE_OPEN                     ROBOT_MAGAZINE_COMPARE_OPEN       /**< 弹仓打开位置对应的 PWM compare 值。 */
#define IMU_HEAT_TIM_ID                           ROBOT_BOARD_IMU_HEAT_TIM_ID       /**< IMU 加热 PWM 定时器。 */
#define IMU_HEAT_CHANNEL                          ROBOT_BOARD_IMU_HEAT_TIM_CHANNEL  /**< IMU 加热 PWM 通道。 */

/* ==========================================================================
 * 电机设备默认配置
 * --------------------------------------------------------------------------
 * 把板级电机总线映射转成设备层使用的短宏，避免业务代码直接依赖板级宏名称。
 * ========================================================================== */

#define MOTOR_BUS0_ID                             ROBOT_BOARD_MOTOR_BUS0_ID         /**< 电机接收槽 0 使用的 FDCAN 总线。 */
#define MOTOR_BUS0_RX_FIFO                        ROBOT_BOARD_MOTOR_BUS0_RX_FIFO    /**< 电机接收槽 0 使用的 FDCAN RX FIFO。 */
#define MOTOR_BUS1_ID                             ROBOT_BOARD_MOTOR_BUS1_ID         /**< 电机接收槽 1 使用的 FDCAN 总线。 */
#define MOTOR_BUS1_RX_FIFO                        ROBOT_BOARD_MOTOR_BUS1_RX_FIFO    /**< 电机接收槽 1 使用的 FDCAN RX FIFO。 */
#define MOTOR_BUS2_ID                             ROBOT_BOARD_MOTOR_BUS2_ID         /**< 电机接收槽 2 使用的 FDCAN 总线。 */
#define MOTOR_BUS2_RX_FIFO                        ROBOT_BOARD_MOTOR_BUS2_RX_FIFO    /**< 电机接收槽 2 使用的 FDCAN RX FIFO。 */
#define MOTOR_CHASSIS_BUS                         ROBOT_BOARD_MOTOR_CHASSIS_BUS     /**< 底盘 DJI 电机 FDCAN 总线。 */
#define MOTOR_CHASSIS_STEER_BUS                   ROBOT_BOARD_MOTOR_CHASSIS_STEER_BUS /**< 底盘舵向 DJI 电机 FDCAN 总线。 */
#define MOTOR_FRIC_BUS                            ROBOT_BOARD_MOTOR_FRIC_BUS        /**< 摩擦轮 DJI 电机 FDCAN 总线。 */
#define MOTOR_GIMBAL_BUS                          ROBOT_BOARD_MOTOR_GIMBAL_BUS      /**< 云台 DJI 电机 FDCAN 总线。 */
#define MOTOR_CHASSIS_TX_ID                       ROBOT_BOARD_MOTOR_CHASSIS_TX_ID   /**< 底盘 DJI 电机发送 ID。 */
#define MOTOR_CHASSIS_STEER_TX_ID                 ROBOT_BOARD_MOTOR_CHASSIS_STEER_TX_ID /**< 底盘舵向 DJI 电机发送 ID。 */
#define MOTOR_FRIC_TX_ID                          ROBOT_BOARD_MOTOR_FRIC_TX_ID      /**< 摩擦轮 DJI 电机发送 ID。 */
#define MOTOR_GIMBAL_TX_ID                        ROBOT_BOARD_MOTOR_GIMBAL_TX_ID    /**< 云台 DJI 电机发送 ID。 */
#define MOTOR_CHASSIS_DRIVE_0_ID                  ROBOT_BOARD_MOTOR_CHASSIS_DRIVE_0_ID /**< 底盘驱动 0 反馈 ID。 */
#define MOTOR_CHASSIS_DRIVE_1_ID                  ROBOT_BOARD_MOTOR_CHASSIS_DRIVE_1_ID /**< 底盘驱动 1 反馈 ID。 */
#define MOTOR_CHASSIS_DRIVE_2_ID                  ROBOT_BOARD_MOTOR_CHASSIS_DRIVE_2_ID /**< 底盘驱动 2 反馈 ID。 */
#define MOTOR_CHASSIS_DRIVE_3_ID                  ROBOT_BOARD_MOTOR_CHASSIS_DRIVE_3_ID /**< 底盘驱动 3 反馈 ID。 */
#define MOTOR_CHASSIS_STEER_0_ID                  ROBOT_BOARD_MOTOR_CHASSIS_STEER_0_ID /**< 底盘舵向 0 反馈 ID。 */
#define MOTOR_CHASSIS_STEER_1_ID                  ROBOT_BOARD_MOTOR_CHASSIS_STEER_1_ID /**< 底盘舵向 1 反馈 ID。 */
#define MOTOR_CHASSIS_STEER_2_ID                  ROBOT_BOARD_MOTOR_CHASSIS_STEER_2_ID /**< 底盘舵向 2 反馈 ID。 */
#define MOTOR_CHASSIS_STEER_3_ID                  ROBOT_BOARD_MOTOR_CHASSIS_STEER_3_ID /**< 底盘舵向 3 反馈 ID。 */
#define MOTOR_FRIC_0_ID                           ROBOT_BOARD_MOTOR_FRIC_0_ID       /**< 摩擦轮 0 反馈 ID。 */
#define MOTOR_FRIC_1_ID                           ROBOT_BOARD_MOTOR_FRIC_1_ID       /**< 摩擦轮 1 反馈 ID。 */
#define MOTOR_FRIC_2_ID                           ROBOT_BOARD_MOTOR_FRIC_2_ID       /**< 摩擦轮 2 反馈 ID。 */
#define MOTOR_FRIC_3_ID                           ROBOT_BOARD_MOTOR_FRIC_3_ID       /**< 摩擦轮 3 反馈 ID。 */
#define MOTOR_GIMBAL_YAW_ID                       ROBOT_BOARD_MOTOR_GIMBAL_YAW_ID   /**< 云台 yaw 电机反馈 ID。 */
#define MOTOR_GIMBAL_PITCH_ID                     ROBOT_BOARD_MOTOR_GIMBAL_PITCH_ID /**< 云台 pitch 电机反馈 ID。 */
#define MOTOR_TRIGGER_ID                          ROBOT_BOARD_MOTOR_TRIGGER_ID      /**< 拨弹电机反馈 ID。 */

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_CONFIG_ROBOT_BOARD_H */
