/**
 ******************************************************************************
 * @file    imu_bmi088_reg.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   BMI088 寄存器映射和位定义。
 *
 * @note    本文件只保存 BMI088 寄存器地址、固定值和位定义，不包含
 *          SPI 传输、初始化流程或板级资源配置。
 * @note    差异: 与旧版 BMI088reg.h 的寄存器值保持一致；新版修正了
 *          SHFITS/CONGIF 等拼写，并修正 BMI088_FATAL_ERROR 的自引用宏。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_DEVICE_BMI088_REG_H
#define PRINTK_DEVICE_BMI088_REG_H

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 设备头文件描述协议事实、配置结构、状态和诊断，具体接口绑定由 Robot 配置展开。
 * ========================================================================== */
#define BMI088_ACC_CHIP_ID 0x00U                                                          /**< BMI088 加速度计 `CHIP_ID` 寄存器地址。 */
#define BMI088_ACC_CHIP_ID_VALUE 0x1EU                                                    /**< BMI088 加速度计 `CHIP_ID` 固定返回值。 */
#define BMI088_ACC_ERR_REG 0x02U                                                          /**< BMI088 加速度计错误状态寄存器地址。 */
#define BMI088_ACCEL_CONFIG_ERROR_SHIFTS 0x2U                                             /**< BMI088 加速度计配置错误状态位偏移。 */
#define BMI088_ACCEL_CONFIG_ERROR (1U << BMI088_ACCEL_CONFIG_ERROR_SHIFTS)                /**< BMI088 加速度计配置错误状态位掩码。 */
#define BMI088_FATAL_ERROR_SHIFTS 0x0U                                                    /**< BMI088 致命错误状态位偏移。 */
#define BMI088_FATAL_ERROR (1U << BMI088_FATAL_ERROR_SHIFTS)                              /**< BMI088 致命错误状态位掩码；修正旧版自引用宏。 */
#define BMI088_ACC_STATUS 0x03U                                                           /**< BMI088 加速度计状态寄存器地址。 */
#define BMI088_ACCEL_DRDY_SHIFTS 0x7U                                                     /**< BMI088 加速度计数据就绪位偏移。 */
#define BMI088_ACCEL_DRDY (1U << BMI088_ACCEL_DRDY_SHIFTS)                                /**< BMI088 加速度计数据就绪位掩码。 */
#define BMI088_ACCEL_XOUT_L 0x12U                                                         /**< BMI088 加速度计 X 轴低字节寄存器地址。 */
#define BMI088_ACCEL_XOUT_M 0x13U                                                         /**< BMI088 加速度计 X 轴高字节寄存器地址。 */
#define BMI088_ACCEL_YOUT_L 0x14U                                                         /**< BMI088 加速度计 Y 轴低字节寄存器地址。 */
#define BMI088_ACCEL_YOUT_M 0x15U                                                         /**< BMI088 加速度计 Y 轴高字节寄存器地址。 */
#define BMI088_ACCEL_ZOUT_L 0x16U                                                         /**< BMI088 加速度计 Z 轴低字节寄存器地址。 */
#define BMI088_ACCEL_ZOUT_M 0x17U                                                         /**< BMI088 加速度计 Z 轴高字节寄存器地址。 */
#define BMI088_SENSORTIME_DATA_L 0x18U                                                    /**< BMI088 传感器时间低字节寄存器地址。 */
#define BMI088_SENSORTIME_DATA_M 0x19U                                                    /**< BMI088 传感器时间中字节寄存器地址。 */
#define BMI088_SENSORTIME_DATA_H 0x1AU                                                    /**< BMI088 传感器时间高字节寄存器地址。 */
#define BMI088_ACC_INT_STAT_1 0x1DU                                                       /**< BMI088 加速度计中断状态 1 寄存器地址。 */
#define BMI088_ACCEL_DRDY_INTERRUPT_SHIFTS 0x7U                                           /**< BMI088 加速度计中断数据就绪位偏移。 */
#define BMI088_ACCEL_DRDY_INTERRUPT (1U << BMI088_ACCEL_DRDY_INTERRUPT_SHIFTS)            /**< BMI088 加速度计中断数据就绪位掩码。 */
#define BMI088_TEMP_M 0x22U                                                               /**< BMI088 温度高位寄存器地址。 */
#define BMI088_TEMP_L 0x23U                                                               /**< BMI088 温度低位寄存器地址。 */
#define BMI088_ACC_CONF 0x40U                                                             /**< BMI088 加速度计配置寄存器地址。 */
#define BMI088_ACC_CONF_MUST_Set 0x80U                                                    /**< BMI088 加速度计配置寄存器必须置位的保留位。 */
#define BMI088_ACC_BWP_SHIFTS 0x4U                                                        /**< BMI088 加速度计带宽配置位偏移。 */
#define BMI088_ACC_OSR4 (0x0U << BMI088_ACC_BWP_SHIFTS)                                   /**< BMI088 加速度计过采样模式 `OSR4`。 */
#define BMI088_ACC_OSR2 (0x1U << BMI088_ACC_BWP_SHIFTS)                                   /**< BMI088 加速度计过采样模式 `OSR2`。 */
#define BMI088_ACC_NORMAL (0x2U << BMI088_ACC_BWP_SHIFTS)                                 /**< BMI088 加速度计正常滤波模式。 */
#define BMI088_ACC_ODR_SHIFTS 0x0U                                                        /**< BMI088 加速度计输出数据率位偏移。 */
#define BMI088_ACC_12_5_HZ (0x5U << BMI088_ACC_ODR_SHIFTS)                                /**< BMI088 加速度计输出数据率 12.5 Hz。 */
#define BMI088_ACC_25_HZ (0x6U << BMI088_ACC_ODR_SHIFTS)                                  /**< BMI088 加速度计输出数据率 25 Hz。 */
#define BMI088_ACC_50_HZ (0x7U << BMI088_ACC_ODR_SHIFTS)                                  /**< BMI088 加速度计输出数据率 50 Hz。 */
#define BMI088_ACC_100_HZ (0x8U << BMI088_ACC_ODR_SHIFTS)                                 /**< BMI088 加速度计输出数据率 100 Hz。 */
#define BMI088_ACC_200_HZ (0x9U << BMI088_ACC_ODR_SHIFTS)                                 /**< BMI088 加速度计输出数据率 200 Hz。 */
#define BMI088_ACC_400_HZ (0xAU << BMI088_ACC_ODR_SHIFTS)                                 /**< BMI088 加速度计输出数据率 400 Hz。 */
#define BMI088_ACC_800_HZ (0xBU << BMI088_ACC_ODR_SHIFTS)                                 /**< BMI088 加速度计输出数据率 800 Hz。 */
#define BMI088_ACC_1600_HZ (0xCU << BMI088_ACC_ODR_SHIFTS)                                /**< BMI088 加速度计输出数据率 1600 Hz。 */
#define BMI088_ACC_RANGE 0x41U                                                            /**< BMI088 加速度计量程配置寄存器地址。 */
#define BMI088_ACC_RANGE_SHIFTS 0x0U                                                      /**< BMI088 加速度计量程配置位偏移。 */
#define BMI088_ACC_RANGE_3G (0x0U << BMI088_ACC_RANGE_SHIFTS)                             /**< BMI088 加速度计量程为 ±3G。 */
#define BMI088_ACC_RANGE_6G (0x1U << BMI088_ACC_RANGE_SHIFTS)                             /**< BMI088 加速度计量程为 ±6G。 */
#define BMI088_ACC_RANGE_12G (0x2U << BMI088_ACC_RANGE_SHIFTS)                            /**< BMI088 加速度计量程为 ±12G。 */
#define BMI088_ACC_RANGE_24G (0x3U << BMI088_ACC_RANGE_SHIFTS)                            /**< BMI088 加速度计量程为 ±24G。 */
#define BMI088_INT1_IO_CTRL 0x53U                                                         /**< BMI088 加速度计 INT1 引脚控制寄存器地址。 */
#define BMI088_ACC_INT1_IO_ENABLE_SHIFTS 0x3U                                             /**< BMI088 加速度计 INT1 输出使能位偏移。 */
#define BMI088_ACC_INT1_IO_ENABLE (0x1U << BMI088_ACC_INT1_IO_ENABLE_SHIFTS)              /**< BMI088 加速度计 INT1 输出使能位。 */
#define BMI088_ACC_INT1_GPIO_MODE_SHIFTS 0x2U                                             /**< BMI088 加速度计 INT1 输出模式位偏移。 */
#define BMI088_ACC_INT1_GPIO_PP (0x0U << BMI088_ACC_INT1_GPIO_MODE_SHIFTS)                /**< BMI088 加速度计 INT1 推挽输出。 */
#define BMI088_ACC_INT1_GPIO_OD (0x1U << BMI088_ACC_INT1_GPIO_MODE_SHIFTS)                /**< BMI088 加速度计 INT1 开漏输出。 */
#define BMI088_ACC_INT1_GPIO_LVL_SHIFTS 0x1U                                              /**< BMI088 加速度计 INT1 电平位偏移。 */
#define BMI088_ACC_INT1_GPIO_LOW (0x0U << BMI088_ACC_INT1_GPIO_LVL_SHIFTS)                /**< BMI088 加速度计 INT1 低电平有效。 */
#define BMI088_ACC_INT1_GPIO_HIGH (0x1U << BMI088_ACC_INT1_GPIO_LVL_SHIFTS)               /**< BMI088 加速度计 INT1 高电平有效。 */
#define BMI088_INT2_IO_CTRL 0x54U                                                         /**< BMI088 加速度计 INT2 引脚控制寄存器地址。 */
#define BMI088_ACC_INT2_IO_ENABLE_SHIFTS 0x3U                                             /**< BMI088 加速度计 INT2 输出使能位偏移。 */
#define BMI088_ACC_INT2_IO_ENABLE (0x1U << BMI088_ACC_INT2_IO_ENABLE_SHIFTS)              /**< BMI088 加速度计 INT2 输出使能位。 */
#define BMI088_ACC_INT2_GPIO_MODE_SHIFTS 0x2U                                             /**< BMI088 加速度计 INT2 输出模式位偏移。 */
#define BMI088_ACC_INT2_GPIO_PP (0x0U << BMI088_ACC_INT2_GPIO_MODE_SHIFTS)                /**< BMI088 加速度计 INT2 推挽输出。 */
#define BMI088_ACC_INT2_GPIO_OD (0x1U << BMI088_ACC_INT2_GPIO_MODE_SHIFTS)                /**< BMI088 加速度计 INT2 开漏输出。 */
#define BMI088_ACC_INT2_GPIO_LVL_SHIFTS 0x1U                                              /**< BMI088 加速度计 INT2 电平位偏移。 */
#define BMI088_ACC_INT2_GPIO_LOW (0x0U << BMI088_ACC_INT2_GPIO_LVL_SHIFTS)                /**< BMI088 加速度计 INT2 低电平有效。 */
#define BMI088_ACC_INT2_GPIO_HIGH (0x1U << BMI088_ACC_INT2_GPIO_LVL_SHIFTS)               /**< BMI088 加速度计 INT2 高电平有效。 */
#define BMI088_INT_MAP_DATA 0x58U                                                         /**< BMI088 加速度计中断映射寄存器地址。 */
#define BMI088_ACC_INT2_DRDY_INTERRUPT_SHIFTS 0x6U                                        /**< BMI088 加速度计 INT2 数据就绪映射位偏移。 */
#define BMI088_ACC_INT2_DRDY_INTERRUPT (0x1U << BMI088_ACC_INT2_DRDY_INTERRUPT_SHIFTS)    /**< BMI088 加速度计数据就绪映射到 INT2。 */
#define BMI088_ACC_INT1_DRDY_INTERRUPT_SHIFTS 0x2U                                        /**< BMI088 加速度计 INT1 数据就绪映射位偏移。 */
#define BMI088_ACC_INT1_DRDY_INTERRUPT (0x1U << BMI088_ACC_INT1_DRDY_INTERRUPT_SHIFTS)    /**< BMI088 加速度计数据就绪映射到 INT1。 */
#define BMI088_ACC_SELF_TEST 0x6DU                                                        /**< BMI088 加速度计自检寄存器地址。 */
#define BMI088_ACC_SELF_TEST_OFF 0x00U                                                    /**< BMI088 加速度计关闭自检。 */
#define BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL 0x0DU                                        /**< BMI088 加速度计正向激励自检。 */
#define BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL 0x09U                                        /**< BMI088 加速度计反向激励自检。 */
#define BMI088_ACC_PWR_CONF 0x7CU                                                         /**< BMI088 加速度计电源模式寄存器地址。 */
#define BMI088_ACC_PWR_SUSPEND_MODE 0x03U                                                 /**< BMI088 加速度计挂起模式。 */
#define BMI088_ACC_PWR_ACTIVE_MODE 0x00U                                                  /**< BMI088 加速度计正常工作模式。 */
#define BMI088_ACC_PWR_CTRL 0x7DU                                                         /**< BMI088 加速度计电源控制寄存器地址。 */
#define BMI088_ACC_ENABLE_ACC_OFF 0x00U                                                   /**< BMI088 加速度计关闭输出。 */
#define BMI088_ACC_ENABLE_ACC_ON 0x04U                                                    /**< BMI088 加速度计开启输出。 */
#define BMI088_ACC_SOFTRESET 0x7EU                                                        /**< BMI088 加速度计软复位寄存器地址。 */
#define BMI088_ACC_SOFTRESET_VALUE 0xB6U                                                  /**< BMI088 加速度计软复位命令值。 */
#define BMI088_GYRO_CHIP_ID 0x00U                                                         /**< BMI088 陀螺仪 `CHIP_ID` 寄存器地址。 */
#define BMI088_GYRO_CHIP_ID_VALUE 0x0FU                                                   /**< BMI088 陀螺仪 `CHIP_ID` 固定返回值。 */
#define BMI088_GYRO_X_L 0x02U                                                             /**< BMI088 陀螺仪 X 轴低字节寄存器地址。 */
#define BMI088_GYRO_X_H 0x03U                                                             /**< BMI088 陀螺仪 X 轴高字节寄存器地址。 */
#define BMI088_GYRO_Y_L 0x04U                                                             /**< BMI088 陀螺仪 Y 轴低字节寄存器地址。 */
#define BMI088_GYRO_Y_H 0x05U                                                             /**< BMI088 陀螺仪 Y 轴高字节寄存器地址。 */
#define BMI088_GYRO_Z_L 0x06U                                                             /**< BMI088 陀螺仪 Z 轴低字节寄存器地址。 */
#define BMI088_GYRO_Z_H 0x07U                                                             /**< BMI088 陀螺仪 Z 轴高字节寄存器地址。 */
#define BMI088_GYRO_INT_STAT_1 0x0AU                                                      /**< BMI088 陀螺仪中断状态寄存器地址。 */
#define BMI088_GYRO_DYDR_SHIFTS 0x7U                                                      /**< BMI088 陀螺仪数据就绪位偏移。 */
#define BMI088_GYRO_DYDR (0x1U << BMI088_GYRO_DYDR_SHIFTS)                                /**< BMI088 陀螺仪数据就绪位掩码。 */
#define BMI088_GYRO_RANGE 0x0FU                                                           /**< BMI088 陀螺仪量程配置寄存器地址。 */
#define BMI088_GYRO_RANGE_SHIFTS 0x0U                                                     /**< BMI088 陀螺仪量程配置位偏移。 */
#define BMI088_GYRO_2000 (0x0U << BMI088_GYRO_RANGE_SHIFTS)                               /**< BMI088 陀螺仪量程为 ±2000 dps。 */
#define BMI088_GYRO_1000 (0x1U << BMI088_GYRO_RANGE_SHIFTS)                               /**< BMI088 陀螺仪量程为 ±1000 dps。 */
#define BMI088_GYRO_500 (0x2U << BMI088_GYRO_RANGE_SHIFTS)                                /**< BMI088 陀螺仪量程为 ±500 dps。 */
#define BMI088_GYRO_250 (0x3U << BMI088_GYRO_RANGE_SHIFTS)                                /**< BMI088 陀螺仪量程为 ±250 dps。 */
#define BMI088_GYRO_125 (0x4U << BMI088_GYRO_RANGE_SHIFTS)                                /**< BMI088 陀螺仪量程为 ±125 dps。 */
#define BMI088_GYRO_BANDWIDTH 0x10U                                                       /**< BMI088 陀螺仪带宽配置寄存器地址。 */
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80U                                              /**< BMI088 陀螺仪带宽寄存器必须置位的保留位。 */
#define BMI088_GYRO_2000_532_HZ 0x00U                                                     /**< BMI088 陀螺仪输出速率/带宽配置为 2000/532 Hz。 */
#define BMI088_GYRO_2000_230_HZ 0x01U                                                     /**< BMI088 陀螺仪输出速率/带宽配置为 2000/230 Hz。 */
#define BMI088_GYRO_1000_116_HZ 0x02U                                                     /**< BMI088 陀螺仪输出速率/带宽配置为 1000/116 Hz。 */
#define BMI088_GYRO_400_47_HZ 0x03U                                                       /**< BMI088 陀螺仪输出速率/带宽配置为 400/47 Hz。 */
#define BMI088_GYRO_200_23_HZ 0x04U                                                       /**< BMI088 陀螺仪输出速率/带宽配置为 200/23 Hz。 */
#define BMI088_GYRO_100_12_HZ 0x05U                                                       /**< BMI088 陀螺仪输出速率/带宽配置为 100/12 Hz。 */
#define BMI088_GYRO_200_64_HZ 0x06U                                                       /**< BMI088 陀螺仪输出速率/带宽配置为 200/64 Hz。 */
#define BMI088_GYRO_100_32_HZ 0x07U                                                       /**< BMI088 陀螺仪输出速率/带宽配置为 100/32 Hz。 */
#define BMI088_GYRO_LPM1 0x11U                                                            /**< BMI088 陀螺仪功耗模式寄存器地址。 */
#define BMI088_GYRO_NORMAL_MODE 0x00U                                                     /**< BMI088 陀螺仪正常工作模式。 */
#define BMI088_GYRO_SUSPEND_MODE 0x80U                                                    /**< BMI088 陀螺仪挂起模式。 */
#define BMI088_GYRO_DEEP_SUSPEND_MODE 0x20U                                               /**< BMI088 陀螺仪深度挂起模式。 */
#define BMI088_GYRO_SOFTRESET 0x14U                                                       /**< BMI088 陀螺仪软复位寄存器地址。 */
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6U                                                 /**< BMI088 陀螺仪软复位命令值。 */
#define BMI088_GYRO_CTRL 0x15U                                                            /**< BMI088 陀螺仪控制寄存器地址。 */
#define BMI088_DRDY_OFF 0x00U                                                             /**< BMI088 陀螺仪关闭数据就绪输出。 */
#define BMI088_DRDY_ON 0x80U                                                              /**< BMI088 陀螺仪开启数据就绪输出。 */
#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16U                                               /**< BMI088 陀螺仪 INT3/INT4 引脚配置寄存器地址。 */
#define BMI088_GYRO_INT4_GPIO_MODE_SHIFTS 0x3U                                            /**< BMI088 陀螺仪 INT4 输出模式位偏移。 */
#define BMI088_GYRO_INT4_GPIO_PP (0x0U << BMI088_GYRO_INT4_GPIO_MODE_SHIFTS)              /**< BMI088 陀螺仪 INT4 推挽输出。 */
#define BMI088_GYRO_INT4_GPIO_OD (0x1U << BMI088_GYRO_INT4_GPIO_MODE_SHIFTS)              /**< BMI088 陀螺仪 INT4 开漏输出。 */
#define BMI088_GYRO_INT4_GPIO_LVL_SHIFTS 0x2U                                             /**< BMI088 陀螺仪 INT4 电平位偏移。 */
#define BMI088_GYRO_INT4_GPIO_LOW (0x0U << BMI088_GYRO_INT4_GPIO_LVL_SHIFTS)              /**< BMI088 陀螺仪 INT4 低电平有效。 */
#define BMI088_GYRO_INT4_GPIO_HIGH (0x1U << BMI088_GYRO_INT4_GPIO_LVL_SHIFTS)             /**< BMI088 陀螺仪 INT4 高电平有效。 */
#define BMI088_GYRO_INT3_GPIO_MODE_SHIFTS 0x1U                                            /**< BMI088 陀螺仪 INT3 输出模式位偏移。 */
#define BMI088_GYRO_INT3_GPIO_PP (0x0U << BMI088_GYRO_INT3_GPIO_MODE_SHIFTS)              /**< BMI088 陀螺仪 INT3 推挽输出。 */
#define BMI088_GYRO_INT3_GPIO_OD (0x1U << BMI088_GYRO_INT3_GPIO_MODE_SHIFTS)              /**< BMI088 陀螺仪 INT3 开漏输出。 */
#define BMI088_GYRO_INT3_GPIO_LVL_SHIFTS 0x0U                                             /**< BMI088 陀螺仪 INT3 电平位偏移。 */
#define BMI088_GYRO_INT3_GPIO_LOW (0x0U << BMI088_GYRO_INT3_GPIO_LVL_SHIFTS)              /**< BMI088 陀螺仪 INT3 低电平有效。 */
#define BMI088_GYRO_INT3_GPIO_HIGH (0x1U << BMI088_GYRO_INT3_GPIO_LVL_SHIFTS)             /**< BMI088 陀螺仪 INT3 高电平有效。 */
#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18U                                                /**< BMI088 陀螺仪 INT3/INT4 映射寄存器地址。 */
#define BMI088_GYRO_DRDY_IO_OFF 0x00U                                                     /**< BMI088 陀螺仪不输出数据就绪中断。 */
#define BMI088_GYRO_DRDY_IO_INT3 0x01U                                                    /**< BMI088 陀螺仪将数据就绪映射到 INT3。 */
#define BMI088_GYRO_DRDY_IO_INT4 0x80U                                                    /**< BMI088 陀螺仪将数据就绪映射到 INT4。 */
#define BMI088_GYRO_DRDY_IO_BOTH (BMI088_GYRO_DRDY_IO_INT3 | BMI088_GYRO_DRDY_IO_INT4)    /**< BMI088 陀螺仪将数据就绪同时映射到 INT3/INT4。 */
#define BMI088_GYRO_SELF_TEST 0x3CU                                                       /**< BMI088 陀螺仪自检寄存器地址。 */
#define BMI088_GYRO_RATE_OK_SHIFTS 0x4U                                                   /**< BMI088 陀螺仪输出速率有效位偏移。 */
#define BMI088_GYRO_RATE_OK (0x1U << BMI088_GYRO_RATE_OK_SHIFTS)                          /**< BMI088 陀螺仪输出速率有效位。 */
#define BMI088_GYRO_BIST_FAIL_SHIFTS 0x2U                                                 /**< BMI088 陀螺仪内建自检失败位偏移。 */
#define BMI088_GYRO_BIST_FAIL (0x1U << BMI088_GYRO_BIST_FAIL_SHIFTS)                      /**< BMI088 陀螺仪内建自检失败位。 */
#define BMI088_GYRO_BIST_RDY_SHIFTS 0x1U                                                  /**< BMI088 陀螺仪内建自检完成位偏移。 */
#define BMI088_GYRO_BIST_RDY (0x1U << BMI088_GYRO_BIST_RDY_SHIFTS)                        /**< BMI088 陀螺仪内建自检完成位。 */
#define BMI088_GYRO_TRIG_BIST_SHIFTS 0x0U                                                 /**< BMI088 陀螺仪触发内建自检位偏移。 */
#define BMI088_GYRO_TRIG_BIST (0x1U << BMI088_GYRO_TRIG_BIST_SHIFTS)                      /**< BMI088 陀螺仪触发内建自检位。 */

#endif // PRINTK_DEVICE_BMI088_REG_H
