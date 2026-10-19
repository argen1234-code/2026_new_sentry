/**
 ******************************************************************************
 * @file    shoot_mono.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-18
 * @brief   单组摩擦轮发射构型接口。
 *
 * @note    本构型对应旧 shoot_3508，只保留机构控制，不读取任务、CAN 或裁判系统全局状态。
 * @note    所有运行时命令、反馈、热量和时间戳均通过 ShootInput 注入。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_MECHANISM_SHOOT_MONO_H
#define PRINTK_MECHANISM_SHOOT_MONO_H

#include "shoot.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * 组件头文件定义公共语言、构型宏和上下文结构，避免把 App 调度或设备绑定写入组件层。
 * ========================================================================== */
/** @brief 单组摩擦轮数量。 */
#define SHOOT_MONO_FRIC_NUM 2U

/**
 * @brief 单组发射 PID 运行状态。
 */
typedef struct
{
    ShootPidCfg cfg;     /**< PID 参数副本。 */
    float32_t set;       /**< 当前目标值。 */
    float32_t get;       /**< 当前反馈值。 */
    float32_t err;       /**< 当前误差。 */
    float32_t last_err;  /**< 上一次误差。 */
    float32_t iout;      /**< 积分输出。 */
    float32_t out;       /**< PID 输出。 */
} ShootMonoPid;

/**
 * @brief 单个摩擦轮运行状态。
 */
typedef struct
{
    float32_t speed;     /**< 当前摩擦轮速度。 */
    float32_t speed_set; /**< 当前摩擦轮目标速度。 */
    int16_t current;     /**< 当前摩擦轮输出电流。 */
    uint8_t online;      /**< 非 0 表示摩擦轮反馈在线。 */
} ShootMonoFric;

/**
 * @brief 单组摩擦轮发射私有上下文。
 */
typedef struct
{
    ShootMode mode;                              /**< 当前发射模式。 */
    ShootMonoPid trigger_pid;                    /**< 拨弹电机速度 PID。 */
    ShootMonoPid fric_pid[SHOOT_MONO_FRIC_NUM];  /**< 摩擦轮速度 PID。 */
    ShootMonoFric fric[SHOOT_MONO_FRIC_NUM];     /**< 摩擦轮状态。 */
    float32_t trigger_speed;                     /**< 拨弹电机速度反馈。 */
    float32_t trigger_speed_set;                 /**< 拨弹电机目标速度。 */
    float32_t trigger_angle;                     /**< 拨弹电机连续角度。 */
    float32_t trigger_target;                    /**< 拨弹目标角度。 */
    int32_t trigger_count;                       /**< 拨弹编码器跨圈计数。 */
    uint16_t last_ecd;                           /**< 上一次拨弹编码器值。 */
    uint8_t trigger_online;                      /**< 非 0 表示拨弹反馈在线。 */
    uint8_t ecd_ready;                           /**< 非 0 表示编码器基准已建立。 */
    uint8_t move_flag;                           /**< 非 0 表示正在执行一发拨弹。 */
    uint8_t last_fire_once;                      /**< 上一次单发命令状态，用于上升沿检测。 */
    uint16_t block_time;                         /**< 堵转累计周期数。 */
    uint16_t reverse_time;                       /**< 反转累计周期数。 */
    float32_t heat;                              /**< 本地累计热量。 */
    float32_t heat_limit;                        /**< 当前热量上限。 */
    uint32_t last_tick;                          /**< 上一次时间戳。 */
    int16_t trigger_current;                     /**< 拨弹电机输出电流。 */
    int16_t fric_current[SHOOT_MONO_FRIC_NUM];   /**< 摩擦轮输出电流。 */
    uint8_t ready;                               /**< 非 0 表示构型已初始化。 */
} ShootMonoCtx;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按 init/update/control/output 或初始化/查询/发送的顺序排列，便于从数据流阅读。
 * ========================================================================== */

/**
 * @brief 初始化单组摩擦轮发射构型。
 *
 * @param shoot 发射公共上下文。
 * @param impl ShootMonoCtx 私有上下文。
 * @return 执行状态。
 */
ShootStatus shoot_mono_init(Shoot *shoot, void *impl);

/**
 * @brief 执行一次单组摩擦轮发射周期。
 *
 * @param shoot 发射公共上下文。
 * @param impl ShootMonoCtx 私有上下文。
 * @param input 周期输入。
 * @param output 周期输出。
 * @return 执行状态。
 */
ShootStatus shoot_mono_step(Shoot *shoot, void *impl, const ShootInput *input, ShootOutput *output);

/**
 * @brief 更新单组摩擦轮发射反馈。
 *
 * @param shoot 发射公共上下文。
 * @param impl ShootMonoCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
ShootStatus shoot_mono_update(Shoot *shoot, void *impl, const ShootInput *input);

/**
 * @brief 执行单组摩擦轮发射控制计算。
 *
 * @param shoot 发射公共上下文。
 * @param impl ShootMonoCtx 私有上下文。
 * @param input 周期输入。
 * @return 执行状态。
 */
ShootStatus shoot_mono_control(Shoot *shoot, void *impl, const ShootInput *input);

/**
 * @brief 生成单组摩擦轮发射输出。
 *
 * @param shoot 发射公共上下文。
 * @param impl ShootMonoCtx 私有上下文。
 * @param output 周期输出。
 * @return 执行状态。
 */
ShootStatus shoot_mono_output(const Shoot *shoot, void *impl, ShootOutput *output);

/**
 * @brief 同步单组摩擦轮发射目标。
 *
 * @param shoot 发射公共上下文。
 * @param impl ShootMonoCtx 私有上下文。
 * @return 执行状态。
 */
ShootStatus shoot_mono_sync(Shoot *shoot, void *impl);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_MECHANISM_SHOOT_MONO_H
