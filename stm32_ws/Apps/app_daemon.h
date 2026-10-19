/**
 ******************************************************************************
 * @file    app_daemon.h
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   Daemon 应用层守护线程接口。
 *
 * @note    本模块维护统一健康表、在线离线边沿事件和基础诊断快照。
 * @note    本模块只做集中观测，不反向接管 Gimbal、Shoot、Chassis 等安全输出钳位。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_APPS_DAEMON_H
#define PRINTK_APPS_DAEMON_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 状态与诊断类型
 * --------------------------------------------------------------------------
 * App 头文件只暴露快照、诊断和必要的注入类型，让下游按稳定接口读取状态。
 * ========================================================================== */
/**
 * @brief Daemon 健康项。
 *
 * 该枚举固定描述当前母仓库关心的健康对象。具体 target 未启用的项目会在配置阶段关闭，
 * 关闭项目不会参与错误汇总。
 */
typedef enum
{
    AppDaemonRcDbus = 0,      /**< DBUS 遥控输入链路。 */
    AppDaemonReferee,         /**< 裁判系统链路。 */
    AppDaemonCommHost,        /**< Host 485 通讯链路。 */
    AppDaemonCommVision,      /**< Host 自瞄输入链路。 */
    AppDaemonObserveImu,      /**< IMU 设备采样链路。 */
    AppDaemonObserveIns,      /**< INS 姿态观测状态。 */
    AppDaemonGimbalYaw,       /**< 云台 yaw 电机反馈。 */
    AppDaemonGimbalPitch,     /**< 云台 pitch 电机反馈。 */
    AppDaemonShootTrigger,    /**< 发射 trigger 电机反馈。 */
    AppDaemonShootFric,       /**< 发射摩擦轮反馈组。 */
    AppDaemonChassisDrive,    /**< 底盘驱动反馈组。 */
    AppDaemonChassisSteer,    /**< 底盘舵向反馈组。 */
    AppDaemonTfmini,          /**< TFmini 测距设备。 */
    AppDaemonItemCount        /**< 健康项数量。 */
} AppDaemonItem;

/**
 * @brief Daemon 操作状态。
 */
typedef enum
{
    AppDaemonOk = 0,    /**< 操作成功。 */
    AppDaemonNull = 1,  /**< 输入指针为空。 */
    AppDaemonArg = 2,   /**< 输入参数非法。 */
    AppDaemonBusy = 3   /**< 模块状态不满足操作要求。 */
} AppDaemonStatus;

/**
 * @brief Daemon 事件类型。
 */
typedef enum
{
    AppDaemonEventNone = 0,       /**< 无事件。 */
    AppDaemonEventOnline = 1,     /**< 项目稳定恢复在线。 */
    AppDaemonEventOffline = 2,    /**< 项目进入离线错误。 */
    AppDaemonEventDataError = 3,  /**< 项目报告数据错误。 */
    AppDaemonEventRecovering = 4  /**< 项目从离线进入恢复等待。 */
} AppDaemonEventKind;

/**
 * @brief 单项健康状态。
 */
typedef struct
{
    uint32_t new_time;     /**< 最新一次心跳时间戳，单位 ms。 */
    uint32_t last_time;    /**< 上一次心跳时间戳，单位 ms。 */
    uint32_t lost_time;    /**< 最近一次进入离线的时间戳，单位 ms。 */
    uint32_t work_time;    /**< 最近一次从离线恢复的时间戳，单位 ms。 */
    uint32_t offline_ms;   /**< 判定离线的超时时间，单位 ms。 */
    uint32_t online_ms;    /**< 恢复后判定稳定在线的等待时间，单位 ms。 */
    float frequency;       /**< 心跳估算频率，单位 Hz。 */
    uint8_t priority;      /**< 错误优先级，数值越大优先级越高。 */
    uint8_t enable;        /**< 非 0 表示该项参与健康检查。 */
    uint8_t error;         /**< 非 0 表示当前存在错误或恢复等待。 */
    uint8_t lost;          /**< 非 0 表示当前已经离线。 */
    uint8_t data_error;    /**< 非 0 表示最新心跳携带数据错误。 */
} AppDaemonHealth;

/**
 * @brief Daemon 边沿事件。
 */
typedef struct
{
    AppDaemonEventKind kind; /**< 事件类型。 */
    AppDaemonItem item;      /**< 事件所属健康项。 */
    uint32_t tick_ms;        /**< 事件发生时间戳，单位 ms。 */
    uint8_t priority;        /**< 事件发生时的项目优先级。 */
} AppDaemonEvent;

/**
 * @brief Daemon 总状态快照。
 */
typedef struct
{
    AppDaemonHealth item[AppDaemonItemCount]; /**< 健康表。 */
    AppDaemonItem highest_error;              /**< 当前最高优先级错误项，any_error 为 0 时无意义。 */
    uint32_t event_seq;                       /**< 已写入事件序号。 */
    uint8_t any_error;                        /**< 非 0 表示存在至少一个错误项。 */
} AppDaemonState;

/**
 * @brief Daemon 诊断快照。
 */
typedef struct
{
    uint32_t loop_count;        /**< 守护线程周期执行次数。 */
    uint32_t report_count;      /**< 外部主动上报心跳次数。 */
    uint32_t poll_count;        /**< 周期轮询心跳次数。 */
    uint32_t poll_fail_count;   /**< 周期轮询读取快照失败次数。 */
    uint32_t event_write_count; /**< 成功写入事件次数。 */
    uint32_t event_drop_count;  /**< 事件 ring 满导致丢弃次数。 */
    uint32_t last_tick_ms;      /**< 最近一次守护周期时间戳，单位 ms。 */
    uint8_t ready;              /**< 非 0 表示 Daemon 已初始化。 */
} AppDaemonDiag;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 公开接口按初始化、快照读取和外部注入排列；旧 task/global API 不再作为入口。
 * ========================================================================== */
/**
 * @brief 初始化并启动 Daemon 守护线程。
 *
 * @return 成功返回 1，失败返回 0。
 */
uint8_t app_daemon_init(void);

/**
 * @brief 主动上报某个健康项心跳。
 *
 * 后续 ISR 或 App 可调用该接口提交外部心跳；当前第一版仍以 Daemon 周期轮询为主。
 *
 * @param item 健康项。
 * @param tick_ms 心跳时间戳，单位 ms；为 0 时使用当前系统时间。
 * @param data_error 非 0 表示本次心跳数据异常。
 * @return 返回 Daemon 状态码。
 */
AppDaemonStatus app_daemon_report(AppDaemonItem item, uint32_t tick_ms, uint8_t data_error);

/**
 * @brief 判断某个健康项是否处于错误状态。
 *
 * @param item 健康项。
 * @return 错误返回 1，正常或项目未启用返回 0。
 */
uint8_t app_daemon_is_error(AppDaemonItem item);

/**
 * @brief 读取单个健康项快照。
 *
 * @param item 健康项。
 * @param out 健康项快照输出。
 * @return 返回 Daemon 状态码。
 */
AppDaemonStatus app_daemon_get_item(AppDaemonItem item, AppDaemonHealth *out);

/**
 * @brief 读取 Daemon 总状态快照。
 *
 * @param out 总状态快照输出。
 * @return 返回 Daemon 状态码。
 */
AppDaemonStatus app_daemon_get_state(AppDaemonState *out);

/**
 * @brief 读取 Daemon 诊断快照。
 *
 * @param out 诊断快照输出。
 * @return 返回 Daemon 状态码。
 */
AppDaemonStatus app_daemon_get_diag(AppDaemonDiag *out);

/**
 * @brief 读取一条边沿事件。
 *
 * @param out 事件输出。
 * @return 读到事件返回 1，事件为空或参数非法返回 0。
 */
uint8_t app_daemon_read_event(AppDaemonEvent *out);

/**
 * @brief 清空 Daemon 事件 ring。
 */
void app_daemon_clear_events(void);

#ifdef __cplusplus
}
#endif

#endif /* PRINTK_APPS_DAEMON_H */
