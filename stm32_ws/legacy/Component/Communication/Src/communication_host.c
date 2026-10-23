/**
 ******************************************************************************
 * @file    communication_host.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-20
 * @brief   485 host 实时通讯实现。
 *
 * @note    本文件只绑定 USART2 DMA idle、Proto2 RT parser 和最新输入缓存。
 * @note    不读取任务层、不读取 HAL tick、不计算 RTT、不接入控制逻辑。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "communication_host.h"

#include "main.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
/** @brief Cortex-M7 D-Cache 行长度，单位 byte。 */
#define COMM_HOST_CACHE_LINE 32U

/** @brief TX DMA 缓冲区按 cache line 对齐后的存储长度。 */
#define COMM_HOST_TX_SIZE (((PROTO2_RT_MAX_FRAME_BYTES + COMM_HOST_CACHE_LINE - 1U) / COMM_HOST_CACHE_LINE) * COMM_HOST_CACHE_LINE)

/** @brief RX DMA 缓冲区按 cache line 对齐后的存储长度。 */
#define COMM_HOST_RX_STORE_SIZE (((COMM_HOST_RX_SIZE + COMM_HOST_CACHE_LINE - 1U) / COMM_HOST_CACHE_LINE) * COMM_HOST_CACHE_LINE)

/**
 * @brief host 通讯内部上下文。
 */
typedef struct
{
    CommunicationHostCfg cfg;          /**< 配置副本。 */
    UART_HandleTypeDef *huart;         /**< 当前使用的 UART 句柄。 */
    proto2_rt_t rt;                    /**< Proto2 RT parser 上下文。 */
    CommunicationAimState aim;         /**< 最新原始自瞄误差缓存。 */
    CommunicationProbe probe;          /**< 最新 probe 缓存。 */
    CommunicationProbeAck ack;         /**< 最新 probe ack 缓存。 */
    CommunicationHostDiag diag;        /**< 诊断缓存。 */
    uint16_t tx_seq;                   /**< 本端发送帧序号。 */
    CommunicationStatus frame_status;  /**< 当前 feed 期间 frame 回调产生的状态。 */
    uint8_t ready;                     /**< 初始化完成标志。 */
} CommunicationHost;

/** @brief host 485 接收 DMA 缓冲区。 */
static uint8_t host_rx_buf[COMM_HOST_RX_STORE_SIZE] __ALIGNED(32);

/** @brief host 485 发送 DMA 缓冲区。 */
static uint8_t host_tx_buf[COMM_HOST_TX_SIZE] __ALIGNED(32);

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * 组件状态由调用方或本文件私有上下文保存，避免运行时分配和跨层全局读写。
 * ========================================================================== */
/** @brief host 通讯单例。 */
static CommunicationHost host;

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具只处理参数、限幅、wrap、矩阵工作区等实现细节，不读取任务或设备全局。
 * ========================================================================== */

/**
 * @brief 进入短临界区。
 *
 * @return 进入前的 PRIMASK。
 */
static uint32_t irq_enter(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/**
 * @brief 退出短临界区。
 *
 * @param primask 进入前保存的 PRIMASK。
 */
static void irq_exit(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 将 Proto2 RT 状态映射为通讯组件状态。
 *
 * @param status Proto2 RT 状态。
 * @return 通讯组件状态。
 */
static CommunicationStatus rt_status(proto2_rt_status_t status)
{
    switch (status) {
    case PROTO2_RT_STATUS_OK:
        return CommunicationOk;
    case PROTO2_RT_STATUS_NULL:
        return CommunicationNull;
    case PROTO2_RT_STATUS_SMALL:
        return CommunicationSmall;
    case PROTO2_RT_STATUS_LENGTH:
        return CommunicationLength;
    case PROTO2_RT_STATUS_ID:
        return CommunicationId;
    case PROTO2_RT_STATUS_CRC:
        return CommunicationCrc;
    case PROTO2_RT_STATUS_DIRECTION:
        return CommunicationDirection;
    case PROTO2_RT_STATUS_DECODE:
        return CommunicationDecode;
    default:
        return CommunicationIo;
    }
}

/**
 * @brief 同步 Proto2 RT parser 统计到诊断缓存。
 */
static void sync_rt_diag(void)
{
    host.diag.rx_frames = host.rt.rx_frames;
    host.diag.crc_errors = host.rt.crc_errors;
    host.diag.id_errors = host.rt.id_errors;
    host.diag.seq_errors = host.rt.seq_errors;
}

/**
 * @brief 写入最近一次组件状态。
 *
 * @param status 组件状态。
 * @return 透传 status。
 */
static CommunicationStatus set_status(CommunicationStatus status)
{
    host.diag.last_status = (uint8_t)status;
    return status;
}

/* ==========================================================================
 * 输入、状态更新与控制步骤
 * --------------------------------------------------------------------------
 * 这一组把注入的输入转换成组件输出；具体调度和硬件发送仍由 App 层负责。
 * ========================================================================== */

/**
 * @brief 使 CPU 读取 DMA 最新写入的 RX 缓冲区。
 */
static void invalidate_rx_buf(void)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)host_rx_buf, (int32_t)sizeof(host_rx_buf));
}

/**
 * @brief 在 DMA TX 前清理 D-Cache，确保外设读取最新帧。
 */
static void clean_tx_buf(void)
{
    SCB_CleanDCache_by_Addr((uint32_t *)host_tx_buf, (int32_t)sizeof(host_tx_buf));
}

/**
 * @brief 判断 UART DMA idle 接收是否仍处于运行状态。
 *
 * @return 运行中返回 1，否则返回 0。
 */
static uint8_t rx_active(void)
{
    if ((host.huart == NULL) || (host.huart->hdmarx == NULL)) {
        return 0U;
    }
    if (host.huart->RxState != HAL_UART_STATE_BUSY_RX) {
        return 0U;
    }
    if (host.huart->hdmarx->State == HAL_DMA_STATE_READY) {
        return 0U;
    }
    return 1U;
}

/**
 * @brief 启动 USART2 DMA idle 接收。
 *
 * @return HAL 状态。
 */
static HAL_StatusTypeDef start_rx(void)
{
    bsp_uart_rx_t rx;
    HAL_StatusTypeDef status;

    if (host.huart == NULL) {
        return HAL_ERROR;
    }

    if (rx_active() != 0U) {
        host.diag.last_restart_status = (uint8_t)HAL_OK;
        return HAL_OK;
    }

    rx.data = host_rx_buf;
    rx.size = (uint16_t)COMM_HOST_RX_SIZE;
    rx.timeout = 0U;
    rx.mode = BSP_UART_MODE_DMA_TO_IDLE;

    status = bsp_uart_rx(host.huart, &rx);
    host.diag.last_restart_status = (uint8_t)status;
    if (status == HAL_OK) {
        if (host.huart->hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(host.huart->hdmarx, DMA_IT_HT);
        }
    } else {
        host.diag.restart_fail_count++;
    }

    return status;
}

/**
 * @brief UART idle 事件适配回调。
 *
 * @param huart 触发事件的 UART 句柄。
 * @param size 本次 idle 收到的字节数。
 * @param user 用户上下文，当前未使用。
 */
static void uart_idle(UART_HandleTypeDef *huart, uint16_t size, void *user)
{
    (void)huart;
    (void)user;
    (void)communication_host_rx_event(size);
}

/**
 * @brief UART 错误事件适配回调。
 *
 * @param huart 触发事件的 UART 句柄。
 * @param user 用户上下文，当前未使用。
 */
static void uart_error(UART_HandleTypeDef *huart, void *user)
{
    (void)huart;
    (void)user;
    host.diag.uart_errors++;
    (void)start_rx();
}

/**
 * @brief 判断 TX DMA 是否可以写入新帧。
 *
 * @return 可以发送返回 CommunicationOk。
 */
static CommunicationStatus tx_ready(void)
{
    if ((host.ready == 0U) || (host.huart == NULL)) {
        return CommunicationState;
    }
    if (host.huart->gState != HAL_UART_STATE_READY) {
        return CommunicationBusy;
    }
    return CommunicationOk;
}

/**
 * @brief 发送已经编码到 host_tx_buf 的协议帧。
 *
 * @param length 帧长度，单位 byte。
 * @return 通讯组件状态。
 */
static CommunicationStatus send_tx(uint16_t length)
{
    bsp_uart_tx_t tx;
    HAL_StatusTypeDef status;

    if ((length == 0U) || (length > PROTO2_RT_MAX_FRAME_BYTES)) {
        host.diag.tx_errors++;
        return set_status(CommunicationSmall);
    }

    clean_tx_buf();

    tx.data = host_tx_buf;
    tx.size = length;
    tx.timeout = 0U;
    tx.mode = BSP_UART_MODE_DMA;

    status = bsp_uart_tx(host.huart, &tx);
    if (status == HAL_OK) {
        host.diag.tx_frames++;
        return set_status(CommunicationOk);
    }
    host.diag.tx_errors++;
    if (status == HAL_BUSY) {
        return set_status(CommunicationBusy);
    }
    return set_status(CommunicationIo);
}

/**
 * @brief 提交 HostAim 为 raw aim 缓存。
 *
 * @param msg 已解码 HostAim。
 */
static void commit_aim(const proto2_rt_msg_host_aim_t *msg)
{
    CommunicationAimState next;
    uint32_t primask;

    if (msg == NULL) {
        return;
    }

    (void)memset(&next, 0, sizeof(next));
    next.yaw_err_rad = msg->yaw_rad;
    next.pitch_err_rad = msg->pitch_rad;
    next.priority = host.cfg.aim_priority;
    next.host_time_us = msg->host_time_us;
    next.tick_ms = msg->host_time_us / 1000U;

    if (msg->enable != 0U) {
        next.enable = 1U;
        if ((msg->valid_mask & COMM_HOST_AIM_YAW_VALID) != 0U) {
            next.valid_mask |= COMM_HOST_AIM_YAW_VALID;
        }
        if ((msg->valid_mask & COMM_HOST_AIM_PITCH_VALID) != 0U) {
            next.valid_mask |= COMM_HOST_AIM_PITCH_VALID;
        }
    }

    primask = irq_enter();
    next.seq = host.aim.seq + 1U;
    host.aim = next;
    host.diag.aim_count++;
    irq_exit(primask);
}

/**
 * @brief 提交 RtProbe 缓存。
 *
 * @param msg 已解码 RtProbe。
 */
static void commit_probe(const proto2_rt_msg_rt_probe_t *msg)
{
    uint32_t primask;

    if (msg == NULL) {
        return;
    }

    primask = irq_enter();
    host.probe.token = msg->token;
    host.probe.time_us = msg->time_us;
    host.probe.seq++;
    host.diag.probe_count++;
    irq_exit(primask);
}

/**
 * @brief 提交 RtProbeAck 缓存。
 *
 * @param msg 已解码 RtProbeAck。
 */
static void commit_ack(const proto2_rt_msg_rt_probe_ack_t *msg)
{
    uint32_t primask;

    if (msg == NULL) {
        return;
    }

    primask = irq_enter();
    host.ack.token = msg->token;
    host.ack.request_time_us = msg->request_time_us;
    host.ack.reply_time_us = msg->reply_time_us;
    host.ack.seq++;
    host.diag.ack_count++;
    irq_exit(primask);
}

/**
 * @brief 解码并分发一帧合法 Proto2 RT 帧。
 *
 * @param user 用户上下文，当前未使用。
 * @param frame 合法帧视图。
 */
static void on_frame(void *user, const proto2_rt_frame_t *frame)
{
    proto2_rt_status_t decode_status;

    (void)user;

    if (frame == NULL) {
        host.frame_status = CommunicationNull;
        return;
    }

    host.diag.last_msg_id = frame->msg_id;
    switch (frame->msg_id) {
    case PROTO2_RT_MSG_ID_HOST_AIM: {
        proto2_rt_msg_host_aim_t msg;
        decode_status = proto2_rt_decode_host_aim(&msg, frame->payload, frame->payload_len);
        if (decode_status != PROTO2_RT_STATUS_OK) {
            host.diag.decode_errors++;
            host.frame_status = rt_status(decode_status);
            return;
        }
        commit_aim(&msg);
        break;
    }

    case PROTO2_RT_MSG_ID_RT_PROBE: {
        proto2_rt_msg_rt_probe_t msg;
        decode_status = proto2_rt_decode_rt_probe(&msg, frame->payload, frame->payload_len);
        if (decode_status != PROTO2_RT_STATUS_OK) {
            host.diag.decode_errors++;
            host.frame_status = rt_status(decode_status);
            return;
        }
        commit_probe(&msg);
        break;
    }

    case PROTO2_RT_MSG_ID_RT_PROBE_ACK: {
        proto2_rt_msg_rt_probe_ack_t msg;
        decode_status = proto2_rt_decode_rt_probe_ack(&msg, frame->payload, frame->payload_len);
        if (decode_status != PROTO2_RT_STATUS_OK) {
            host.diag.decode_errors++;
            host.frame_status = rt_status(decode_status);
            return;
        }
        commit_ack(&msg);
        break;
    }

    default:
        host.frame_status = CommunicationId;
        break;
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
CommunicationStatus communication_host_default_cfg(CommunicationHostCfg *cfg)
{
    if (cfg == NULL) {
        return CommunicationNull;
    }

    cfg->uart_id = COMM_HOST_UART_ID;
    cfg->aim_priority = (uint8_t)COMM_HOST_AIM_PRIORITY;

    return CommunicationOk;
}

CommunicationStatus communication_host_init(const CommunicationHostCfg *cfg)
{
    HAL_StatusTypeDef status;

    if (cfg == NULL) {
        return CommunicationNull;
    }

    (void)memset(&host, 0, sizeof(host));
    (void)memset(host_rx_buf, 0, sizeof(host_rx_buf));
    (void)memset(host_tx_buf, 0, sizeof(host_tx_buf));

    host.cfg = *cfg;
    host.huart = bsp_uart_handle(cfg->uart_id);
    if (host.huart == NULL) {
        return set_status(CommunicationArg);
    }

    proto2_rt_init(&host.rt);
    host.rt.role = PROTO2_RT_ROLE_DEVICE;

    status = bsp_uart_on_rx_idle(host.huart, uart_idle, NULL);
    if (status != HAL_OK) {
        return set_status(CommunicationIo);
    }

    status = bsp_uart_on(host.huart, BSP_UART_EVENT_ERROR, uart_error, NULL);
    if (status != HAL_OK) {
        return set_status(CommunicationIo);
    }

    status = start_rx();
    if (status != HAL_OK) {
        return set_status(CommunicationIo);
    }

    host.ready = 1U;
    host.diag.ready = 1U;
    return set_status(CommunicationOk);
}

CommunicationStatus communication_host_rx_event(uint16_t size)
{
    CommunicationStatus status;

    host.diag.rx_events++;
    host.diag.last_rx_size = size;

    invalidate_rx_buf();

    if (host.ready == 0U) {
        (void)start_rx();
        return set_status(CommunicationState);
    }

    if (size > COMM_HOST_RX_SIZE) {
        host.diag.decode_errors++;
        (void)start_rx();
        return set_status(CommunicationLength);
    }

    status = communication_host_feed(host_rx_buf, size);
    (void)start_rx();

    return status;
}

CommunicationStatus communication_host_feed(const uint8_t *data, uint16_t length)
{
    proto2_rt_status_t parser_status;
    CommunicationStatus status;

    if ((data == NULL) && (length != 0U)) {
        return set_status(CommunicationNull);
    }

    host.frame_status = CommunicationOk;
    parser_status = proto2_rt_feed(&host.rt, data, length, on_frame, NULL);
    sync_rt_diag();

    status = rt_status(parser_status);
    if (status == CommunicationDirection) {
        host.diag.direction_errors++;
    }

    if (status != CommunicationOk) {
        return set_status(status);
    }
    if (host.frame_status != CommunicationOk) {
        return set_status(host.frame_status);
    }

    return set_status(CommunicationOk);
}

CommunicationStatus communication_host_get_aim(CommunicationAimState *out)
{
    uint32_t primask;

    if (out == NULL) {
        return CommunicationNull;
    }
    if (host.ready == 0U) {
        return CommunicationState;
    }

    primask = irq_enter();
    *out = host.aim;
    irq_exit(primask);

    return CommunicationOk;
}

CommunicationStatus communication_host_get_probe(CommunicationProbe *out)
{
    uint32_t primask;

    if (out == NULL) {
        return CommunicationNull;
    }

    primask = irq_enter();
    *out = host.probe;
    irq_exit(primask);

    return CommunicationOk;
}

CommunicationStatus communication_host_get_ack(CommunicationProbeAck *out)
{
    uint32_t primask;

    if (out == NULL) {
        return CommunicationNull;
    }

    primask = irq_enter();
    *out = host.ack;
    irq_exit(primask);

    return CommunicationOk;
}

CommunicationStatus communication_host_send_state(const CommunicationGimbalState *state)
{
    proto2_rt_msg_gimbal_state_t msg;
    CommunicationStatus status;
    uint16_t length;

    if (state == NULL) {
        return set_status(CommunicationNull);
    }
    if (proto2_rt_can_tx(PROTO2_RT_ROLE_DEVICE, PROTO2_RT_MSG_ID_GIMBAL_STATE) == 0U) {
        return set_status(CommunicationDirection);
    }

    status = tx_ready();
    if (status != CommunicationOk) {
        return set_status(status);
    }

    msg.yaw_rad = state->yaw_rad;
    msg.pitch_rad = state->pitch_rad;
    msg.yaw_rate_rad_s = state->yaw_rate_rad_s;
    msg.pitch_rate_rad_s = state->pitch_rate_rad_s;
    msg.status = state->status;
    msg.time_us = state->time_us;

    length = proto2_rt_encode_gimbal_state(host_tx_buf, (uint16_t)sizeof(host_tx_buf), host.tx_seq++, &msg);
    return send_tx(length);
}

CommunicationStatus communication_host_send_probe(uint16_t token, uint32_t time_us)
{
    proto2_rt_msg_rt_probe_t msg;
    CommunicationStatus status;
    uint16_t length;

    if (proto2_rt_can_tx(PROTO2_RT_ROLE_DEVICE, PROTO2_RT_MSG_ID_RT_PROBE) == 0U) {
        return set_status(CommunicationDirection);
    }

    status = tx_ready();
    if (status != CommunicationOk) {
        return set_status(status);
    }

    msg.token = token;
    msg.time_us = time_us;

    length = proto2_rt_encode_rt_probe(host_tx_buf, (uint16_t)sizeof(host_tx_buf), host.tx_seq++, &msg);
    return send_tx(length);
}

CommunicationStatus communication_host_send_ack(uint16_t token,
                                                uint32_t request_time_us,
                                                uint32_t reply_time_us)
{
    proto2_rt_msg_rt_probe_ack_t msg;
    CommunicationStatus status;
    uint16_t length;

    if (proto2_rt_can_tx(PROTO2_RT_ROLE_DEVICE, PROTO2_RT_MSG_ID_RT_PROBE_ACK) == 0U) {
        return set_status(CommunicationDirection);
    }

    status = tx_ready();
    if (status != CommunicationOk) {
        return set_status(status);
    }

    msg.token = token;
    msg.request_time_us = request_time_us;
    msg.reply_time_us = reply_time_us;

    length = proto2_rt_encode_rt_probe_ack(host_tx_buf, (uint16_t)sizeof(host_tx_buf), host.tx_seq++, &msg);
    return send_tx(length);
}

CommunicationStatus communication_host_diag(CommunicationHostDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return CommunicationNull;
    }

    sync_rt_diag();
    primask = irq_enter();
    *out = host.diag;
    irq_exit(primask);

    return CommunicationOk;
}
