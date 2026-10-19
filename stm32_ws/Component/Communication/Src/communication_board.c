/**
 ******************************************************************************
 * @file    communication_board.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-21
 * @brief   板间 RT 通信缓存与发送实现。
 *
 * @note    本实现只绑定 UART、喂入 Proto2 RT parser 并维护最新快照。
 * @note    App 层负责决定何时发送、发送什么以及如何消费接收到的状态。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "communication_board.h"

#include "main.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置组件私有宏、上下文类型和编译期构型选择，事实来源仍是 C 实现。
 * ========================================================================== */
/** @brief Cortex-M7 D-Cache 行长度，单位 byte。 */
#define COMM_BOARD_CACHE_LINE 32U

/** @brief TX DMA 缓冲区按 cache line 对齐后的存储长度。 */
#define COMM_BOARD_TX_SIZE (((PROTO2_RT_MAX_FRAME_BYTES + COMM_BOARD_CACHE_LINE - 1U) / COMM_BOARD_CACHE_LINE) * COMM_BOARD_CACHE_LINE)

/** @brief RX DMA 缓冲区按 cache line 对齐后的存储长度。 */
#define COMM_BOARD_RX_STORE_SIZE (((COMM_BOARD_RX_SIZE + COMM_BOARD_CACHE_LINE - 1U) / COMM_BOARD_CACHE_LINE) * COMM_BOARD_CACHE_LINE)

/**
 * @brief 板间通信内部上下文。
 */
typedef struct
{
    CommunicationBoardCfg cfg;            /**< 配置副本。 */
    UART_HandleTypeDef *huart;            /**< 绑定的 UART 句柄。 */
    proto2_rt_t rt;                       /**< Proto2 RT parser 上下文。 */
    CommunicationBoardState board;        /**< 最新云台板状态。 */
    CommunicationChassisState chassis;    /**< 最新底盘板状态。 */
    CommunicationBoardDiag diag;          /**< 诊断快照。 */
    uint16_t tx_seq;                      /**< 发送帧序号。 */
    CommunicationStatus frame_status;     /**< frame 回调产生的状态。 */
    uint8_t ready;                        /**< 初始化完成标志。 */
} CommunicationBoard;

/** @brief 板间接收 DMA 缓冲区。 */
static uint8_t board_rx_buf[COMM_BOARD_RX_STORE_SIZE] __ALIGNED(32);

/** @brief 板间发送 DMA 缓冲区。 */
static uint8_t board_tx_buf[COMM_BOARD_TX_SIZE] __ALIGNED(32);

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * 组件状态由调用方或本文件私有上下文保存，避免运行时分配和跨层全局读写。
 * ========================================================================== */
/** @brief 板间通信单例。 */
static CommunicationBoard board_comm;

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
static uint32_t board_irq_enter(void)
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
static void board_irq_exit(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 将 Proto2 RT 状态映射为通信组件状态。
 *
 * @param status Proto2 RT 状态。
 * @return 通信组件状态。
 */
static CommunicationStatus board_rt_status(proto2_rt_status_t status)
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
 * @brief 同步 parser 统计到诊断快照。
 */
static void board_sync_diag(void)
{
    board_comm.diag.rx_frames = board_comm.rt.rx_frames;
    board_comm.diag.crc_errors = board_comm.rt.crc_errors;
    board_comm.diag.id_errors = board_comm.rt.id_errors;
    board_comm.diag.seq_errors = board_comm.rt.seq_errors;
}

/**
 * @brief 写入最近状态并透传。
 *
 * @param status 通信组件状态。
 * @return 透传 status。
 */
static CommunicationStatus board_set_status(CommunicationStatus status)
{
    board_comm.diag.last_status = (uint8_t)status;
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
static void board_invalidate_rx(void)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)board_rx_buf, (int32_t)sizeof(board_rx_buf));
}

/**
 * @brief 在 DMA TX 前清理 D-Cache。
 */
static void board_clean_tx(void)
{
    SCB_CleanDCache_by_Addr((uint32_t *)board_tx_buf, (int32_t)sizeof(board_tx_buf));
}

/**
 * @brief 判断 UART DMA idle 接收是否仍在运行。
 *
 * @return 正在运行返回 1，否则返回 0。
 */
static uint8_t board_rx_active(void)
{
    if ((board_comm.huart == NULL) || (board_comm.huart->hdmarx == NULL)) {
        return 0U;
    }
    if (board_comm.huart->RxState != HAL_UART_STATE_BUSY_RX) {
        return 0U;
    }
    if (board_comm.huart->hdmarx->State == HAL_DMA_STATE_READY) {
        return 0U;
    }
    return 1U;
}

/**
 * @brief 启动板间 UART DMA idle 接收。
 *
 * @return HAL 状态。
 */
static HAL_StatusTypeDef board_start_rx(void)
{
    bsp_uart_rx_t rx;
    HAL_StatusTypeDef status;

    if (board_comm.huart == NULL) {
        return HAL_ERROR;
    }

    if (board_rx_active() != 0U) {
        board_comm.diag.last_restart_status = (uint8_t)HAL_OK;
        return HAL_OK;
    }

    rx.data = board_rx_buf;
    rx.size = (uint16_t)COMM_BOARD_RX_SIZE;
    rx.timeout = 0U;
    rx.mode = BSP_UART_MODE_DMA_TO_IDLE;

    status = bsp_uart_rx(board_comm.huart, &rx);
    board_comm.diag.last_restart_status = (uint8_t)status;
    if (status == HAL_OK) {
        if (board_comm.huart->hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(board_comm.huart->hdmarx, DMA_IT_HT);
        }
    } else {
        board_comm.diag.restart_fail_count++;
    }

    return status;
}

/**
 * @brief UART idle 事件适配回调。
 *
 * @param huart 触发事件的 UART 句柄。
 * @param size 本次 idle 收到的字节数。
 * @param user 用户上下文，未使用。
 */
static void board_uart_idle(UART_HandleTypeDef *huart, uint16_t size, void *user)
{
    (void)huart;
    (void)user;
    (void)communication_board_rx_event(size);
}

/**
 * @brief UART 错误事件适配回调。
 *
 * @param huart 触发事件的 UART 句柄。
 * @param user 用户上下文，未使用。
 */
static void board_uart_error(UART_HandleTypeDef *huart, void *user)
{
    (void)huart;
    (void)user;
    board_comm.diag.uart_errors++;
    (void)board_start_rx();
}

/**
 * @brief 判断 TX DMA 是否可写。
 *
 * @return 可发送返回 CommunicationOk。
 */
static CommunicationStatus board_tx_ready(void)
{
    if ((board_comm.ready == 0U) || (board_comm.huart == NULL)) {
        return CommunicationState;
    }
    if (board_comm.huart->gState != HAL_UART_STATE_READY) {
        return CommunicationBusy;
    }
    return CommunicationOk;
}

/**
 * @brief 发送已经编码到 board_tx_buf 的协议帧。
 *
 * @param length 帧长度，单位 byte。
 * @return 通信组件状态。
 */
static CommunicationStatus board_send_tx(uint16_t length)
{
    bsp_uart_tx_t tx;
    HAL_StatusTypeDef status;

    if ((length == 0U) || (length > PROTO2_RT_MAX_FRAME_BYTES)) {
        board_comm.diag.tx_errors++;
        return board_set_status(CommunicationSmall);
    }

    board_clean_tx();
    tx.data = board_tx_buf;
    tx.size = length;
    tx.timeout = 0U;
    tx.mode = BSP_UART_MODE_DMA;

    status = bsp_uart_tx(board_comm.huart, &tx);
    if (status == HAL_OK) {
        board_comm.diag.tx_frames++;
        return board_set_status(CommunicationOk);
    }

    board_comm.diag.tx_errors++;
    if (status == HAL_BUSY) {
        return board_set_status(CommunicationBusy);
    }
    return board_set_status(CommunicationIo);
}

/**
 * @brief 提交云台板状态缓存。
 *
 * @param msg 解码后的协议消息。
 */
static void board_commit_state(const proto2_rt_msg_communication_board_state_t *msg)
{
    CommunicationBoardState next;
    uint32_t primask;

    if (msg == NULL) {
        return;
    }

    (void)memset(&next, 0, sizeof(next));
    next.yaw_motor_rad = msg->yaw_motor_rad;
    next.ins_yaw_rad = msg->ins_yaw_rad;
    next.ins_pitch_rad = msg->ins_pitch_rad;
    next.ins_roll_rad = msg->ins_roll_rad;
    next.gimbal_mode = msg->gimbal_mode;
    next.shoot_mode = msg->shoot_mode;
    next.rc_ch0 = msg->rc_ch0;
    next.rc_ch1 = msg->rc_ch1;
    next.rc_ch2 = msg->rc_ch2;
    next.rc_ch3 = msg->rc_ch3;
    next.rc_sw0 = msg->rc_sw0;
    next.rc_sw1 = msg->rc_sw1;
    next.status = msg->status;
    next.time_us = msg->time_us;

    primask = board_irq_enter();
    next.seq = board_comm.board.seq + 1U;
    board_comm.board = next;
    board_comm.diag.board_count++;
    board_irq_exit(primask);
}

/**
 * @brief 提交底盘板状态缓存。
 *
 * @param msg 解码后的协议消息。
 */
static void board_commit_chassis(const proto2_rt_msg_communication_chassis_state_t *msg)
{
    CommunicationChassisState next;
    uint32_t primask;

    if (msg == NULL) {
        return;
    }

    (void)memset(&next, 0, sizeof(next));
    next.yaw_rad = msg->yaw_rad;
    next.pitch_rad = msg->pitch_rad;
    next.roll_rad = msg->roll_rad;
    next.robot_id = msg->robot_id;
    next.status = msg->status;
    next.time_us = msg->time_us;

    primask = board_irq_enter();
    next.seq = board_comm.chassis.seq + 1U;
    board_comm.chassis = next;
    board_comm.diag.chassis_count++;
    board_irq_exit(primask);
}

/**
 * @brief Proto2 RT 合法帧回调。
 *
 * @param user 用户上下文，未使用。
 * @param frame 合法帧视图。
 */
static void board_on_frame(void *user, const proto2_rt_frame_t *frame)
{
    proto2_rt_status_t decode_status;

    (void)user;
    if (frame == NULL) {
        board_comm.frame_status = CommunicationNull;
        return;
    }

    board_comm.diag.last_msg_id = frame->msg_id;
    switch (frame->msg_id) {
    case PROTO2_RT_MSG_ID_COMMUNICATION_BOARD_STATE: {
        proto2_rt_msg_communication_board_state_t msg;

        decode_status = proto2_rt_decode_communication_board_state(&msg, frame->payload, frame->payload_len);
        if (decode_status != PROTO2_RT_STATUS_OK) {
            board_comm.diag.decode_errors++;
            board_comm.frame_status = board_rt_status(decode_status);
            return;
        }
        board_commit_state(&msg);
        break;
    }

    case PROTO2_RT_MSG_ID_COMMUNICATION_CHASSIS_STATE: {
        proto2_rt_msg_communication_chassis_state_t msg;

        decode_status = proto2_rt_decode_communication_chassis_state(&msg, frame->payload, frame->payload_len);
        if (decode_status != PROTO2_RT_STATUS_OK) {
            board_comm.diag.decode_errors++;
            board_comm.frame_status = board_rt_status(decode_status);
            return;
        }
        board_commit_chassis(&msg);
        break;
    }

    default:
        board_comm.frame_status = CommunicationId;
        break;
    }
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 是上层可依赖的公共语言，具体构型通过宏在编译期分派。
 * ========================================================================== */
CommunicationStatus communication_board_default_cfg(CommunicationBoardCfg *cfg)
{
    if (cfg == NULL) {
        return CommunicationNull;
    }

    cfg->enable = (uint8_t)COMM_BOARD_ENABLE_DEFAULT;
    cfg->uart_id = COMM_BOARD_UART_ID;
    return CommunicationOk;
}

CommunicationStatus communication_board_init(const CommunicationBoardCfg *cfg)
{
    HAL_StatusTypeDef status;

    if (cfg == NULL) {
        return CommunicationNull;
    }

    (void)memset(&board_comm, 0, sizeof(board_comm));
    (void)memset(board_rx_buf, 0, sizeof(board_rx_buf));
    (void)memset(board_tx_buf, 0, sizeof(board_tx_buf));

    board_comm.cfg = *cfg;
    board_comm.diag.enabled = cfg->enable;
    if (cfg->enable == 0U) {
        return board_set_status(CommunicationOk);
    }

    board_comm.huart = bsp_uart_handle(cfg->uart_id);
    if (board_comm.huart == NULL) {
        return board_set_status(CommunicationArg);
    }

    proto2_rt_init(&board_comm.rt);
    board_comm.rt.role = PROTO2_RT_ROLE_ANY;

    status = bsp_uart_on_rx_idle(board_comm.huart, board_uart_idle, NULL);
    if (status != HAL_OK) {
        return board_set_status(CommunicationIo);
    }

    status = bsp_uart_on(board_comm.huart, BSP_UART_EVENT_ERROR, board_uart_error, NULL);
    if (status != HAL_OK) {
        return board_set_status(CommunicationIo);
    }

    status = board_start_rx();
    if (status != HAL_OK) {
        return board_set_status(CommunicationIo);
    }

    board_comm.ready = 1U;
    board_comm.diag.ready = 1U;
    return board_set_status(CommunicationOk);
}

CommunicationStatus communication_board_rx_event(uint16_t size)
{
    CommunicationStatus status;

    board_comm.diag.rx_events++;
    board_comm.diag.last_rx_size = size;
    board_invalidate_rx();

    if (board_comm.ready == 0U) {
        (void)board_start_rx();
        return board_set_status(CommunicationState);
    }

    if (size > COMM_BOARD_RX_SIZE) {
        board_comm.diag.decode_errors++;
        (void)board_start_rx();
        return board_set_status(CommunicationLength);
    }

    status = communication_board_feed(board_rx_buf, size);
    (void)board_start_rx();
    return status;
}

CommunicationStatus communication_board_feed(const uint8_t *data, uint16_t length)
{
    proto2_rt_status_t parser_status;
    CommunicationStatus status;

    if ((data == NULL) && (length != 0U)) {
        return board_set_status(CommunicationNull);
    }
    if (board_comm.cfg.enable == 0U) {
        return board_set_status(CommunicationState);
    }

    board_comm.frame_status = CommunicationOk;
    parser_status = proto2_rt_feed(&board_comm.rt, data, length, board_on_frame, NULL);
    board_sync_diag();

    status = board_rt_status(parser_status);
    if (status == CommunicationDirection) {
        board_comm.diag.direction_errors++;
    }
    if (status != CommunicationOk) {
        return board_set_status(status);
    }
    if (board_comm.frame_status != CommunicationOk) {
        return board_set_status(board_comm.frame_status);
    }

    return board_set_status(CommunicationOk);
}

CommunicationStatus communication_board_get_state(CommunicationBoardState *out)
{
    uint32_t primask;

    if (out == NULL) {
        return CommunicationNull;
    }

    primask = board_irq_enter();
    *out = board_comm.board;
    board_irq_exit(primask);
    return CommunicationOk;
}

CommunicationStatus communication_board_get_chassis(CommunicationChassisState *out)
{
    uint32_t primask;

    if (out == NULL) {
        return CommunicationNull;
    }

    primask = board_irq_enter();
    *out = board_comm.chassis;
    board_irq_exit(primask);
    return CommunicationOk;
}

CommunicationStatus communication_board_send_state(const CommunicationBoardState *state)
{
    proto2_rt_msg_communication_board_state_t msg;
    CommunicationStatus status;
    uint16_t length;

    if (state == NULL) {
        return board_set_status(CommunicationNull);
    }
    if (proto2_rt_can_tx(PROTO2_RT_ROLE_ANY, PROTO2_RT_MSG_ID_COMMUNICATION_BOARD_STATE) == 0U) {
        return board_set_status(CommunicationDirection);
    }

    status = board_tx_ready();
    if (status != CommunicationOk) {
        return board_set_status(status);
    }

    msg.yaw_motor_rad = state->yaw_motor_rad;
    msg.ins_yaw_rad = state->ins_yaw_rad;
    msg.ins_pitch_rad = state->ins_pitch_rad;
    msg.ins_roll_rad = state->ins_roll_rad;
    msg.gimbal_mode = state->gimbal_mode;
    msg.shoot_mode = state->shoot_mode;
    msg.rc_ch0 = state->rc_ch0;
    msg.rc_ch1 = state->rc_ch1;
    msg.rc_ch2 = state->rc_ch2;
    msg.rc_ch3 = state->rc_ch3;
    msg.rc_sw0 = state->rc_sw0;
    msg.rc_sw1 = state->rc_sw1;
    msg.status = state->status;
    msg.time_us = state->time_us;

    length = proto2_rt_encode_communication_board_state(board_tx_buf,
                                                        (uint16_t)sizeof(board_tx_buf),
                                                        board_comm.tx_seq++,
                                                        &msg);
    return board_send_tx(length);
}

CommunicationStatus communication_board_send_chassis(const CommunicationChassisState *state)
{
    proto2_rt_msg_communication_chassis_state_t msg;
    CommunicationStatus status;
    uint16_t length;

    if (state == NULL) {
        return board_set_status(CommunicationNull);
    }
    if (proto2_rt_can_tx(PROTO2_RT_ROLE_ANY, PROTO2_RT_MSG_ID_COMMUNICATION_CHASSIS_STATE) == 0U) {
        return board_set_status(CommunicationDirection);
    }

    status = board_tx_ready();
    if (status != CommunicationOk) {
        return board_set_status(status);
    }

    msg.yaw_rad = state->yaw_rad;
    msg.pitch_rad = state->pitch_rad;
    msg.roll_rad = state->roll_rad;
    msg.robot_id = state->robot_id;
    msg.status = state->status;
    msg.time_us = state->time_us;

    length = proto2_rt_encode_communication_chassis_state(board_tx_buf,
                                                          (uint16_t)sizeof(board_tx_buf),
                                                          board_comm.tx_seq++,
                                                          &msg);
    return board_send_tx(length);
}

CommunicationStatus communication_board_diag(CommunicationBoardDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return CommunicationNull;
    }

    board_sync_diag();
    primask = board_irq_enter();
    *out = board_comm.diag;
    board_irq_exit(primask);
    return CommunicationOk;
}
