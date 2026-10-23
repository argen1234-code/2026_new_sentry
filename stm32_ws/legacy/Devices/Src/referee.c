/**
 ******************************************************************************
 * @file    referee.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-19
 * @brief   RoboMaster 裁判系统设备实现。
 *
 * @note    本文件绑定裁判系统 UART，完成 DMA idle 接收、协议喂入、状态缓存和发送。
 * @note    本文件不读取任务、遥控器、云台、底盘、发射或 UI 画面状态。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "referee.h"

#include "main.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */
/** @brief 裁判系统 DMA 缓存行对齐长度，单位 byte。 */
#define REFEREE_DMA_BUF_SIZE ((uint16_t)((REFEREE_RX_BUF_SIZE + 31U) & (uint16_t)~31U))

/** @brief 裁判系统设备私有上下文。 */
typedef struct
{
    RefereeCfg cfg;                         /**< 配置副本。 */
    RefereeState state;                     /**< 最近一次状态缓存。 */
    RefereeDiag diag;                       /**< 诊断信息。 */
    RefereeUnpack unpack;                   /**< 协议解包状态机。 */
    UART_HandleTypeDef *huart;              /**< 绑定的 UART 句柄。 */
    uint8_t rx_buf[REFEREE_DMA_BUF_SIZE] __ALIGNED(32); /**< DMA idle 接收缓存。 */
    uint8_t tx_buf[REFEREE_FRAME_MAX_SIZE]; /**< 发送打包缓存。 */
    uint8_t seq;                            /**< 发送帧序号。 */
    uint8_t ready;                          /**< 初始化完成标志。 */
} RefereeDev;

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * 设备层只缓存原始设备状态、诊断和收发缓冲，不写机器人控制语义。
 * ========================================================================== */
/** @brief 裁判系统固定设备实例。 */
static RefereeDev referee_dev;

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责帧格式、数值范围、在线时间和底层返回值检查。
 * ========================================================================== */

/**
 * @brief 保存并关闭全局中断。
 *
 * @return 进入临界区前的 PRIMASK。
 */
static uint32_t irq_enter(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

/**
 * @brief 按保存的 PRIMASK 恢复中断状态。
 *
 * @param primask 进入临界区前的 PRIMASK。
 */
static void irq_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/**
 * @brief 判断机器人 ID 是否为裁判系统合法 ID。
 *
 * @param id 机器人 ID。
 * @return 合法返回 1，否则返回 0。
 */
static uint8_t robot_id_valid(uint8_t id)
{
    return (((id >= 1U) && (id <= 7U)) || ((id >= 101U) && (id <= 107U))) ? 1U : 0U;
}

/**
 * @brief 配置裁判系统 UART 参数。
 *
 * @param huart UART 句柄。
 * @return HAL 状态。
 */
static HAL_StatusTypeDef cfg_uart(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef status;

    if (huart == NULL) {
        return HAL_ERROR;
    }

    if (huart->RxState != HAL_UART_STATE_READY) {
        (void)HAL_UART_AbortReceive(huart);
    }

    huart->Init.BaudRate = referee_dev.cfg.baud;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    status = HAL_UART_Init(huart);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_OK;
}

/**
 * @brief 更新 UART/DMA 诊断快照。
 */
static void update_diag(void)
{
    UART_HandleTypeDef *huart = referee_dev.huart;

    if (huart == NULL) {
        return;
    }

    referee_dev.diag.uart_rx_state = (uint8_t)huart->RxState;
    referee_dev.diag.uart_g_state = (uint8_t)huart->gState;
    referee_dev.diag.uart_error_code = huart->ErrorCode;

    if (huart->hdmarx != NULL) {
        referee_dev.diag.dma_state = (uint8_t)huart->hdmarx->State;
        referee_dev.diag.dma_error_code = huart->hdmarx->ErrorCode;
        referee_dev.diag.dma_left = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
    } else {
        referee_dev.diag.dma_state = 0U;
        referee_dev.diag.dma_error_code = 0U;
        referee_dev.diag.dma_left = 0U;
    }
}

/* ==========================================================================
 * 收发、解析与打包
 * --------------------------------------------------------------------------
 * 这里把字节流或寄存器数据转换成设备事实；控制含义由 Component/Input 或 Apps 解释。
 * ========================================================================== */

/**
 * @brief 让 CPU 读取 DMA 最新写入的裁判系统缓存。
 */
static void clear_rx_buf(void)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)referee_dev.rx_buf, (int32_t)REFEREE_DMA_BUF_SIZE);
}

/**
 * @brief 判断 UART DMA 接收是否仍处于运行状态。
 *
 * @param huart UART 句柄。
 * @return 接收运行中返回 1，否则返回 0。
 */
static uint8_t rx_active(const UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->hdmarx == NULL)) {
        return 0U;
    }

    if (huart->RxState != HAL_UART_STATE_BUSY_RX) {
        return 0U;
    }

    if (huart->hdmarx->State == HAL_DMA_STATE_READY) {
        return 0U;
    }

    return 1U;
}

/**
 * @brief 启动裁判系统 UART DMA idle 接收。
 *
 * @return HAL 状态。
 */
static HAL_StatusTypeDef start_rx(void)
{
    bsp_uart_rx_t rx;
    HAL_StatusTypeDef status;

    if (referee_dev.huart == NULL) {
        return HAL_ERROR;
    }

    if (rx_active(referee_dev.huart) != 0U) {
        referee_dev.diag.last_restart_status = (uint8_t)HAL_OK;
        update_diag();
        return HAL_OK;
    }

    rx.data = referee_dev.rx_buf;
    rx.size = REFEREE_RX_BUF_SIZE;
    rx.timeout = 0U;
    rx.mode = BSP_UART_MODE_DMA_TO_IDLE;

    status = bsp_uart_rx(referee_dev.huart, &rx);
    referee_dev.diag.last_restart_status = (uint8_t)status;
    if (status != HAL_OK) {
        referee_dev.diag.restart_fail_count++;
    } else if (referee_dev.huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(referee_dev.huart->hdmarx, DMA_IT_HT);
    }
    update_diag();

    return status;
}

/**
 * @brief UART idle 事件适配回调。
 *
 * @param huart UART 句柄，当前未使用。
 * @param size 接收长度，单位 byte。
 * @param user 用户上下文，当前未使用。
 */
static void uart_idle(UART_HandleTypeDef *huart, uint16_t size, void *user)
{
    (void)huart;
    (void)user;

    referee_rx_event(size);
}

/**
 * @brief UART 错误事件适配回调。
 *
 * @param huart UART 句柄，当前未使用。
 * @param user 用户上下文，当前未使用。
 */
static void uart_error(UART_HandleTypeDef *huart, void *user)
{
    (void)huart;
    (void)user;

    referee_dev.diag.error_count++;
    update_diag();
    memset(referee_dev.rx_buf, 0, sizeof(referee_dev.rx_buf));
    (void)start_rx();
}

/**
 * @brief 获取完整帧数据段指针。
 *
 * @param frame 完整原始帧。
 * @return 数据段指针。
 */
static const uint8_t *payload_ptr(const RefereeFrame *frame)
{
    return &frame->data[REFEREE_HEADER_SIZE + REFEREE_CMD_SIZE];
}

/**
 * @brief 复制固定长度数据段到状态缓存。
 *
 * @param dst 目标状态字段。
 * @param dst_size 目标字段长度。
 * @param frame 完整原始帧。
 * @param valid_bit 状态有效位。
 * @return 执行状态。
 */
static RefereeStatus copy_payload(void *dst, uint16_t dst_size, const RefereeFrame *frame, uint32_t valid_bit)
{
    if ((dst == NULL) || (frame == NULL)) {
        return RefereeNull;
    }

    if (frame->data_len < dst_size) {
        return RefereeLen;
    }

    memcpy(dst, payload_ptr(frame), dst_size);
    referee_dev.state.valid_mask |= valid_bit;
    return RefereeOk;
}

/**
 * @brief 复制机器人交互数据段到状态缓存。
 *
 * @param frame 完整原始帧。
 * @return 执行状态。
 */
static RefereeStatus copy_interact(const RefereeFrame *frame)
{
    uint16_t copy_len;

    if (frame == NULL) {
        return RefereeNull;
    }

    if (frame->data_len < REFEREE_INTERACT_HEAD_SIZE) {
        return RefereeLen;
    }

    memset(&referee_dev.state.interact, 0, sizeof(referee_dev.state.interact));
    copy_len = (frame->data_len < sizeof(referee_dev.state.interact)) ?
        frame->data_len :
        (uint16_t)sizeof(referee_dev.state.interact);
    memcpy(&referee_dev.state.interact, payload_ptr(frame), copy_len);
    referee_dev.state.valid_mask |= REFEREE_VALID_INTERACT;
    return RefereeOk;
}

/**
 * @brief 根据命令 ID 更新状态缓存。
 *
 * @param frame 完整原始帧。
 * @param tick_ms 当前时间戳，单位 ms。
 * @return 执行状态。
 */
static RefereeStatus handle_frame(const RefereeFrame *frame, uint32_t tick_ms)
{
    RefereeStatus status;
    uint32_t primask;

    if (frame == NULL) {
        return RefereeNull;
    }

    primask = irq_enter();
    referee_dev.state.last_rx_tick = tick_ms;
    referee_dev.state.last_cmd_id = frame->cmd_id;
    referee_dev.state.last_seq = frame->header.seq;
    referee_dev.diag.frame_count++;
    referee_dev.diag.last_valid_tick = tick_ms;
    referee_dev.diag.last_frame_len = frame->len;
    referee_dev.diag.last_cmd_id = frame->cmd_id;

    switch (frame->cmd_id) {
    case RefereeCmdGameStatus:
        status = copy_payload(&referee_dev.state.game_status, sizeof(RefereeGameStatus), frame, REFEREE_VALID_GAME_STATUS);
        break;
    case RefereeCmdGameResult:
        status = copy_payload(&referee_dev.state.game_result, sizeof(RefereeGameResult), frame, REFEREE_VALID_GAME_RESULT);
        break;
    case RefereeCmdGameRobotHp:
        status = copy_payload(&referee_dev.state.game_robot_hp, sizeof(RefereeGameRobotHp), frame, REFEREE_VALID_GAME_ROBOT_HP);
        break;
    case RefereeCmdEventData:
        status = copy_payload(&referee_dev.state.event_data, sizeof(RefereeEventData), frame, REFEREE_VALID_EVENT_DATA);
        break;
    case RefereeCmdWarning:
        status = copy_payload(&referee_dev.state.warning, sizeof(RefereeWarning), frame, REFEREE_VALID_WARNING);
        break;
    case RefereeCmdDartInfo:
        status = copy_payload(&referee_dev.state.dart_info, sizeof(RefereeDartInfo), frame, REFEREE_VALID_DART_INFO);
        break;
    case RefereeCmdRobotStatus:
        status = copy_payload(&referee_dev.state.robot_status, sizeof(RefereeRobotStatus), frame, REFEREE_VALID_ROBOT_STATUS);
        break;
    case RefereeCmdPowerHeat:
        status = copy_payload(&referee_dev.state.power_heat, sizeof(RefereePowerHeat), frame, REFEREE_VALID_POWER_HEAT);
        break;
    case RefereeCmdRobotPos:
        status = copy_payload(&referee_dev.state.robot_pos, sizeof(RefereeRobotPos), frame, REFEREE_VALID_ROBOT_POS);
        break;
    case RefereeCmdBuff:
        status = copy_payload(&referee_dev.state.buff, sizeof(RefereeBuff), frame, REFEREE_VALID_BUFF);
        break;
    case RefereeCmdRobotHurt:
        status = copy_payload(&referee_dev.state.hurt, sizeof(RefereeHurt), frame, REFEREE_VALID_HURT);
        break;
    case RefereeCmdShootData:
        status = copy_payload(&referee_dev.state.shoot_data, sizeof(RefereeShootData), frame, REFEREE_VALID_SHOOT_DATA);
        break;
    case RefereeCmdProjectile:
        status = copy_payload(&referee_dev.state.projectile, sizeof(RefereeProjectile), frame, REFEREE_VALID_PROJECTILE);
        break;
    case RefereeCmdRfid:
        status = copy_payload(&referee_dev.state.rfid, sizeof(RefereeRfid), frame, REFEREE_VALID_RFID);
        break;
    case RefereeCmdDartClient:
        status = copy_payload(&referee_dev.state.dart_client, sizeof(RefereeDartClient), frame, REFEREE_VALID_DART_CLIENT);
        break;
    case RefereeCmdGroundPos:
        status = copy_payload(&referee_dev.state.ground_pos, sizeof(RefereeGroundPos), frame, REFEREE_VALID_GROUND_POS);
        break;
    case RefereeCmdRadarMark:
        status = copy_payload(&referee_dev.state.radar_mark, sizeof(RefereeRadarMark), frame, REFEREE_VALID_RADAR_MARK);
        break;
    case RefereeCmdSentryInfo:
        status = copy_payload(&referee_dev.state.sentry_info, sizeof(RefereeSentryInfo), frame, REFEREE_VALID_SENTRY_INFO);
        break;
    case RefereeCmdRadarInfo:
        status = copy_payload(&referee_dev.state.radar_info, sizeof(RefereeRadarInfo), frame, REFEREE_VALID_RADAR_INFO);
        break;
    case RefereeCmdRobotInteract:
        status = copy_interact(frame);
        break;
    case RefereeCmdCustomRobot:
        status = copy_payload(&referee_dev.state.custom_robot, sizeof(RefereeCustomRobot), frame, REFEREE_VALID_CUSTOM_ROBOT);
        break;
    case RefereeCmdMapCommand:
        status = copy_payload(&referee_dev.state.map_command, sizeof(RefereeMapCommand), frame, REFEREE_VALID_MAP_COMMAND);
        break;
    case RefereeCmdRcData:
        status = copy_payload(&referee_dev.state.rc_data, sizeof(RefereeRcData), frame, REFEREE_VALID_RC_DATA);
        break;
    case RefereeCmdMapRobot:
        status = copy_payload(&referee_dev.state.map_robot, sizeof(RefereeMapRobot), frame, REFEREE_VALID_MAP_ROBOT);
        break;
    case RefereeCmdCustomClient:
        status = copy_payload(&referee_dev.state.custom_client, sizeof(RefereeCustomClient), frame, REFEREE_VALID_CUSTOM_CLIENT);
        break;
    case RefereeCmdMapData:
        status = copy_payload(&referee_dev.state.map_data, sizeof(RefereeMapData), frame, REFEREE_VALID_MAP_DATA);
        break;
    case RefereeCmdCustomInfo:
        status = copy_payload(&referee_dev.state.custom_info, sizeof(RefereeCustomInfo), frame, REFEREE_VALID_CUSTOM_INFO);
        break;
    case RefereeCmdRobotCustom:
        status = copy_payload(&referee_dev.state.robot_custom, sizeof(RefereeRobotCustom), frame, REFEREE_VALID_ROBOT_CUSTOM);
        break;
    default:
        referee_dev.diag.unknown_cmd_count++;
        status = RefereeArg;
        break;
    }

    if (status == RefereeLen) {
        referee_dev.diag.length_error_count++;
    }
    irq_exit(primask);
    return status;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
RefereeStatus referee_init(void)
{
    HAL_StatusTypeDef status;

    memset(&referee_dev, 0, sizeof(referee_dev));
    referee_dev.cfg.uart_id = REFEREE_UART_ID;
    referee_dev.cfg.baud = REFEREE_UART_BAUD;
    referee_dev.cfg.tx_timeout_ms = REFEREE_TX_TIMEOUT_MS;
    referee_dev.huart = bsp_uart_handle(referee_dev.cfg.uart_id);
    referee_unpack_reset(&referee_dev.unpack);

    if (referee_dev.huart == NULL) {
        return RefereeIo;
    }

    status = cfg_uart(referee_dev.huart);
    if (status != HAL_OK) {
        return RefereeIo;
    }

    status = bsp_uart_on_rx_idle(referee_dev.huart, uart_idle, NULL);
    if (status != HAL_OK) {
        return RefereeIo;
    }

    status = bsp_uart_on(referee_dev.huart, BSP_UART_EVENT_ERROR, uart_error, NULL);
    if (status != HAL_OK) {
        return RefereeIo;
    }

    status = start_rx();
    referee_dev.ready = (status == HAL_OK) ? 1U : 0U;
    referee_dev.diag.ready = referee_dev.ready;

    return (status == HAL_OK) ? RefereeOk : RefereeIo;
}

void referee_rx_event(uint16_t size)
{
    uint32_t tick = HAL_GetTick();

    referee_dev.diag.rx_event_count++;
    referee_dev.diag.last_event_tick = tick;
    referee_dev.diag.last_size = size;
    clear_rx_buf();
    update_diag();

    if (referee_dev.ready == 0U) {
        (void)start_rx();
        return;
    }

    if (size > REFEREE_RX_BUF_SIZE) {
        referee_dev.diag.length_error_count++;
        memset(referee_dev.rx_buf, 0, sizeof(referee_dev.rx_buf));
        (void)start_rx();
        return;
    }

    (void)referee_feed(referee_dev.rx_buf, size, tick);
    (void)start_rx();
}

RefereeStatus referee_feed(const uint8_t *data, uint16_t len, uint32_t tick_ms)
{
    RefereeFrame frame;
    RefereeStatus result = RefereeOk;
    RefereeStatus status;
    uint16_t i;

    if ((data == NULL) && (len != 0U)) {
        return RefereeNull;
    }

    for (i = 0U; i < len; i++) {
        status = referee_unpack_byte(&referee_dev.unpack, data[i], &frame);
        if (status == RefereeOk) {
            status = handle_frame(&frame, tick_ms);
            if ((status != RefereeOk) && (result == RefereeOk)) {
                result = status;
            }
        } else if (status == RefereeCrc) {
            referee_dev.diag.crc_error_count++;
            if (result == RefereeOk) {
                result = status;
            }
        } else if (status == RefereeLen) {
            referee_dev.diag.length_error_count++;
            if (result == RefereeOk) {
                result = status;
            }
        } else if ((status != RefereeNoFrame) && (result == RefereeOk)) {
            result = status;
        }
    }

    return result;
}

RefereeStatus referee_get(RefereeState *out)
{
    uint32_t primask;

    if (out == NULL) {
        return RefereeNull;
    }

    primask = irq_enter();
    *out = referee_dev.state;
    irq_exit(primask);
    return RefereeOk;
}

RefereeStatus referee_diag(RefereeDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return RefereeNull;
    }

    update_diag();
    primask = irq_enter();
    *out = referee_dev.diag;
    irq_exit(primask);
    return RefereeOk;
}

uint8_t referee_online(uint32_t timeout_ms, uint32_t now_ms)
{
    RefereeState state;

    if (referee_get(&state) != RefereeOk) {
        return 0U;
    }

    if (robot_id_valid(state.robot_status.robot_id) == 0U) {
        return 0U;
    }

    if (timeout_ms == 0U) {
        return 1U;
    }

    if (state.last_rx_tick == 0U) {
        return 0U;
    }

    return ((now_ms - state.last_rx_tick) <= timeout_ms) ? 1U : 0U;
}

RefereeStatus referee_send(uint16_t cmd_id, const uint8_t *payload, uint16_t payload_len)
{
    bsp_uart_tx_t tx;
    RefereeStatus pack_status;
    HAL_StatusTypeDef tx_status;
    uint16_t out_len = 0U;

    if (referee_dev.ready == 0U) {
        return RefereeBadState;
    }

    pack_status = referee_pack_frame(cmd_id,
                                     payload,
                                     payload_len,
                                     referee_dev.seq++,
                                     referee_dev.tx_buf,
                                     (uint16_t)sizeof(referee_dev.tx_buf),
                                     &out_len);
    if (pack_status != RefereeOk) {
        return pack_status;
    }

    tx.data = referee_dev.tx_buf;
    tx.size = out_len;
    tx.timeout = referee_dev.cfg.tx_timeout_ms;
    tx.mode = BSP_UART_MODE_POLL;
    tx_status = bsp_uart_tx(referee_dev.huart, &tx);
    referee_dev.diag.last_tx_status = (uint8_t)tx_status;
    if (tx_status != HAL_OK) {
        referee_dev.diag.tx_error_count++;
        return RefereeIo;
    }

    referee_dev.diag.tx_count++;
    return RefereeOk;
}

RefereeStatus referee_send_interact(uint16_t sender_id,
                                    uint16_t receiver_id,
                                    uint16_t data_cmd_id,
                                    const uint8_t *data,
                                    uint16_t data_len)
{
    bsp_uart_tx_t tx;
    RefereeStatus pack_status;
    HAL_StatusTypeDef tx_status;
    uint16_t out_len = 0U;

    if (referee_dev.ready == 0U) {
        return RefereeBadState;
    }

    if ((robot_id_valid((uint8_t)sender_id) == 0U) || (receiver_id == 0U)) {
        return RefereeArg;
    }

    pack_status = referee_pack_interact(sender_id,
                                        receiver_id,
                                        data_cmd_id,
                                        data,
                                        data_len,
                                        referee_dev.seq++,
                                        referee_dev.tx_buf,
                                        (uint16_t)sizeof(referee_dev.tx_buf),
                                        &out_len);
    if (pack_status != RefereeOk) {
        return pack_status;
    }

    tx.data = referee_dev.tx_buf;
    tx.size = out_len;
    tx.timeout = referee_dev.cfg.tx_timeout_ms;
    tx.mode = BSP_UART_MODE_POLL;
    tx_status = bsp_uart_tx(referee_dev.huart, &tx);
    referee_dev.diag.last_tx_status = (uint8_t)tx_status;
    if (tx_status != HAL_OK) {
        referee_dev.diag.tx_error_count++;
        return RefereeIo;
    }

    referee_dev.diag.tx_count++;
    return RefereeOk;
}

uint8_t referee_robot_id(void)
{
    RefereeState state;

    if (referee_get(&state) != RefereeOk) {
        return 0U;
    }

    return state.robot_status.robot_id;
}

uint32_t referee_last_rx_tick(void)
{
    RefereeState state;

    if (referee_get(&state) != RefereeOk) {
        return 0U;
    }

    return state.last_rx_tick;
}

RefereeStatus referee_chassis_budget(float *limit, float *buffer)
{
    RefereeState state;

    if ((limit == NULL) || (buffer == NULL)) {
        return RefereeNull;
    }

    if (referee_get(&state) != RefereeOk) {
        return RefereeBadState;
    }

    if ((state.valid_mask & (REFEREE_VALID_ROBOT_STATUS | REFEREE_VALID_POWER_HEAT)) !=
        (REFEREE_VALID_ROBOT_STATUS | REFEREE_VALID_POWER_HEAT)) {
        return RefereeBadState;
    }

    *limit = (float)state.robot_status.chassis_power_limit;
    *buffer = (float)state.power_heat.buffer_energy;
    return RefereeOk;
}

RefereeStatus referee_shoot_heat(uint8_t shooter, uint16_t *heat_limit, uint16_t *heat)
{
    RefereeState state;

    if ((heat_limit == NULL) || (heat == NULL)) {
        return RefereeNull;
    }

    if (referee_get(&state) != RefereeOk) {
        return RefereeBadState;
    }

    if ((state.valid_mask & (REFEREE_VALID_ROBOT_STATUS | REFEREE_VALID_POWER_HEAT)) !=
        (REFEREE_VALID_ROBOT_STATUS | REFEREE_VALID_POWER_HEAT)) {
        return RefereeBadState;
    }

    *heat_limit = state.robot_status.shooter_barrel_heat_limit;
    if (shooter == 1U) {
        *heat = state.power_heat.shooter_17mm_1_barrel_heat;
    } else if (shooter == 2U) {
        *heat = state.power_heat.shooter_17mm_2_barrel_heat;
    } else if (shooter == 3U) {
        *heat = state.power_heat.shooter_42mm_barrel_heat;
    } else {
        return RefereeArg;
    }

    return RefereeOk;
}
