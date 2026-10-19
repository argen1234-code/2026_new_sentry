/**
 ******************************************************************************
 * @file    rc.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   遥控器输入设备实现。
 *
 * @note    本文件管理配置 UART 上的遥控器输入，使用 bsp_uart 进行 HAL 回调分发，
 *          只缓存原始遥控帧。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "rc.h"

#include "main.h"

#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */
/** @brief 遥控器 DMA idle 接收缓存，独占一个 D-Cache line。 */
static uint8_t rc_rx_buf[RC_DMA_BUF_BYTES] __ALIGNED(32);

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * 设备层只缓存原始设备状态、诊断和收发缓冲，不写机器人控制语义。
 * ========================================================================== */
/** @brief 最近一次有效原始帧。 */
static RcFrame rc_last;

/** @brief 遥控器接收诊断缓存。 */
static RcDiag rc_diag_cache;

/** @brief 遥控器设备是否完成初始化。 */
static uint8_t rc_ready;

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责帧格式、数值范围、在线时间和底层返回值检查。
 * ========================================================================== */

/**
 * @brief 计算 int16_t 绝对值。
 *
 * @param value 输入值。
 * @return 绝对值结果。
 */
static int16_t abs_i16(int16_t value)
{
    return (value >= 0) ? value : (int16_t)(-value);
}

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
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 按 DBUS 遥控链路配置接收 UART。
 *
 * 当前配置使用 18 byte DBUS 帧，UART 使用 100 kbaud、
 * 8 数据位、偶校验、2 stop bit。
 *
 * @param huart 遥控器 UART 句柄。
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

    huart->Init.BaudRate = RC_UART_BAUD;
    huart->Init.WordLength = UART_WORDLENGTH_9B;
    huart->Init.StopBits = UART_STOPBITS_2;
    huart->Init.Parity = UART_PARITY_EVEN;
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

    __HAL_UART_DISABLE(huart);
    CLEAR_BIT(huart->Instance->CR2, USART_CR2_RXINV);
    __HAL_UART_ENABLE(huart);

    return HAL_OK;
}

/**
 * @brief 更新遥控器 UART/DMA 底层诊断快照。
 */
static void update_diag(void)
{
    UART_HandleTypeDef *huart = bsp_uart_handle(RC_UART_ID);

    if (huart == NULL) {
        return;
    }

    rc_diag_cache.uart_rx_state = (uint8_t)huart->RxState;
    rc_diag_cache.uart_g_state = (uint8_t)huart->gState;
    rc_diag_cache.uart_error_code = huart->ErrorCode;

    if (huart->hdmarx != NULL) {
        rc_diag_cache.dma_state = (uint8_t)huart->hdmarx->State;
        rc_diag_cache.dma_error_code = huart->hdmarx->ErrorCode;
        rc_diag_cache.dma_left = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
    } else {
        rc_diag_cache.dma_state = 0U;
        rc_diag_cache.dma_error_code = 0U;
        rc_diag_cache.dma_left = 0U;
    }
}

/* ==========================================================================
 * 收发、解析与打包
 * --------------------------------------------------------------------------
 * 这里把字节流或寄存器数据转换成设备事实；控制含义由 Component/Input 或 Apps 解释。
 * ========================================================================== */

/**
 * @brief 让 CPU 读取 DMA 最新写入的遥控器缓存。
 *
 * M7 D-Cache 不会自动感知 DMA 写入，解析前必须失效对应缓存行。
 */
static void clear_rx_buf(void)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)rc_rx_buf, (int32_t)RC_DMA_BUF_BYTES);
}

/**
 * @brief 判断遥控器 UART DMA 接收是否仍处于运行状态。
 *
 * @param huart 遥控器 UART 句柄。
 * @return 接收仍在运行返回 1，否则返回 0。
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
 * @brief 启动遥控器 UART DMA idle 接收。
 *
 * @return 返回 HAL UART 状态码。
 */
static HAL_StatusTypeDef start_rx(void)
{
    UART_HandleTypeDef *huart = bsp_uart_handle(RC_UART_ID);
    bsp_uart_rx_t rx;
    HAL_StatusTypeDef status;

    if (huart == NULL) {
        return HAL_ERROR;
    }

    if (rx_active(huart) != 0U) {
        rc_diag_cache.last_restart_status = (uint8_t)HAL_OK;
        update_diag();
        return HAL_OK;
    }

    rx.data = rc_rx_buf;
    rx.size = RC_RX_BUF_SIZE;
    rx.timeout = 0U;
    rx.mode = BSP_UART_MODE_DMA_TO_IDLE;

    status = bsp_uart_rx(huart, &rx);
    rc_diag_cache.last_restart_status = (uint8_t)status;
    if (status != HAL_OK) {
        rc_diag_cache.restart_fail_count++;
    } else if (huart->hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
    update_diag();

    return status;
}

/**
 * @brief 遥控器 UART idle 事件适配回调。
 *
 * @param huart 触发事件的 UART 句柄。
 * @param size 本次接收长度。
 * @param user 用户上下文，当前未使用。
 */
static void uart_idle(UART_HandleTypeDef *huart, uint16_t size, void *user)
{
    (void)huart;
    (void)user;

    rc_rx_event(size);
}

/**
 * @brief 遥控器 UART 错误事件适配回调。
 *
 * @param huart 触发事件的 UART 句柄。
 * @param user 用户上下文，当前未使用。
 */
static void uart_error(UART_HandleTypeDef *huart, void *user)
{
    (void)huart;
    (void)user;

    rc_error_event();
}

/**
 * @brief 提交最近一次有效原始帧。
 *
 * @param data 原始帧数据。
 * @param size 原始帧长度。
 * @param tick_ms 接收时间戳。
 */
static void commit_frame(const uint8_t *data, uint16_t size, uint32_t tick_ms)
{
    uint32_t primask;

    if ((data == NULL) || (size != RC_FRAME_LEN)) {
        return;
    }

    primask = irq_enter();
    memcpy(rc_last.data, data, RC_FRAME_LEN);
    rc_last.size = size;
    rc_last.tick_ms = tick_ms;
    rc_last.seq++;
    irq_exit(primask);
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
HAL_StatusTypeDef rc_init(void)
{
    UART_HandleTypeDef *huart = bsp_uart_handle(RC_UART_ID);
    HAL_StatusTypeDef status;

    if (huart == NULL) {
        return HAL_ERROR;
    }

    status = cfg_uart(huart);
    if (status != HAL_OK) {
        return status;
    }

    memset(&rc_last, 0, sizeof(rc_last));
    memset(rc_rx_buf, 0, sizeof(rc_rx_buf));
    memset(&rc_diag_cache, 0, sizeof(rc_diag_cache));

    status = bsp_uart_on_rx_idle(huart, uart_idle, NULL);
    if (status != HAL_OK) {
        return status;
    }

    status = bsp_uart_on(huart, BSP_UART_EVENT_ERROR, uart_error, NULL);
    if (status != HAL_OK) {
        return status;
    }

    status = start_rx();
    if (status == HAL_OK) {
        rc_ready = 1U;
    }
    rc_diag_cache.ready = rc_ready;

    return status;
}

void rc_rx_event(uint16_t size)
{
    UART_HandleTypeDef *huart = bsp_uart_handle(RC_UART_ID);
    uint32_t now_tick;

    rc_diag_cache.rx_event_count++;
    rc_diag_cache.last_event_tick = HAL_GetTick();
    rc_diag_cache.last_size = size;
    rc_diag_cache.last_frame_ok = 0U;
    if (huart != NULL) {
        rc_diag_cache.last_event_type = (uint8_t)HAL_UARTEx_GetRxEventType(huart);
    }
    clear_rx_buf();
    update_diag();
    if (rc_ready == 0U) {
        (void)start_rx();
        return;
    }

    if (size == RC_FRAME_LEN) {
        now_tick = HAL_GetTick();
        commit_frame(rc_rx_buf, size, now_tick);
        rc_diag_cache.valid_count++;
        rc_diag_cache.last_valid_tick = now_tick;
        rc_diag_cache.last_frame_ok = 1U;
    } else {
        rc_diag_cache.invalid_size_count++;
        memset(rc_rx_buf, 0, sizeof(rc_rx_buf));
    }

    (void)start_rx();
}

void rc_error_event(void)
{
    rc_diag_cache.error_count++;
    update_diag();
    memset(rc_rx_buf, 0, sizeof(rc_rx_buf));
    (void)start_rx();
}

uint8_t rc_frame(RcFrame *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = irq_enter();
    *out = rc_last;
    irq_exit(primask);

    return 1U;
}

uint8_t rc_diag(RcDiag *out)
{
    uint32_t primask;

    if (out == NULL) {
        return 0U;
    }

    primask = irq_enter();
    update_diag();
    *out = rc_diag_cache;
    irq_exit(primask);

    return 1U;
}

void rc_dbus_state_clear(RcDbusState *state)
{
    if (state == NULL) {
        return;
    }

    memset(state, 0, sizeof(*state));
    state->rc.sw[0] = RC_SW_DOWN;
    state->rc.sw[1] = RC_SW_DOWN;
    state->online = 0U;
}

uint8_t rc_sw_valid(uint8_t sw)
{
    return ((sw == RC_SW_UP) ||
            (sw == RC_SW_MID) ||
            (sw == RC_SW_DOWN)) ? 1U : 0U;
}

uint8_t rc_dbus_state_valid(const RcDbusState *state)
{
    uint32_t i;

    if (state == NULL) {
        return 0U;
    }

    for (i = 0U; i < 5U; i++) {
        if (abs_i16(state->rc.ch[i]) > RC_CH_ERR_VALUE) {
            return 0U;
        }
    }

    if ((rc_sw_valid(state->rc.sw[0]) == 0U) ||
        (rc_sw_valid(state->rc.sw[1]) == 0U)) {
        return 0U;
    }

    return 1U;
}

uint8_t rc_sw_is_up(uint8_t sw)
{
    return (sw == RC_SW_UP) ? 1U : 0U;
}

uint8_t rc_sw_is_mid(uint8_t sw)
{
    return (sw == RC_SW_MID) ? 1U : 0U;
}

uint8_t rc_sw_is_down(uint8_t sw)
{
    return (sw == RC_SW_DOWN) ? 1U : 0U;
}

uint8_t rc_key_down(uint16_t key, uint16_t mask)
{
    return ((key & mask) != 0U) ? 1U : 0U;
}

int16_t rc_deadband(int16_t input, int16_t deadband)
{
    int16_t band = deadband;

    if (band < 0) {
        band = (int16_t)(-band);
    }

    if ((input > -band) && (input < band)) {
        return 0;
    }

    return input;
}
