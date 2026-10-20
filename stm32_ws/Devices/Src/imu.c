/**
 ******************************************************************************
 * @file    imu.c
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-17
 * @brief   IMU 设备公共接口实现。
 *
 * @note    本文件把配置 SPI、ACC/GYRO 片选 GPIO、DWT 延时和时间基准适配给
 *          当前板载 IMU 实现。
 * @note    当前板卡实际使用 BMI088，具体寄存器协议和初始化序列保留在
 *          imu_bmi088.c 内部，上层只使用 imu.h 的抽象采样接口。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "imu.h"

#include "main.h"
#include "bsp_dwt.h"
#include "bsp_gpio.h"
#include "bsp_spi.h"
#include "imu_bmi088.h"
#include "robot_board.h"

#include <math.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */

/* ==========================================================================
 * 状态缓存与工作区
 * --------------------------------------------------------------------------
 * 设备层只缓存原始设备状态、诊断和收发缓冲，不写机器人控制语义。
 * ========================================================================== */

/**
 * @brief 板载 BMI088 设备对象。
 */
static bmi088_t imu_bmi088;

/**
 * @brief IMU 最近一次抽象采样值。
 */
static imu_sample_t imu_last_sample;

/**
 * @brief IMU 是否已经完成初始化。
 */
static uint8_t imu_ready;

/** @brief 温度读取周期，单位 ms。 */
#define IMU_TEMP_PERIOD_MS 100U

/** @brief 连续读取错误达到该值时判定 IMU 失效。 */
#define IMU_ERR_DEBOUNCE 20U

/** @brief 读取/标定互斥忙标志。 */
static volatile uint8_t imu_busy;

/** @brief 最近一次温度读取时间，单位 ms。 */
static uint32_t imu_last_temp_ms;

/** @brief 连续读取错误计数。 */
static uint32_t imu_err_count;

/** @brief 最近一次成功读取时间，单位 ms。 */
static uint32_t imu_last_ok_ms;

/** @brief 最近一次底层读取状态。 */
static int32_t imu_last_status;

/**
 * @brief 板级 BMI088 默认标定参数。
 *
 * 该参数属于当前板卡和当前 IMU 的实测数据，不属于 BMI088 器件通用驱动。
 */
static bmi088_calibration_t imu_bmi088_default_cal = {
    .gyro_offset = {-0.000681414269f, -0.00134240754f, -0.00143384014f},
    .accel_offset = {0.299675316f, 0.0720675737f, 0.0f},
    .g_norm = 9.84484291f,
    .temp_when_cali = 40.0f,
    .accel_offset_enabled = 1U,
};

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责帧格式、数值范围、在线时间和底层返回值检查。
 * ========================================================================== */

/**
 * @brief 将公开 IMU 标定格式复制到 BMI088 标定格式。
 *
 * @param dst BMI088 标定参数。
 * @param src 公开 IMU 标定参数。
 */
static void cali_to_bmi088(bmi088_calibration_t *dst, const ImuCali *src)
{
    if ((dst == NULL) || (src == NULL)) {
        return;
    }

    dst->gyro_offset[0] = src->gyro_offset[0];
    dst->gyro_offset[1] = src->gyro_offset[1];
    dst->gyro_offset[2] = src->gyro_offset[2];
    dst->accel_offset[0] = src->accel_offset[0];
    dst->accel_offset[1] = src->accel_offset[1];
    dst->accel_offset[2] = src->accel_offset[2];
    dst->g_norm = src->g_norm;
    dst->temp_when_cali = src->temp_when_cali;
    dst->accel_offset_enabled = src->accel_offset_enabled;
}

/**
 * @brief 将 BMI088 当前标定复制到公开 IMU 标定格式。
 *
 * @param out 公开 IMU 标定参数。
 * @param src BMI088 标定来源。
 */
static void bmi088_to_cali(ImuCali *out, const bmi088_t *src)
{
    if ((out == NULL) || (src == NULL)) {
        return;
    }

    out->gyro_offset[0] = src->gyro_offset[0];
    out->gyro_offset[1] = src->gyro_offset[1];
    out->gyro_offset[2] = src->gyro_offset[2];
    out->accel_offset[0] = src->accel_offset[0];
    out->accel_offset[1] = src->accel_offset[1];
    out->accel_offset[2] = src->accel_offset[2];
    out->g_norm = src->g_norm;
    out->temp_when_cali = src->temp_when_cali;
    out->accel_offset_enabled = src->accel_offset_enabled;
}

/**
 * @brief 将公开 IMU 标定复制到 BMI088 当前设备对象。
 *
 * @param dev BMI088 设备对象。
 * @param cali 公开 IMU 标定参数。
 */
static void cali_to_dev(bmi088_t *dev, const ImuCali *cali)
{
    if ((dev == NULL) || (cali == NULL)) {
        return;
    }

    dev->gyro_offset[0] = cali->gyro_offset[0];
    dev->gyro_offset[1] = cali->gyro_offset[1];
    dev->gyro_offset[2] = cali->gyro_offset[2];
    dev->accel_offset[0] = cali->accel_offset[0];
    dev->accel_offset[1] = cali->accel_offset[1];
    dev->accel_offset[2] = cali->accel_offset[2];
    dev->g_norm = cali->g_norm;
    dev->temp_when_cali = cali->temp_when_cali;
    dev->accel_offset_enabled = cali->accel_offset_enabled;
    dev->accel_scale = (cali->g_norm > 1.0f) ? (9.81f / cali->g_norm) : 1.0f;
}

/**
 * @brief 获取 BMI088 加速度计片选 GPIO。
 *
 * @return 片选引脚描述，失败返回 NULL。
 */
static const bsp_gpio_pin_t *acc_cs_pin(void)
{
    return bsp_gpio_pin(IMU_ACC_CS_GPIO_ID);
}

/**
 * @brief 获取 BMI088 陀螺仪片选 GPIO。
 *
 * @return 片选引脚描述，失败返回 NULL。
 */
static const bsp_gpio_pin_t *gyr_cs_pin(void)
{
    return bsp_gpio_pin(IMU_GYRO_CS_GPIO_ID);
}

/**
 * @brief 获取 BMI088 使用的 SPI 句柄。
 *
 * @return SPI 句柄，失败返回 NULL。
 */
static SPI_HandleTypeDef *spi_handle(void)
{
    return bsp_spi_handle(IMU_SPI_ID);
}

/**
 * @brief 检查板载 IMU 依赖的 SPI 和片选资源是否可用。
 *
 * @return 可用返回 BMI088_STATUS_OK，否则返回 BMI088_STATUS_PORT_ERROR。
 */
static int res_ok(void)
{
    if ((spi_handle() == NULL) ||
        (acc_cs_pin() == NULL) ||
        (gyr_cs_pin() == NULL)) {
        return BMI088_STATUS_PORT_ERROR;
    }

    return BMI088_STATUS_OK;
}

/**
 * @brief 将 BMI088 驱动状态转换为 IMU 设备状态。
 *
 * @param status BMI088 驱动状态。
 * @return IMU 设备状态。
 */
static imu_status_t conv_status(int status)
{
    if (status == BMI088_STATUS_OK) {
        return IMU_OK;
    }

    if (status == BMI088_STATUS_INVALID_ARG) {
        return IMU_ERR_ARG;
    }

    if (status == BMI088_STATUS_PORT_ERROR) {
        return IMU_ERR_IO;
    }

    if ((status == BMI088_STATUS_CALI_TIMEOUT) ||
        (status == BMI088_STATUS_CALI_UNSTABLE)) {
        return IMU_ERR_CALI;
    }

    return IMU_ERR;
}

/**
 * @brief 进入 IMU 短临界区。
 *
 * @return 进入前的 PRIMASK。
 */
static uint32_t imu_critical_enter(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

/**
 * @brief 退出 IMU 短临界区。
 *
 * @param primask 进入前的 PRIMASK。
 */
static void imu_critical_exit(uint32_t primask)
{
    __set_PRIMASK(primask);
}

/**
 * @brief 尝试占用 IMU 读/标定互斥。
 *
 * @return 非 0 表示占用成功。
 */
static uint8_t imu_try_begin(void)
{
    uint32_t primask = imu_critical_enter();
    uint8_t ok = 0U;

    if (imu_busy == 0U) {
        imu_busy = 1U;
        ok = 1U;
    }
    imu_critical_exit(primask);
    return ok;
}

/**
 * @brief 释放 IMU 读/标定互斥。
 */
static void imu_end(void)
{
    uint32_t primask = imu_critical_enter();
    imu_busy = 0U;
    imu_critical_exit(primask);
}

/**
 * @brief 从 BMI088 对象刷新抽象采样缓存。
 */
static void update_sample(void)
{
    imu_last_sample.accel[0] = imu_bmi088.accel[0];
    imu_last_sample.accel[1] = imu_bmi088.accel[1];
    imu_last_sample.accel[2] = imu_bmi088.accel[2];
    imu_last_sample.gyro[0] = imu_bmi088.gyro[0];
    imu_last_sample.gyro[1] = imu_bmi088.gyro[1];
    imu_last_sample.gyro[2] = imu_bmi088.gyro[2];
    imu_last_sample.temperature = imu_bmi088.temperature;
    imu_last_sample.cal_temp = imu_bmi088.temp_when_cali;
}

/**
 * @brief 设置 BMI088 加速度计片选电平。
 *
 * @param user port 用户上下文，本实现未使用。
 * @param level BMI088 驱动传入的片选电平，0 表示选中，非 0 表示释放。
 */
static void acc_cs(void *user, uint8_t level)
{
    (void)user;
    (void)bsp_gpio_write(acc_cs_pin(), (level == 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief 设置 BMI088 陀螺仪片选电平。
 *
 * @param user port 用户上下文，本实现未使用。
 * @param level BMI088 驱动传入的片选电平，0 表示选中，非 0 表示释放。
 */
static void gyr_cs(void *user, uint8_t level)
{
    (void)user;
    (void)bsp_gpio_write(gyr_cs_pin(), (level == 0U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/**
 * @brief 执行 BMI088 1 字节 SPI 全双工交换。
 *
 * @param user port 用户上下文，本实现未使用。
 * @param tx 待发送字节。
 * @return 成功返回收到的字节；传输失败时返回 0。
 */
static uint8_t spi_xfer8(void *user, uint8_t tx)
{
    uint8_t rx = 0U;
    bsp_spi_transfer_t xfer;

    (void)user;

    xfer.tx_data = &tx;
    xfer.rx_data = &rx;
    xfer.size = 1U;
    xfer.timeout = IMU_SPI_TIMEOUT_MS;
    xfer.mode = BSP_SPI_MODE_POLL;

    if (bsp_spi_transfer(spi_handle(), &xfer) != HAL_OK) {
        return 0U;
    }

    return rx;
}

/**
 * @brief 执行 BMI088 burst SPI 全双工交换。
 *
 * @param user port 用户上下文，本实现未使用。
 * @param tx 待发送缓冲区。
 * @param rx 接收缓冲区。
 * @param len 收发长度，单位 byte。
 * @return 成功返回 0，失败返回 -1。
 */
static int spi_xfer(void *user, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    bsp_spi_transfer_t xfer;

    (void)user;

    if ((tx == NULL) || (rx == NULL) || (len == 0U)) {
        return -1;
    }

    xfer.tx_data = tx;
    xfer.rx_data = rx;
    xfer.size = len;
    xfer.timeout = IMU_SPI_TIMEOUT_MS;
    xfer.mode = BSP_SPI_MODE_POLL;

    return (bsp_spi_transfer(spi_handle(), &xfer) == HAL_OK) ? 0 : -1;
}

/**
 * @brief BMI088 port 毫秒延时回调。
 *
 * @param user port 用户上下文，本实现未使用。
 * @param ms 延时长度，单位 ms。
 */
static void delay_ms(void *user, uint32_t ms)
{
    (void)user;
    HAL_Delay(ms);
}

/**
 * @brief BMI088 port 微秒延时回调。
 *
 * @param user port 用户上下文，本实现未使用。
 * @param us 延时长度，单位 us。
 */
static void delay_us(void *user, uint32_t us)
{
    (void)user;
    bsp_dwt_delay((float)us / 1000000.0f);
}

/**
 * @brief BMI088 port 当前时间回调。
 *
 * @param user port 用户上下文，本实现未使用。
 * @return 当前运行时间，单位 s。
 */
static float time_s(void *user)
{
    (void)user;
    return bsp_dwt_get_timeline_s();
}

/**
 * @brief 板载 BMI088 port 回调表。
 */
static const bmi088_port_t imu_bmi088_port = {
    .user = NULL,
    .accel_cs = acc_cs,
    .gyro_cs = gyr_cs,
    .spi_transfer_byte = spi_xfer8,
    .spi_transfer = spi_xfer,
    .delay_ms = delay_ms,
    .delay_us = delay_us,
    .get_time_s = time_s,
};

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
imu_status_t imu_init(uint8_t calibrate)
{
    int status;

    imu_busy = 0U;
    imu_err_count = 0U;
    imu_last_status = BMI088_STATUS_OK;
    imu_last_temp_ms = HAL_GetTick();
    imu_last_ok_ms = imu_last_temp_ms;

    if (res_ok() != BMI088_STATUS_OK) {
        return IMU_ERR_IO;
    }

    (void)bsp_gpio_write(acc_cs_pin(), GPIO_PIN_SET);
    (void)bsp_gpio_write(gyr_cs_pin(), GPIO_PIN_SET);

    status = bmi088_init(&imu_bmi088,
                         &imu_bmi088_port,
                         &imu_bmi088_default_cal,
                         calibrate);
    if (status != BMI088_STATUS_OK) {
        imu_ready = 0U;
        return conv_status(status);
    }

    imu_ready = 1U;
    imu_err_count = 0U;
    imu_last_status = BMI088_STATUS_OK;
    imu_last_ok_ms = HAL_GetTick();
    imu_last_temp_ms = imu_last_ok_ms;
    update_sample();
    return IMU_OK;
}

imu_status_t imu_read(void)
{
    int status;
    uint32_t now_ms;
    uint8_t read_temp;

    if (imu_ready == 0U) {
        return IMU_ERR_NOT_READY;
    }

    if (imu_try_begin() == 0U) {
        /* 标定或另一任务正在占用 SPI。 */
        return IMU_ERR_NOT_READY;
    }

    if (res_ok() != BMI088_STATUS_OK) {
        imu_end();
        return IMU_ERR_IO;
    }

    now_ms = HAL_GetTick();
    read_temp = ((uint32_t)(now_ms - imu_last_temp_ms) >= IMU_TEMP_PERIOD_MS) ? 1U : 0U;

    status = bmi088_read_ex(&imu_bmi088, read_temp);
    if (status != BMI088_STATUS_OK) {
        /* 一次重试；连续失败由 debounce 计数处理。 */
        status = bmi088_read_ex(&imu_bmi088, read_temp);
    }

    imu_last_status = status;
    if (status == BMI088_STATUS_OK) {
        if (read_temp != 0U) {
            imu_last_temp_ms = now_ms;
        }
        imu_last_ok_ms = now_ms;
        imu_err_count = 0U;
        update_sample();
    } else {
        imu_err_count++;
        if (imu_err_count >= IMU_ERR_DEBOUNCE) {
            imu_ready = 0U;
        }
    }

    imu_end();
    return conv_status(status);
}

imu_status_t imu_calibrate(void)
{
    int status;

    if (imu_ready == 0U) {
        return IMU_ERR_NOT_READY;
    }

    if (imu_try_begin() == 0U) {
        return IMU_ERR_NOT_READY;
    }

    if (res_ok() != BMI088_STATUS_OK) {
        imu_end();
        return IMU_ERR_IO;
    }

    status = bmi088_calibrate(&imu_bmi088);
    imu_last_status = status;
    if (status != BMI088_STATUS_OK) {
        imu_end();
        return conv_status(status);
    }

    imu_bmi088_default_cal.gyro_offset[0] = imu_bmi088.gyro_offset[0];
    imu_bmi088_default_cal.gyro_offset[1] = imu_bmi088.gyro_offset[1];
    imu_bmi088_default_cal.gyro_offset[2] = imu_bmi088.gyro_offset[2];
    imu_bmi088_default_cal.accel_offset[0] = imu_bmi088.accel_offset[0];
    imu_bmi088_default_cal.accel_offset[1] = imu_bmi088.accel_offset[1];
    imu_bmi088_default_cal.accel_offset[2] = imu_bmi088.accel_offset[2];
    imu_bmi088_default_cal.g_norm = imu_bmi088.g_norm;
    imu_bmi088_default_cal.temp_when_cali = imu_bmi088.temp_when_cali;
    imu_bmi088_default_cal.accel_offset_enabled = imu_bmi088.accel_offset_enabled;
    imu_last_ok_ms = HAL_GetTick();
    imu_err_count = 0U;
    update_sample();
    imu_end();
    return IMU_OK;
}

imu_status_t imu_set_cali(const ImuCali *cali)
{
    uint32_t i;

    if (cali == NULL) {
        return IMU_ERR_ARG;
    }
    if ((isfinite(cali->g_norm) == 0) || (cali->g_norm <= 1.0f)) {
        return IMU_ERR_ARG;
    }
    for (i = 0U; i < 3U; i++) {
        if ((isfinite(cali->gyro_offset[i]) == 0) ||
            (isfinite(cali->accel_offset[i]) == 0)) {
            return IMU_ERR_ARG;
        }
    }

    if (imu_try_begin() == 0U) {
        return IMU_ERR_NOT_READY;
    }

    cali_to_bmi088(&imu_bmi088_default_cal, cali);
    if (imu_ready != 0U) {
        cali_to_dev(&imu_bmi088, cali);
        update_sample();
    }

    imu_end();
    return IMU_OK;
}

imu_status_t imu_get_cali(ImuCali *out)
{
    uint32_t primask;

    if (out == NULL) {
        return IMU_ERR_ARG;
    }

    primask = imu_critical_enter();
    if (imu_ready != 0U) {
        bmi088_to_cali(out, &imu_bmi088);
    } else {
        out->gyro_offset[0] = imu_bmi088_default_cal.gyro_offset[0];
        out->gyro_offset[1] = imu_bmi088_default_cal.gyro_offset[1];
        out->gyro_offset[2] = imu_bmi088_default_cal.gyro_offset[2];
        out->accel_offset[0] = imu_bmi088_default_cal.accel_offset[0];
        out->accel_offset[1] = imu_bmi088_default_cal.accel_offset[1];
        out->accel_offset[2] = imu_bmi088_default_cal.accel_offset[2];
        out->g_norm = imu_bmi088_default_cal.g_norm;
        out->temp_when_cali = imu_bmi088_default_cal.temp_when_cali;
        out->accel_offset_enabled = imu_bmi088_default_cal.accel_offset_enabled;
    }
    imu_critical_exit(primask);

    return IMU_OK;
}

imu_status_t imu_sample(imu_sample_t *out)
{
    uint32_t primask;

    if (out == NULL) {
        return IMU_ERR_ARG;
    }

    if (imu_ready == 0U) {
        return IMU_ERR_NOT_READY;
    }

    primask = imu_critical_enter();
    *out = imu_last_sample;
    imu_critical_exit(primask);
    return IMU_OK;
}

imu_status_t imu_get_health(imu_health_t *out)
{
    uint32_t primask;

    if (out == NULL) {
        return IMU_ERR_ARG;
    }

    primask = imu_critical_enter();
    out->err_count = imu_err_count;
    out->last_ok_ms = imu_last_ok_ms;
    out->last_status = imu_last_status;
    imu_critical_exit(primask);
    return IMU_OK;
}

float imu_cal_temp(void)
{
    return imu_bmi088_default_cal.temp_when_cali;
}
