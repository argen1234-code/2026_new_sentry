/**
 ******************************************************************************
 * @file    bmi088.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   BMI088 IMU device driver 实现。
 *
 * @note    本文件只处理 BMI088 寄存器访问、初始化、读数和标定流程。
 *          SPI、片选、延时和时间基准均由 bmi088_port_t 注入。
 * @note    板级片选 GPIO、SPI 实例和实测标定参数应放在 board/device
 *          适配层，不应写入通用 device driver。
 * @note    差异: 与旧版 BMI088driver 相比，本文件保留 BMI088 SPI 器件协议、
 *          初始化寄存器序列、原始数据解码和标定主流程；不再直接依赖 HAL、
 *          全局 BMI088 对象、DWT 或板级默认零偏。
 * @note    差异: 旧版的部分边界行为被改为显式错误码返回，包括 port 参数错误、
 *          标定超时、传输失败和未检测到传感器。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "imu_bmi088.h"
#include "imu_bmi088_reg.h"

#include <math.h>
#include <string.h>

/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置设备驱动的协议常量、默认参数和私有类型，硬件绑定来自 Robot board 配置。
 * ========================================================================== */

/**
 * @brief 获取静态数组元素数量。
 */
#define ARRAY_SIZE(array) ((uint32_t)(sizeof(array) / sizeof((array)[0])))

/**
 * @brief BMI088 当前驱动用到的最大 burst 读取数据长度。
 */
#define MAX_BURST_READ_LEN 8U

/**
 * @brief 片选有效电平。
 */
#define CS_LOW  0U

/**
 * @brief 片选释放电平。
 */
#define CS_HIGH 1U

/**
 * @brief BMI088 初始化寄存器配置项。
 */
typedef struct {
    uint8_t reg;
    uint8_t val;
    uint8_t err;
} reg_cfg_t;

/**
 * @brief 加速度计初始化寄存器写入序列。
 *
 * @note 差异: 寄存器、写入值和错误码与旧版 `write_BMI088_accel_reg_data_error`
 *       保持一致，仅改为结构体表，避免使用下标 0/1/2 表达字段含义。
 */
static const reg_cfg_t acc_init_seq[] = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_6G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}
};

/**
 * @brief 陀螺仪初始化寄存器写入序列。
 *
 * @note 差异: 寄存器、写入值和错误码与旧版 `write_BMI088_gyro_reg_data_error`
 *       保持一致；新版校验失败时使用陀螺仪错误码表，修正旧版误用加速度计
 *       错误码表的问题。
 */
static const reg_cfg_t gyr_init_seq[] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}
};

/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责帧格式、数值范围、在线时间和底层返回值检查。
 * ========================================================================== */

/**
 * @brief 写入当前片选设备的单个寄存器。
 */
static void write_reg(bmi088_t *dev, uint8_t reg, uint8_t val);
/**
 * @brief 读取当前片选设备的单个寄存器。
 */
static uint8_t read_reg(bmi088_t *dev, uint8_t reg);
/* ==========================================================================
 * 收发、解析与打包
 * --------------------------------------------------------------------------
 * 这里把字节流或寄存器数据转换成设备事实；控制含义由 Component/Input 或 Apps 解释。
 * ========================================================================== */

/**
 * @brief 写入加速度计寄存器。
 */
static void acc_write(bmi088_t *dev, uint8_t reg, uint8_t val);
/**
 * @brief 读取加速度计单个寄存器。
 */
static uint8_t acc_read_reg(bmi088_t *dev, uint8_t reg);
/**
 * @brief 读取加速度计连续寄存器。
 */
static int acc_read(bmi088_t *dev, uint8_t reg, uint8_t *buf, uint8_t len);
/**
 * @brief 写入陀螺仪寄存器。
 */
static void gyr_write(bmi088_t *dev, uint8_t reg, uint8_t val);
/**
 * @brief 读取陀螺仪单个寄存器。
 */
static uint8_t gyr_read_reg(bmi088_t *dev, uint8_t reg);
/**
 * @brief 读取陀螺仪连续寄存器。
 */
static int gyr_read(bmi088_t *dev, uint8_t reg, uint8_t *buf, uint8_t len);

/**
 * @brief 检查 port 回调是否满足驱动运行的最低要求。
 */
static int port_ok(const bmi088_port_t *port)
{
    if ((port == NULL) ||
        (port->accel_cs == NULL) ||
        (port->gyro_cs == NULL) ||
        (port->spi_transfer_byte == NULL) ||
        (port->delay_ms == NULL) ||
        (port->delay_us == NULL) ||
        (port->get_time_s == NULL)) {                                                                                                                                                                                                                  
        return BMI088_STATUS_PORT_ERROR;
    }

    return BMI088_STATUS_OK;
}

/**
 * @brief 返回驱动内置空标定参数。
 *
 * @note 差异: 旧版在未重新标定时会写入某块板子的实测零偏；新版 device driver
 *       只提供零偏为 0、重力模长为 9.81 的通用默认值，板级标定应由调用方传入。
 */
static bmi088_calibration_t default_cal(void)
{
    bmi088_calibration_t calibration;

    calibration.gyro_offset[0] = 0.0f;
    calibration.gyro_offset[1] = 0.0f;
    calibration.gyro_offset[2] = 0.0f;
    calibration.accel_offset[0] = 0.0f;
    calibration.accel_offset[1] = 0.0f;
    calibration.accel_offset[2] = 0.0f;
    calibration.g_norm = BMI088_DEFAULT_G_NORM;
    calibration.temp_when_cali = 0.0f;
    calibration.accel_offset_enabled = 0U;

    return calibration;
}

/**
 * @brief 将标定参数写入设备对象。
 *
 * @note 差异: 新版会防御 g_norm 非法值，避免旧式板级标定参数错误导致除零。
 */
static void apply_cal(bmi088_t *dev, const bmi088_calibration_t *calibration)
{
    dev->gyro_offset[0] = calibration->gyro_offset[0];
    dev->gyro_offset[1] = calibration->gyro_offset[1];
    dev->gyro_offset[2] = calibration->gyro_offset[2];
    dev->accel_offset[0] = calibration->accel_offset[0];
    dev->accel_offset[1] = calibration->accel_offset[1];
    dev->accel_offset[2] = calibration->accel_offset[2];
    dev->g_norm = (calibration->g_norm > 1.0f) ? calibration->g_norm : BMI088_DEFAULT_G_NORM;
    dev->temp_when_cali = calibration->temp_when_cali;
    dev->accel_offset_enabled = calibration->accel_offset_enabled;
    dev->accel_scale = 9.81f / dev->g_norm;
}

/**
 * @brief 通过 port 执行 1 字节 SPI 交换。
 */
static uint8_t xfer8(bmi088_t *dev, uint8_t tx)
{
    return dev->port.spi_transfer_byte(dev->port.user, tx);
}

/**
 * @brief 通过 port 执行一次 burst SPI 交换。
 */
static int xfer(bmi088_t *dev, const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (dev->port.spi_transfer != NULL) {
        return (dev->port.spi_transfer(dev->port.user, tx, rx, len) == 0) ? BMI088_STATUS_OK : BMI088_STATUS_PORT_ERROR;
    }

    return BMI088_STATUS_PORT_ERROR;
}

/**
 * @brief 设置加速度计片选电平。
 */
static void acc_cs(bmi088_t *dev, uint8_t level)
{
    dev->port.accel_cs(dev->port.user, level);
}

/**
 * @brief 设置陀螺仪片选电平。
 */
static void gyr_cs(bmi088_t *dev, uint8_t level)
{
    dev->port.gyro_cs(dev->port.user, level);
}

/**
 * @brief 通过 port 延时指定毫秒数。
 */
static void delay_ms(bmi088_t *dev, uint32_t ms)
{
    dev->port.delay_ms(dev->port.user, ms);
}

/**
 * @brief 通过 port 延时指定微秒数。
 */
static void delay_us(bmi088_t *dev, uint32_t us)
{
    dev->port.delay_us(dev->port.user, us);
}

/**
 * @brief 获取 port 提供的当前时间，单位为秒。
 */
static float time_s(bmi088_t *dev)
{
    return dev->port.get_time_s(dev->port.user);
}

/**
 * @brief 将 BMI088 温度寄存器字节解码为有符号原始值。
 */
static int16_t temp_raw(const uint8_t *buf)
{
    int16_t raw_temp = (int16_t)(((int16_t)buf[0] << 3) | ((int16_t)buf[1] >> 5));

    if (raw_temp > 1023) {
        raw_temp = (int16_t)(raw_temp - 2048);
    }

    return raw_temp;
}

/**
 * @brief 设置当前驱动默认量程对应的灵敏度。
 */
static void set_default_sens(bmi088_t *dev)
{
    dev->accel_sen = BMI088_ACCEL_6G_SEN;
    dev->gyro_sen = BMI088_GYRO_2000_SEN;
}

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只提供初始化、喂入、查询和发送入口，不保留旧 task/global 兼容接口。
 * ========================================================================== */
int bmi088_init(bmi088_t *dev, const bmi088_port_t *port, const bmi088_calibration_t *calibration, uint8_t calibrate)
{
    bmi088_calibration_t default_calibration;
    bmi088_port_t port_copy;
    int status;

    if ((dev == NULL) || (port == NULL)) {
        return BMI088_STATUS_INVALID_ARG;
    }

    status = port_ok(port);
    if (status != BMI088_STATUS_OK) {
        return status;
    }

    port_copy = *port;
    (void)memset(dev, 0, sizeof(*dev));
    dev->port = port_copy;
    set_default_sens(dev);

    /* @diff: 旧版直接保存 HAL SPI 句柄和全局 BMI088 对象；新版复制 port 后再清零设备对象，
     *        避免调用方传入 &dev->port 时被 memset 清掉。 */
    default_calibration = default_cal();
    apply_cal(dev, (calibration != NULL) ? calibration : &default_calibration);

    status = bmi088_accel_init(dev);
    status |= bmi088_gyro_init(dev);

    if (status != BMI088_NO_ERROR) {
        return status;
    }

    if (calibrate != 0U) {
        /* @diff: 旧版 BMI088_Init 会在外层死循环等待初始化成功；新版返回错误码，
         *        是否重试由 board/device 适配层决定。 */
        return bmi088_calibrate(dev);
    }

    return BMI088_STATUS_OK;
}

int bmi088_calibrate(bmi088_t *dev)
{
    bmi088_calibration_t default_calibration;
    uint8_t buf[8] = {0};
    int16_t raw_temp;
    float gyro_diff[3] = {0.0f, 0.0f, 0.0f};
    float gyro_max[3] = {0.0f, 0.0f, 0.0f};
    float gyro_min[3] = {0.0f, 0.0f, 0.0f};
    float g_norm_diff = 0.0f;
    float g_norm_temp;
    float g_norm_max = 0.0f;
    float g_norm_min = 0.0f;
    float start_time;
    uint32_t sample_count;
    uint32_t i;
    int status;
    uint8_t j;

    if (dev == NULL) {
        return BMI088_STATUS_INVALID_ARG;
    }

    if (port_ok(&dev->port) != BMI088_STATUS_OK) {
        return BMI088_STATUS_PORT_ERROR;
    }

    default_calibration = default_cal();
    start_time = time_s(dev);

    do {
        if ((time_s(dev) - start_time) > BMI088_CALI_TIMEOUT_S) {
            apply_cal(dev, &default_calibration);
            /* @diff: 旧版标定超时后恢复默认零偏并返回成功；新版显式返回超时错误，
             *        避免上层误认为本次标定有效。 */
            return BMI088_STATUS_CALI_TIMEOUT;
        }

        delay_us(dev, 5000U);
        sample_count = 0U;
        dev->g_norm = 0.0f;
        dev->gyro_offset[0] = 0.0f;
        dev->gyro_offset[1] = 0.0f;
        dev->gyro_offset[2] = 0.0f;

        for (i = 0U; i < BMI088_CALI_GYRO_TIMES; ++i) {
            status = acc_read(dev, BMI088_ACCEL_XOUT_L, buf, 6U);
            if (status != BMI088_STATUS_OK) {
                return status;
            }

            raw_temp = (int16_t)(((int16_t)buf[1] << 8) | buf[0]);
            dev->accel[0] = raw_temp * dev->accel_sen;
            raw_temp = (int16_t)(((int16_t)buf[3] << 8) | buf[2]);
            dev->accel[1] = raw_temp * dev->accel_sen;
            raw_temp = (int16_t)(((int16_t)buf[5] << 8) | buf[4]);
            dev->accel[2] = raw_temp * dev->accel_sen;

            g_norm_temp = sqrtf(
                dev->accel[0] * dev->accel[0] +
                dev->accel[1] * dev->accel[1] +
                dev->accel[2] * dev->accel[2]);
            dev->g_norm += g_norm_temp;
            ++sample_count;

            status = gyr_read(dev, BMI088_GYRO_CHIP_ID, buf, 8U);
            if (status != BMI088_STATUS_OK) {
                return status;
            }

            if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
                raw_temp = (int16_t)(((int16_t)buf[3] << 8) | buf[2]);
                dev->gyro[0] = raw_temp * dev->gyro_sen;
                dev->gyro_offset[0] += dev->gyro[0];

                raw_temp = (int16_t)(((int16_t)buf[5] << 8) | buf[4]);
                dev->gyro[1] = raw_temp * dev->gyro_sen;
                dev->gyro_offset[1] += dev->gyro[1];

                raw_temp = (int16_t)(((int16_t)buf[7] << 8) | buf[6]);
                dev->gyro[2] = raw_temp * dev->gyro_sen;
                dev->gyro_offset[2] += dev->gyro[2];
            }

            if (i == 0U) {
                g_norm_max = g_norm_temp;
                g_norm_min = g_norm_temp;
                for (j = 0U; j < 3U; ++j) {
                    gyro_max[j] = dev->gyro[j];
                    gyro_min[j] = dev->gyro[j];
                }
            } else {
                if (g_norm_temp > g_norm_max) {
                    g_norm_max = g_norm_temp;
                }
                if (g_norm_temp < g_norm_min) {
                    g_norm_min = g_norm_temp;
                }
                for (j = 0U; j < 3U; ++j) {
                    if (dev->gyro[j] > gyro_max[j]) {
                        gyro_max[j] = dev->gyro[j];
                    }
                    if (dev->gyro[j] < gyro_min[j]) {
                        gyro_min[j] = dev->gyro[j];
                    }
                }
            }

            g_norm_diff = g_norm_max - g_norm_min;
            for (j = 0U; j < 3U; ++j) {
                gyro_diff[j] = gyro_max[j] - gyro_min[j];
            }

            if ((g_norm_diff > BMI088_CALI_G_NORM_DIFF_THRES) ||
                (gyro_diff[0] > BMI088_CALI_GYRO_DIFF_THRES) ||
                (gyro_diff[1] > BMI088_CALI_GYRO_DIFF_THRES) ||
                (gyro_diff[2] > BMI088_CALI_GYRO_DIFF_THRES)) {
                break;
            }

            delay_us(dev, 500U);
        }

        if (sample_count == 0U) {
            apply_cal(dev, &default_calibration);
            return BMI088_STATUS_CALI_UNSTABLE;
        }

        /* @diff: 旧版即使中途 break 也固定除以 6000；新版按实际有效样本数求平均。 */
        dev->g_norm /= (float)sample_count;
        for (j = 0U; j < 3U; ++j) {
            dev->gyro_offset[j] /= (float)sample_count;
        }

        status = acc_read(dev, BMI088_TEMP_M, buf, 2U);
        if (status != BMI088_STATUS_OK) {
            return status;
        }

        raw_temp = temp_raw(buf);
        dev->temp_when_cali = raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
    } while ((g_norm_diff > BMI088_CALI_G_NORM_DIFF_THRES) ||
             (fabsf(dev->g_norm - 9.8f) > BMI088_CALI_G_NORM_ABS_THRES) ||
             (gyro_diff[0] > BMI088_CALI_GYRO_DIFF_THRES) ||
             (gyro_diff[1] > BMI088_CALI_GYRO_DIFF_THRES) ||
             (gyro_diff[2] > BMI088_CALI_GYRO_DIFF_THRES) ||
             (fabsf(dev->gyro_offset[0]) > BMI088_CALI_GYRO_OFFSET_ABS_THRES) ||
             (fabsf(dev->gyro_offset[1]) > BMI088_CALI_GYRO_OFFSET_ABS_THRES) ||
             (fabsf(dev->gyro_offset[2]) > BMI088_CALI_GYRO_OFFSET_ABS_THRES));

    if (dev->g_norm <= 1.0f) {
        apply_cal(dev, &default_calibration);
        return BMI088_STATUS_CALI_UNSTABLE;
    }

    dev->accel_scale = 9.81f / dev->g_norm;

    for (i = 0U; i < 3U; ++i) {
        dev->accel_offset[i] = 0.0f;
    }

    for (i = 0U; i < BMI088_CALI_ACCEL_TIMES; ++i) {
        status = acc_read(dev, BMI088_ACCEL_XOUT_L, buf, 6U);
        if (status != BMI088_STATUS_OK) {
            return status;
        }

        raw_temp = (int16_t)(((int16_t)buf[1] << 8) | buf[0]);
        dev->accel[0] = raw_temp * dev->accel_sen * dev->accel_scale;
        dev->accel_offset[0] = dev->accel_offset[0] * 0.8f + dev->accel[0] * 0.2f;

        raw_temp = (int16_t)(((int16_t)buf[3] << 8) | buf[2]);
        dev->accel[1] = raw_temp * dev->accel_sen * dev->accel_scale;
        dev->accel_offset[1] = dev->accel_offset[1] * 0.8f + dev->accel[1] * 0.2f;

        raw_temp = (int16_t)(((int16_t)buf[5] << 8) | buf[4]);
        dev->accel[2] = raw_temp * dev->accel_sen * dev->accel_scale;
        dev->accel_offset[2] = dev->accel_offset[2] * 0.8f + dev->accel[2] * 0.2f;

        delay_us(dev, 500U);
    }

    return BMI088_STATUS_OK;
}

int bmi088_accel_init(bmi088_t *dev)
{
    uint8_t res = 0U;
    uint32_t i;
    int error = BMI088_NO_ERROR;

    if (dev == NULL) {
        return BMI088_STATUS_INVALID_ARG;
    }

    if (port_ok(&dev->port) != BMI088_STATUS_OK) {
        return BMI088_STATUS_PORT_ERROR;
    }

    res = acc_read_reg(dev, BMI088_ACC_CHIP_ID);
    delay_ms(dev, 1U);
    res = acc_read_reg(dev, BMI088_ACC_CHIP_ID);
    delay_ms(dev, 1U);

    acc_write(dev, BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    delay_ms(dev, BMI088_LONG_DELAY_TIME);

    res = acc_read_reg(dev, BMI088_ACC_CHIP_ID);
    delay_ms(dev, 1U);
    res = acc_read_reg(dev, BMI088_ACC_CHIP_ID);
    delay_ms(dev, 1U);

    if (res != BMI088_ACC_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    for (i = 0U; i < ARRAY_SIZE(acc_init_seq); ++i) {
        acc_write(dev, acc_init_seq[i].reg, acc_init_seq[i].val);
        delay_ms(dev, 1U);

        res = acc_read_reg(dev, acc_init_seq[i].reg);
        delay_ms(dev, 1U);

        if (res != acc_init_seq[i].val) {
            error |= acc_init_seq[i].err;
        }
    }

    return error;
}

int bmi088_gyro_init(bmi088_t *dev)
{
    uint8_t res = 0U;
    uint32_t i;
    int error = BMI088_NO_ERROR;

    if (dev == NULL) {
        return BMI088_STATUS_INVALID_ARG;
    }

    if (port_ok(&dev->port) != BMI088_STATUS_OK) {
        return BMI088_STATUS_PORT_ERROR;
    }

    res = gyr_read_reg(dev, BMI088_GYRO_CHIP_ID);
    delay_ms(dev, 1U);
    res = gyr_read_reg(dev, BMI088_GYRO_CHIP_ID);
    delay_ms(dev, 1U);

    gyr_write(dev, BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    delay_ms(dev, BMI088_LONG_DELAY_TIME);

    res = gyr_read_reg(dev, BMI088_GYRO_CHIP_ID);
    delay_ms(dev, 1U);
    res = gyr_read_reg(dev, BMI088_GYRO_CHIP_ID);
    delay_ms(dev, 1U);

    if (res != BMI088_GYRO_CHIP_ID_VALUE) {
        return BMI088_NO_SENSOR;
    }

    for (i = 0U; i < ARRAY_SIZE(gyr_init_seq); ++i) {
        gyr_write(dev, gyr_init_seq[i].reg, gyr_init_seq[i].val);
        delay_ms(dev, 1U);

        res = gyr_read_reg(dev, gyr_init_seq[i].reg);
        delay_ms(dev, 1U);

        if (res != gyr_init_seq[i].val) {
            /* @diff: 旧版此处误用加速度计错误码表；新版使用 gyr_init_seq 对应错误码。 */
            error |= gyr_init_seq[i].err;
        }
    }

    return error;
}

int bmi088_read(bmi088_t *dev)
{
    uint8_t buf[8] = {0};
    int16_t raw_temp;
    int status;

    if (dev == NULL) {
        return BMI088_STATUS_INVALID_ARG;
    }

    if (port_ok(&dev->port) != BMI088_STATUS_OK) {
        return BMI088_STATUS_PORT_ERROR;
    }

    status = acc_read(dev, BMI088_ACCEL_XOUT_L, buf, 6U);
    if (status != BMI088_STATUS_OK) {
        return status;
    }

    raw_temp = (int16_t)(((int16_t)buf[1] << 8) | buf[0]);
    dev->accel[0] = raw_temp * dev->accel_sen * dev->accel_scale;
    if (dev->accel_offset_enabled != 0U) {
        dev->accel[0] -= dev->accel_offset[0];
    }

    raw_temp = (int16_t)(((int16_t)buf[3] << 8) | buf[2]);
    dev->accel[1] = raw_temp * dev->accel_sen * dev->accel_scale;
    if (dev->accel_offset_enabled != 0U) {
        dev->accel[1] -= dev->accel_offset[1];
    }

    raw_temp = (int16_t)(((int16_t)buf[5] << 8) | buf[4]);
    dev->accel[2] = raw_temp * dev->accel_sen * dev->accel_scale;

    status = gyr_read(dev, BMI088_GYRO_CHIP_ID, buf, 8U);
    if (status != BMI088_STATUS_OK) {
        return status;
    }

    if (buf[0] != BMI088_GYRO_CHIP_ID_VALUE) {
        /* @diff: 旧版 BMI088_Read 为 void，gyro ID 异常时只跳过 gyro 更新；
         *        新版仍更新温度并返回 BMI088_NO_SENSOR 供上层感知异常。 */
        status = acc_read(dev, BMI088_TEMP_M, buf, 2U);
        if (status != BMI088_STATUS_OK) {
            return status;
        }

        raw_temp = temp_raw(buf);
        dev->temperature = raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
        return BMI088_NO_SENSOR;
    }

    raw_temp = (int16_t)(((int16_t)buf[3] << 8) | buf[2]);
    dev->gyro[0] = raw_temp * dev->gyro_sen - dev->gyro_offset[0];

    raw_temp = (int16_t)(((int16_t)buf[5] << 8) | buf[4]);
    dev->gyro[1] = raw_temp * dev->gyro_sen - dev->gyro_offset[1];

    raw_temp = (int16_t)(((int16_t)buf[7] << 8) | buf[6]);
    dev->gyro[2] = raw_temp * dev->gyro_sen - dev->gyro_offset[2];

    status = acc_read(dev, BMI088_TEMP_M, buf, 2U);
    if (status != BMI088_STATUS_OK) {
        return status;
    }

    raw_temp = temp_raw(buf);
    dev->temperature = raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    return BMI088_STATUS_OK;
}

/**
 * @brief 写入当前片选设备的单个寄存器。
 */
static void write_reg(bmi088_t *dev, uint8_t reg, uint8_t val)
{
    xfer8(dev, reg);
    xfer8(dev, val);
}

/**
 * @brief 读取当前片选设备的单个寄存器。
 */
static uint8_t read_reg(bmi088_t *dev, uint8_t reg)
{
    xfer8(dev, (uint8_t)(reg | 0x80U));

    return xfer8(dev, 0x55U);
}

/**
 * @brief 写入加速度计寄存器。
 */
static void acc_write(bmi088_t *dev, uint8_t reg, uint8_t val)
{
    acc_cs(dev, CS_LOW);
    write_reg(dev, reg, val);
    acc_cs(dev, CS_HIGH);
}

/**
 * @brief 读取加速度计单个寄存器。
 */
static uint8_t acc_read_reg(bmi088_t *dev, uint8_t reg)
{
    uint8_t val;

    acc_cs(dev, CS_LOW);
    (void)xfer8(dev, (uint8_t)(reg | 0x80U));
    (void)xfer8(dev, 0x55U);
    val = xfer8(dev, 0x55U);
    acc_cs(dev, CS_HIGH);

    return val;
}

/**
 * @brief 写入陀螺仪寄存器。
 */
static void gyr_write(bmi088_t *dev, uint8_t reg, uint8_t val)
{
    gyr_cs(dev, CS_LOW);
    write_reg(dev, reg, val);
    gyr_cs(dev, CS_HIGH);
}

/**
 * @brief 读取陀螺仪单个寄存器。
 */
static uint8_t gyr_read_reg(bmi088_t *dev, uint8_t reg)
{
    uint8_t val;

    gyr_cs(dev, CS_LOW);
    val = read_reg(dev, reg);
    gyr_cs(dev, CS_HIGH);

    return val;
}

/**
 * @brief 读取加速度计连续寄存器。
 */
static int acc_read(bmi088_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx[MAX_BURST_READ_LEN + 2U];
    uint8_t rx[MAX_BURST_READ_LEN + 2U];
    uint8_t i;
    int status;

    if ((dev == NULL) || (buf == NULL)) {
        return BMI088_STATUS_INVALID_ARG;
    }

    acc_cs(dev, CS_LOW);

    if ((dev->port.spi_transfer != NULL) && (len <= MAX_BURST_READ_LEN)) {
        tx[0] = (uint8_t)(reg | 0x80U);
        tx[1] = 0x55U;
        rx[0] = 0U;
        rx[1] = 0U;
        for (i = 0U; i < len; ++i) {
            tx[i + 2U] = 0x55U;
            rx[i + 2U] = 0U;
        }

        status = xfer(dev, tx, rx, (uint16_t)(len + 2U));
        if (status != BMI088_STATUS_OK) {
            acc_cs(dev, CS_HIGH);
            return status;
        }

        for (i = 0U; i < len; ++i) {
            buf[i] = rx[i + 2U];
        }
    } else {
        /* @diff: 旧版 accel burst read 通过“再次发送 reg|0x80”隐式充当 dummy；
         *        新版显式发送 dummy byte，器件协议等价但更不容易被误改。 */
        (void)xfer8(dev, (uint8_t)(reg | 0x80U));
        (void)xfer8(dev, 0x55U);

        while (len != 0U) {
            *buf = xfer8(dev, 0x55U);
            ++buf;
            --len;
        }
    }

    acc_cs(dev, CS_HIGH);

    return BMI088_STATUS_OK;
}

/**
 * @brief 读取陀螺仪连续寄存器。
 */
static int gyr_read(bmi088_t *dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx[MAX_BURST_READ_LEN + 1U];
    uint8_t rx[MAX_BURST_READ_LEN + 1U];
    uint8_t i;
    int status;

    if ((dev == NULL) || (buf == NULL)) {
        return BMI088_STATUS_INVALID_ARG;
    }

    gyr_cs(dev, CS_LOW);

    if ((dev->port.spi_transfer != NULL) && (len <= MAX_BURST_READ_LEN)) {
        tx[0] = (uint8_t)(reg | 0x80U);
        rx[0] = 0U;
        for (i = 0U; i < len; ++i) {
            tx[i + 1U] = 0x55U;
            rx[i + 1U] = 0U;
        }

        status = xfer(dev, tx, rx, (uint16_t)(len + 1U));
        if (status != BMI088_STATUS_OK) {
            gyr_cs(dev, CS_HIGH);
            return status;
        }

        for (i = 0U; i < len; ++i) {
            buf[i] = rx[i + 1U];
        }
    } else {
        (void)xfer8(dev, (uint8_t)(reg | 0x80U));

        while (len != 0U) {
            *buf = xfer8(dev, 0x55U);
            ++buf;
            --len;
        }
    }

    gyr_cs(dev, CS_HIGH);

    return BMI088_STATUS_OK;
}
