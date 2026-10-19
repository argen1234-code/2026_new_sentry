/**
 ******************************************************************************
 * @file    bsp_gpio.hpp
 * @author  GPT
 * @version V1.0.0
 * @date    2026-05-15
 * @brief   GPIO BSP 的 C++ 编译期包装。
 *
 * @note    本文件只做 C++ 包装和模板绑定，运行时实现全部位于 Src/bsp_gpio.c。
 * @note    C 文件应包含 Src/bsp_gpio.h；C++ 文件可包含本文件。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef PRINTK_BSP_GPIO_HPP
#define PRINTK_BSP_GPIO_HPP

#include "Src/bsp_gpio.h"

/* ==========================================================================
 * C++ 零成本包装
 * --------------------------------------------------------------------------
 * hpp 只为 C BSP 接口提供命名空间内 inline 转发，不引入运行时抽象。
 * ========================================================================== */
namespace Bsp::Gpio {

/** @brief GPIO 标识。 */
enum class Id {
    Pc13 = BSP_GPIO_ID_PC13,                  /**< PC13。 */
    Pc14 = BSP_GPIO_ID_PC14,                  /**< PC14。 */
    PwrEn5v = BSP_GPIO_ID_PWR_EN_5V,          /**< 5V 电源使能。 */
    AccCs = BSP_GPIO_ID_ACC_CS,               /**< 加速度计片选。 */
    GyroCs = BSP_GPIO_ID_GYRO_CS,             /**< 陀螺仪片选。 */
    CamTrig = BSP_GPIO_ID_CAM_TRIG,           /**< 相机触发。 */
    ShootDetect = BSP_GPIO_ID_SHOOT_DETECT,   /**< 发射检测。 */
    Cs = BSP_GPIO_ID_CS,                      /**< 通用片选。 */
    AccInt = BSP_GPIO_ID_ACC_INT,             /**< 加速度计中断。 */
    GyroInt = BSP_GPIO_ID_GYRO_INT,           /**< 陀螺仪中断。 */
    Clk = BSP_GPIO_ID_CLK,                    /**< 时钟引脚。 */
};

/** @brief GPIO 引脚描述。 */
using Pin = bsp_gpio_pin_t;

/** @brief GPIO 外部中断回调。 */
using ExtiCallback = bsp_gpio_exti_callback_t;

/* ==========================================================================
 * inline 转发接口
 * --------------------------------------------------------------------------
 * 所有便捷函数都在编译期展开到 C API，不使用虚函数、RTTI 或动态分派。
 * ========================================================================== */
/** @brief 初始化 GPIO BSP。 */
inline void init()
{
    bsp_gpio_init();
}

/** @brief 获取 GPIO 引脚描述。 */
inline const Pin *pin(Id id)
{
    return bsp_gpio_pin(static_cast<bsp_gpio_id_t>(id));
}

/** @brief 判断 GPIO 引脚是否由 BSP 管理。 */
inline bool supported(const Pin *pin)
{
    return bsp_gpio_is_supported(pin) != 0U;
}

/** @brief 读取 GPIO 电平。 */
inline GPIO_PinState read(const Pin *pin)
{
    return bsp_gpio_read(pin);
}

/** @brief 写入 GPIO 电平。 */
inline HAL_StatusTypeDef write(const Pin *pin, GPIO_PinState state)
{
    return bsp_gpio_write(pin, state);
}

/** @brief 翻转 GPIO 电平。 */
inline HAL_StatusTypeDef toggle(const Pin *pin)
{
    return bsp_gpio_toggle(pin);
}

/** @brief 配置 GPIO 端口。 */
inline HAL_StatusTypeDef config(GPIO_TypeDef *port, const GPIO_InitTypeDef &cfg)
{
    return bsp_gpio_config(port, &cfg);
}

/** @brief 反初始化 GPIO。 */
inline HAL_StatusTypeDef deinit(const Pin *pin)
{
    return bsp_gpio_deinit(pin);
}

/** @brief 锁定 GPIO 配置。 */
inline HAL_StatusTypeDef lock(const Pin *pin)
{
    return bsp_gpio_lock(pin);
}

/** @brief 转发 GPIO 外部中断。 */
inline void irq(uint16_t pin_mask)
{
    bsp_gpio_exti_isr(pin_mask);
}

/** @brief 绑定 GPIO 外部中断回调。 */
inline HAL_StatusTypeDef on_exti(const Pin *pin, ExtiCallback callback, void *user)
{
    return bsp_gpio_exti_bind(pin, callback, user);
}

/**
 * @brief GPIO 编译期端口。
 *
 * @tparam PinId GPIO 标识。
 */
template<Id PinId>
struct Port {
    /** @brief 获取 GPIO 引脚描述。 */
    static const Pin *pin()
    {
        return Bsp::Gpio::pin(PinId);
    }

    /** @brief 读取 GPIO 电平。 */
    static GPIO_PinState read()
    {
        return Bsp::Gpio::read(pin());
    }

    /** @brief 写入 GPIO 电平。 */
    static HAL_StatusTypeDef write(GPIO_PinState state)
    {
        return Bsp::Gpio::write(pin(), state);
    }

    /** @brief 翻转 GPIO 电平。 */
    static HAL_StatusTypeDef toggle()
    {
        return Bsp::Gpio::toggle(pin());
    }

    /** @brief 绑定 GPIO 外部中断回调。 */
    static HAL_StatusTypeDef on_exti(ExtiCallback callback, void *user)
    {
        return Bsp::Gpio::on_exti(pin(), callback, user);
    }
};

} // namespace Bsp::Gpio

#endif // PRINTK_BSP_GPIO_HPP
