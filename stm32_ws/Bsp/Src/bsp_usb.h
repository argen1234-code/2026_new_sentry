/**
 ******************************************************************************
 * @file    bsp_usb.h
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   USB OTG PCD BSP 抽象接口。
 * @note    差异: 本文件只封装 USB PCD 控制器句柄、启停、连接控制和 HAL 事件分发；
 *          不包含 CDC/HID、示波器数据帧、打印或上位机协议逻辑。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef PRINTK_BSP_USB_H
#define PRINTK_BSP_USB_H

#include "usbd_conf.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 配置宏与类型
 * --------------------------------------------------------------------------
 * BSP 头文件描述板级外设能力、ID 和轻量配置，不描述设备用途或机器人装配语义。
 * ========================================================================== */

/**
 * @brief 板上可由 BSP 管理的 USB PCD 标识。
 *
 * 新增或删除 USB PCD 实例时，需要同步维护本枚举和 bsp_usb.c 中的 bsp_usb_slots[]。
 */
typedef enum
{
    BSP_USB_ID_HS = 0,
    BSP_USB_ID_COUNT
} bsp_usb_id_t;

/**
 * @brief USB PCD 事件类型。
 */
typedef enum
{
    BSP_USB_EVENT_CONNECT = 0,    /**< USB 连接事件。 */
    BSP_USB_EVENT_DISCONNECT,     /**< USB 断开事件。 */
    BSP_USB_EVENT_RESET,          /**< USB reset 事件。 */
    BSP_USB_EVENT_SUSPEND,        /**< USB suspend 事件。 */
    BSP_USB_EVENT_RESUME,         /**< USB resume 事件。 */
    BSP_USB_EVENT_SOF,            /**< USB SOF 事件。 */
    BSP_USB_EVENT_COUNT
} bsp_usb_event_t;

/* ==========================================================================
 * 公开接口
 * --------------------------------------------------------------------------
 * 接口按初始化、查询、收发和回调辅助排列，上层不应直接访问 CubeMX 句柄。
 * ========================================================================== */

/**
 * @brief USB PCD 事件回调类型。
 * @param hpcd 触发事件的 HAL PCD 句柄。
 * @param user 用户上下文。
 */
typedef void (*bsp_usb_event_callback_t)(PCD_HandleTypeDef *hpcd, void *user);

/**
 * @brief 初始化 USB BSP 内部状态。
 * @note 该函数不替代 CubeMX 生成的 MX_USB_DEVICE_Init()。
 */
void bsp_usb_init(void);

/**
 * @brief 获取指定 USB PCD 句柄。
 * @param id USB PCD 标识。
 * @return 匹配的 HAL PCD 句柄；未找到时返回 NULL。
 */
PCD_HandleTypeDef *bsp_usb_handle(bsp_usb_id_t id);

/**
 * @brief 根据 USB OTG 外设实例反查 HAL PCD 句柄。
 * @param instance USB OTG 外设实例地址。
 * @return 匹配的 HAL PCD 句柄；未找到时返回 NULL。
 */
PCD_HandleTypeDef *bsp_usb_from_instance(USB_OTG_GlobalTypeDef *instance);

/**
 * @brief 判断 PCD 句柄是否由本 BSP 管理。
 * @param hpcd HAL PCD 句柄。
 * @return 1 表示受管；0 表示不受管。
 */
uint8_t bsp_usb_is_supported(PCD_HandleTypeDef *hpcd);

/**
 * @brief 启动 USB PCD。
 * @param hpcd HAL PCD 句柄。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_usb_start(PCD_HandleTypeDef *hpcd);

/**
 * @brief 停止 USB PCD。
 * @param hpcd HAL PCD 句柄。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_usb_stop(PCD_HandleTypeDef *hpcd);

/**
 * @brief 使能 USB device 连接。
 * @param hpcd HAL PCD 句柄。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_usb_dev_connect(PCD_HandleTypeDef *hpcd);

/**
 * @brief 断开 USB device 连接。
 * @param hpcd HAL PCD 句柄。
 * @return HAL 执行状态。
 */
HAL_StatusTypeDef bsp_usb_dev_disconnect(PCD_HandleTypeDef *hpcd);

/**
 * @brief 绑定 USB PCD 事件回调。
 * @param hpcd HAL PCD 句柄。
 * @param event USB PCD 事件类型。
 * @param callback 事件回调；传入 NULL 可清空该事件绑定。
 * @param user 用户上下文。
 * @return HAL_OK 表示绑定成功；HAL_ERROR 表示参数非法或句柄不受管。
 */
HAL_StatusTypeDef bsp_usb_on(PCD_HandleTypeDef *hpcd,
                             bsp_usb_event_t event,
                             bsp_usb_event_callback_t callback,
                             void *user);

#ifdef __cplusplus
}
#endif

#endif // PRINTK_BSP_USB_H
