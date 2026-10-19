/**
 ******************************************************************************
 * @file    bsp_usb.c
 * @author  GPT
 * @version V1.2.0
 * @date    2026-04-27
 * @brief   USB OTG PCD BSP 抽象实现。
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp_usb.h"

#include <stddef.h>

/** @brief ST USB Device 使用的 USB OTG HS PCD 句柄。 */
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;

#ifndef BSP_USB_USE_DEVICE_STACK
/* ==========================================================================
 * 依赖与私有配置
 * --------------------------------------------------------------------------
 * 本区放置板级外设封装需要的私有宏、局部表和轻量状态，不承载机器人业务语义。
 * ========================================================================== */
#define BSP_USB_USE_DEVICE_STACK 1    /**< 置 1 时 USB PCD HAL 回调由 ST USB Device stack 接管。 */
#endif

/**
 * @brief USB PCD 单个事件的回调绑定槽。
 */
typedef struct
{
    bsp_usb_event_callback_t callback;    /**< 事件回调，可为 NULL。 */
    void *user;                           /**< 事件回调用户上下文。 */
} bsp_usb_callback_slot_t;

/**
 * @brief USB PCD 控制器分发槽位。
 */
typedef struct
{
    bsp_usb_id_t id;                                        /**< BSP 层暴露给上层使用的 USB 标识。 */
    PCD_HandleTypeDef *hpcd;                                /**< CubeMX/HAL 生成的 PCD 句柄。 */
    USB_OTG_GlobalTypeDef *instance;                        /**< USB OTG 外设实例地址。 */
    bsp_usb_callback_slot_t events[BSP_USB_EVENT_COUNT];    /**< USB PCD 事件回调表。 */
} bsp_usb_slot_t;

/**
 * @brief 板级 USB PCD 槽位表。
 *
 * 新增或删除 USB PCD 实例时，需要同步维护 bsp_usb_id_t 和该表。
 */
static bsp_usb_slot_t bsp_usb_slots[] = {
    {.id = BSP_USB_ID_HS, .hpcd = &hpcd_USB_OTG_HS, .instance = USB_OTG_HS},
};

#define BSP_USB_SLOT_COUNT ((uint32_t)(sizeof(bsp_usb_slots) / sizeof(bsp_usb_slots[0])))    /**< bsp_usb_slots[] 中受 BSP 管理的 USB PCD 数量。 */

/**
 * @brief 根据 HAL PCD 句柄查找槽位。
 * @param hpcd HAL PCD 句柄。
 * @return 匹配的槽位；未找到时返回 NULL。
 */
static bsp_usb_slot_t *slot_by_handle(PCD_HandleTypeDef *hpcd)
{
    uint32_t i;

    if (hpcd == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_USB_SLOT_COUNT; i++) {
        if (bsp_usb_slots[i].hpcd == hpcd) {
            return &bsp_usb_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据 BSP USB 标识查找槽位。
 * @param id BSP USB 标识。
 * @return 匹配的槽位；未找到时返回 NULL。
 */
static bsp_usb_slot_t *slot_by_id(bsp_usb_id_t id)
{
    uint32_t i;

    for (i = 0U; i < BSP_USB_SLOT_COUNT; i++) {
        if (bsp_usb_slots[i].id == id) {
            return &bsp_usb_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据 USB OTG 外设实例查找槽位。
 * @param instance USB OTG 外设实例地址。
 * @return 匹配的槽位；未找到时返回 NULL。
 */
static bsp_usb_slot_t *slot_by_instance(USB_OTG_GlobalTypeDef *instance)
{
    uint32_t i;

    if (instance == NULL) {
        return NULL;
    }

    for (i = 0U; i < BSP_USB_SLOT_COUNT; i++) {
        if (bsp_usb_slots[i].instance == instance) {
            return &bsp_usb_slots[i];
        }
    }

    return NULL;
}

/**
 * @brief 分发 USB PCD HAL 回调事件。
 * @param hpcd 触发事件的 HAL PCD 句柄。
 * @param event BSP USB 事件类型。
 */
#if (BSP_USB_USE_DEVICE_STACK == 0)
/* ==========================================================================
 * 局部工具与参数检查
 * --------------------------------------------------------------------------
 * 局部工具负责索引、句柄、参数和 HAL 状态检查，避免上层直接碰 CubeMX 句柄细节。
 * ========================================================================== */
static void dispatch_event(PCD_HandleTypeDef *hpcd, bsp_usb_event_t event)
{
    bsp_usb_slot_t *slot = slot_by_handle(hpcd);

    if ((slot == NULL) || (event >= BSP_USB_EVENT_COUNT) || (slot->events[event].callback == NULL)) {
        return;
    }

    slot->events[event].callback(hpcd, slot->events[event].user);
}
#endif

/* ==========================================================================
 * 公开 API
 * --------------------------------------------------------------------------
 * 公开 API 只暴露板级能力和安全钳位，Devices/Apps 通过这些接口绑定硬件。
 * ========================================================================== */
void bsp_usb_init(void)
{
    uint32_t i;
    uint32_t event;

    for (i = 0U; i < BSP_USB_SLOT_COUNT; i++) {
        /* @diff: 旧版 bsp_usb 维护 CDC/demo 发送数据；新版初始化只清空 PCD 事件回调。 */
        for (event = 0U; event < BSP_USB_EVENT_COUNT; event++) {
            bsp_usb_slots[i].events[event].callback = NULL;
            bsp_usb_slots[i].events[event].user = NULL;
        }
    }
}

PCD_HandleTypeDef *bsp_usb_handle(bsp_usb_id_t id)
{
    bsp_usb_slot_t *slot = slot_by_id(id);

    return (slot == NULL) ? NULL : slot->hpcd;
}

PCD_HandleTypeDef *bsp_usb_from_instance(USB_OTG_GlobalTypeDef *instance)
{
    bsp_usb_slot_t *slot = slot_by_instance(instance);

    return (slot == NULL) ? NULL : slot->hpcd;
}

uint8_t bsp_usb_is_supported(PCD_HandleTypeDef *hpcd)
{
    return (slot_by_handle(hpcd) != NULL) ? 1U : 0U;
}

HAL_StatusTypeDef bsp_usb_start(PCD_HandleTypeDef *hpcd)
{
    if (slot_by_handle(hpcd) == NULL) {
        return HAL_ERROR;
    }

    return HAL_PCD_Start(hpcd);
}

HAL_StatusTypeDef bsp_usb_stop(PCD_HandleTypeDef *hpcd)
{
    if (slot_by_handle(hpcd) == NULL) {
        return HAL_ERROR;
    }

    return HAL_PCD_Stop(hpcd);
}

HAL_StatusTypeDef bsp_usb_dev_connect(PCD_HandleTypeDef *hpcd)
{
    if (slot_by_handle(hpcd) == NULL) {
        return HAL_ERROR;
    }

    return HAL_PCD_DevConnect(hpcd);
}

HAL_StatusTypeDef bsp_usb_dev_disconnect(PCD_HandleTypeDef *hpcd)
{
    if (slot_by_handle(hpcd) == NULL) {
        return HAL_ERROR;
    }

    return HAL_PCD_DevDisconnect(hpcd);
}

HAL_StatusTypeDef bsp_usb_on(PCD_HandleTypeDef *hpcd,
                             bsp_usb_event_t event,
                             bsp_usb_event_callback_t callback,
                             void *user)
{
    bsp_usb_slot_t *slot = slot_by_handle(hpcd);

    if ((slot == NULL) || (event >= BSP_USB_EVENT_COUNT)) {
        return HAL_ERROR;
    }

    slot->events[event].callback = callback;
    slot->events[event].user = user;
    return HAL_OK;
}

#if (BSP_USB_USE_DEVICE_STACK == 0)
/**
 * @brief USB 连接回调。
 * @param hpcd HAL PCD 句柄。
 */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
    dispatch_event(hpcd, BSP_USB_EVENT_CONNECT);
}

/**
 * @brief USB 断开回调。
 * @param hpcd HAL PCD 句柄。
 */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    dispatch_event(hpcd, BSP_USB_EVENT_DISCONNECT);
}

/**
 * @brief USB 复位回调。
 * @param hpcd HAL PCD 句柄。
 */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    dispatch_event(hpcd, BSP_USB_EVENT_RESET);
}

/**
 * @brief USB 挂起回调。
 * @param hpcd HAL PCD 句柄。
 */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
    dispatch_event(hpcd, BSP_USB_EVENT_SUSPEND);
}

/**
 * @brief USB 恢复回调。
 * @param hpcd HAL PCD 句柄。
 */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
    dispatch_event(hpcd, BSP_USB_EVENT_RESUME);
}

/**
 * @brief USB SOF 回调。
 * @param hpcd HAL PCD 句柄。
 */
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    dispatch_event(hpcd, BSP_USB_EVENT_SOF);
}
#endif
