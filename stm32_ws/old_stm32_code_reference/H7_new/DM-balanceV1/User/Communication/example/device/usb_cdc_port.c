#include "usb_cdc_port.h"
#include "stm32h7xx_hal.h"
#include "usbd_core.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceHS;

void usb_cdc_port_init(void)
{
    /* Nothing required here; USB is initialized by MX_USB_DEVICE_Init(). */
}

uint32_t usb_cdc_port_write(void *user, const uint8_t *data, uint32_t len)
{
    (void)user;
    if (!data || len == 0) return 0u;
    if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) return 0u;

    /* CDC HS bulk IN max packet typically 512. We submit up to 512 bytes at once. */
    uint16_t to_send = (len > 512u) ? 512u : (uint16_t)len;

    /* If busy, report 0 (non-blocking). Caller may retry later. */
    uint8_t res = CDC_Transmit_HS((uint8_t*)data, to_send);
    if (res == USBD_OK) {
        return (uint32_t)to_send;
    }
    return 0u;
}

static void usb_cdc_port_flush(void *user)
{
    (void)user; /* No-op */
}

static uint16_t usb_cdc_port_get_mtu(void *user)
{
    (void)user;
    return 512u; /* HS bulk IN packet size */
}

void usb_cdc_port_get_ops(uproto_port_ops_t *ops)
{
    if (!ops) return;
    ops->write = usb_cdc_port_write;
    ops->flush = usb_cdc_port_flush;
    ops->get_mtu = usb_cdc_port_get_mtu;
    ops->user = NULL;
}


static uproto_context_t *s_ctx = NULL;

void usb_cdc_port_bind_uproto(uproto_context_t *ctx)
{
    s_ctx = ctx;
}

void usb_cdc_port_on_rx(const uint8_t *data, uint32_t len)
{
    if (!s_ctx || !data || len == 0) return;
    uproto_on_rx_bytes(s_ctx, data, len);
}