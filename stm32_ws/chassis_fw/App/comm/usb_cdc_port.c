/**
  * @file       usb_cdc_port.c
  * @brief      USB CDC 接收环形缓冲实现
  */

#include "usb_cdc_port.h"

#include "legacy_bridge.h"
#include "stm32h7xx_hal.h"

#ifndef USB_RX_RING_SIZE
#define USB_RX_RING_SIZE 1024u
#endif

#ifndef USB_RX_POLL_CHUNK
#define USB_RX_POLL_CHUNK 128u
#endif

static uint8_t s_rx_ring[USB_RX_RING_SIZE];
static volatile uint16_t s_rx_wpos = 0u;
static volatile uint16_t s_rx_rpos = 0u;
static volatile uint16_t s_rx_count = 0u;

static uint32_t usb_cdc_port_enter_critical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void usb_cdc_port_exit_critical(uint32_t primask)
{
    if (primask == 0u)
    {
        __enable_irq();
    }
}

void usb_cdc_port_init(void)
{
    usb_cdc_port_reset_rx();
}

void usb_cdc_port_on_rx(const uint8_t *data, uint32_t len)
{
    uint32_t primask;

    if (!data || len == 0u)
    {
        return;
    }

    primask = usb_cdc_port_enter_critical();
    for (uint32_t i = 0u; i < len; i++)
    {
        /* 缓冲满时丢弃最旧字节，保证最新命令可达 */
        if (s_rx_count >= USB_RX_RING_SIZE)
        {
            s_rx_rpos++;
            if (s_rx_rpos >= USB_RX_RING_SIZE)
            {
                s_rx_rpos = 0u;
            }
            s_rx_count--;
        }

        s_rx_ring[s_rx_wpos++] = data[i];
        if (s_rx_wpos >= USB_RX_RING_SIZE)
        {
            s_rx_wpos = 0u;
        }
        s_rx_count++;
    }
    usb_cdc_port_exit_critical(primask);
}

void usb_cdc_port_poll_rx(void)
{
    uint8_t chunk[USB_RX_POLL_CHUNK];

    while (1)
    {
        uint16_t n;
        uint32_t primask;

        primask = usb_cdc_port_enter_critical();
        n = s_rx_count;
        if (n > USB_RX_POLL_CHUNK)
        {
            n = USB_RX_POLL_CHUNK;
        }

        for (uint16_t i = 0u; i < n; i++)
        {
            chunk[i] = s_rx_ring[s_rx_rpos++];
            if (s_rx_rpos >= USB_RX_RING_SIZE)
            {
                s_rx_rpos = 0u;
            }
        }
        s_rx_count = (uint16_t)(s_rx_count - n);
        usb_cdc_port_exit_critical(primask);

        if (n == 0u)
        {
            break;
        }

        /* 本工程只跑旧协议，没有需要透传的 uproto 数据 */
        legacy_bridge_feed_rx(chunk, n);
    }
}

void usb_cdc_port_reset_rx(void)
{
    uint32_t primask = usb_cdc_port_enter_critical();
    s_rx_wpos = 0u;
    s_rx_rpos = 0u;
    s_rx_count = 0u;
    usb_cdc_port_exit_critical(primask);
}
