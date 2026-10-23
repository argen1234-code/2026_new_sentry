/**
  * @file       usb_cdc_port.h
  * @brief      USB CDC 接收环形缓冲（中断收、任务解析）
  * @details    CDC_Receive_HS 只负责把字节入队，协议解析放到通信任务里，
  *             避免在 USB 中断中运行解析器。
  */

#ifndef USB_CDC_PORT_H
#define USB_CDC_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化内部状态（清零环形缓冲） */
void usb_cdc_port_init(void);

/* CDC_Receive_HS 回调中调用：把新到字节压入环形缓冲 */
void usb_cdc_port_on_rx(const uint8_t *data, uint32_t len);

/* 任务上下文调用：排空环形缓冲并送入 legacy_bridge */
void usb_cdc_port_poll_rx(void);

/* USB 关闭/复位时清空缓冲 */
void usb_cdc_port_reset_rx(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_PORT_H */
