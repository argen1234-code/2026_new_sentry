#ifndef __BSP_USB_H__
#define __BSP_USB_H__

#include "main.h"

#define byte0(dw_temp)     (*(char*)(&dw_temp))
#define byte1(dw_temp)     (*((char*)(&dw_temp) + 1))
#define byte2(dw_temp)     (*((char*)(&dw_temp) + 2))
#define byte3(dw_temp)     (*((char*)(&dw_temp) + 3))
#define TXFIFO_SZ 256


extern void usb_start(void);
extern void usb_send_data(uint8_t num, float data); 
extern void usb_sendframetail(void);
extern void usb_demo(void);
extern void usb_transmit(uint8_t* buf, uint16_t len);
extern void usb_send_referee_data(void);

#endif /* __usb_H__ */














