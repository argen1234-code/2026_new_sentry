/**
  * @file       wattmeter_api.h
  * @brief      功率表桩（本工程不接）
  */
#ifndef WATTMETER_API_H
#define WATTMETER_API_H

#include <stdint.h>

/* 与本工程无关，但 bsp_fdcan.c 的接收分支会引用该 ID */
#ifndef WATTMETER_CAN_ID
#define WATTMETER_CAN_ID 0x212U
#endif

void wattmeter_feedback_handle(uint16_t can_id, const uint8_t *can_rx_data, uint8_t len);

#endif
