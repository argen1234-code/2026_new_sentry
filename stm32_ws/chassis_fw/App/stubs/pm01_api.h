/**
  * @file       pm01_api.h
  * @brief      PM01 功率计桩（本工程不接）
  */
#ifndef PM01_API_H
#define PM01_API_H

#include <stdint.h>

void pm01_response_handle(uint16_t can_id, const uint8_t *can_rx_data);

#endif
