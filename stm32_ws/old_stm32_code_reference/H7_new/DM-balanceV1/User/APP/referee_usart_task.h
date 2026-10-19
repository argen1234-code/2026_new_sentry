/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H
#include "main.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024
#define REF_PROTOCOL_FRAME_MAX_SIZE 192
extern uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
typedef enum
{
  STEP_HEADER_SOF  = 0,		//帧头数据帧中起始字节
  STEP_LENGTH_LOW  = 1,   //帧头数据帧中data的长度(低位)
  STEP_LENGTH_HIGH = 2,   //帧头数据帧中data的长度(高位)
  STEP_FRAME_SEQ   = 3,   //帧头包序号
  STEP_HEADER_CRC8 = 4,   //帧头CRC8校验
  STEP_DATA_CRC16  = 5,   //帧数据帧CRC16校验
	
} unpack_step_e;

typedef struct
{
  uint8_t       	protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
	uint16_t				data_length;
  unpack_step_e 	unpack_step;
  uint16_t      	index;
	
} unpack_data_t;
/**
  * @brief          referee task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          裁判系统任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void referee_usart_task(void const * argument);
#endif
