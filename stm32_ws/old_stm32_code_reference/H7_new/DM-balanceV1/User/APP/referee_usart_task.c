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
#include "referee_usart_task.h"
#include "main.h"
#include "cmsis_os.h"

#include "remote_control.h"
#include "bsp_usart.h"
#include "detect_task.h"

#include "CRC8_CRC16.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"




/**
  * @brief          single byte upacked 
  * @param[in]      void
  * @retval         none
  */
/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
static void referee_unpack_fifo_data(void);

 
extern UART_HandleTypeDef huart6;

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];

//fifo_s_t referee_fifo;
//uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
//unpack_data_t referee_unpack_obj;
unpack_data_t referee_unpack_data = {0};						//裁判系统数据解包结构体变量
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
void referee_usart_task(void const * argument)
{
    init_referee_data();
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);
//    usart6_init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);

    while(1)
    {

        referee_unpack_fifo_data();
        osDelay(10);
    }
}

/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void)
{
  uint8_t sof = HEADER_SOF;
	uint8_t byte = 0;
  unpack_data_t *p_unpack = &referee_unpack_data;
	
	uint8_t zero[256] = {0};
	memcpy(&referee_unpack_data, zero, sizeof(unpack_data_t));

  while(fifo_s_used(&referee_fifo))
  {
    byte = fifo_s_get(&referee_fifo);
    switch(p_unpack->unpack_step)
    {
      case STEP_HEADER_SOF:
      {
        if(byte == sof)
        {
					p_unpack->protocol_packet[p_unpack->index++] = byte;
					p_unpack->unpack_step = STEP_LENGTH_LOW;
        }
        else
        {
          p_unpack->index = 0;
        }
      }break;
      /******************************************/
      case STEP_LENGTH_LOW:
      {
        p_unpack->data_length = byte;
        p_unpack->protocol_packet[p_unpack->index++] = byte;
        p_unpack->unpack_step = STEP_LENGTH_HIGH;
      }break;
      /******************************************/
      case STEP_LENGTH_HIGH:
      {
        p_unpack->data_length |= (byte << 8);
        p_unpack->protocol_packet[p_unpack->index++] = byte;

        if(p_unpack->data_length < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
        {
          p_unpack->unpack_step = STEP_FRAME_SEQ;
        }
        else
        {
          p_unpack->unpack_step = STEP_HEADER_SOF;
          p_unpack->index = 0;
        }
      }break;
      /******************************************/
      case STEP_FRAME_SEQ:
      {
        p_unpack->protocol_packet[p_unpack->index++] = byte;
        p_unpack->unpack_step = STEP_HEADER_CRC8;
      }break;
      /******************************************/
      case STEP_HEADER_CRC8:
      {
        p_unpack->protocol_packet[p_unpack->index++] = byte;

        if(p_unpack->index == REF_PROTOCOL_HEADER_SIZE)
        {
          if(verify_CRC8_check_sum(p_unpack->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
          {
            p_unpack->unpack_step = STEP_DATA_CRC16;
          }
          else
          {
            p_unpack->unpack_step = STEP_HEADER_SOF;
            p_unpack->index = 0;
          }
        }
      }break;
      /******************************************/
      case STEP_DATA_CRC16:
      {
        if(p_unpack->index < (REF_HEADER_CRC_CMDID_LEN + p_unpack->data_length))
        {
           p_unpack->protocol_packet[p_unpack->index++] = byte;  
        }
        else
				{
					if(verify_CRC16_check_sum(p_unpack->protocol_packet, (REF_HEADER_CRC_CMDID_LEN + p_unpack->data_length)))
					{
						referee_handle_data(p_unpack->protocol_packet);
					}
          p_unpack->unpack_step = STEP_HEADER_SOF;
          p_unpack->index = 0;
        }
      }break;
      /******************************************/
      default:
      {
        p_unpack->unpack_step = STEP_HEADER_SOF;
        p_unpack->index = 0;
      }break;
    }
  }
}





