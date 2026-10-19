#include "bsp_usb.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "math.h"
#include "usbd_cdc_if.h"
#include "bsp_usart.h"  /* gimbal_data_t */
//#include "usart.h"

#define MAX_BUFFER_SIZE 1024
uint8_t send_buf[MAX_BUFFER_SIZE];
uint8_t receive_buf[MAX_BUFFER_SIZE * 2];
uint16_t cnt = 0;

/**
***********************************************************************
* @brief:      usb_start(void)
* @param:		   void
* @retval:     void
* @details:    发送数据给上位机
***********************************************************************
**/
void usb_start(void)
{
	usb_demo();		// demo示例
	
//	usb_sendframetail();
}

/**
***********************************************************************
* @brief:      usb_transmit(uint8_t* buf, uint16_t len)
* @param:		void
* @retval:     void
* @details:    修改通信工具，USART或者USB
***********************************************************************
**/
void usb_transmit(uint8_t* buf, uint16_t len)
{
//	HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 0xFFFF);
	CDC_Transmit_HS((uint8_t *)buf, len);
}
/**
***********************************************************************
* @brief:      usb_send_data(float data)
* @param[in]:  num: 数据编号 data: 数据 
* @retval:     void
* @details:    将浮点数据拆分成单字节
***********************************************************************
**/
void usb_send_data(uint8_t num, float data) 
{
	send_buf[cnt++] = byte0(data);
	send_buf[cnt++] = byte1(data);
	send_buf[cnt++] = byte2(data);
	send_buf[cnt++] = byte3(data);
}
/**
***********************************************************************
* @brief      usb_sendframetail(void)
* @param      NULL 
* @retval     void
* @details:   给数据包发送帧尾
***********************************************************************
**/
void usb_sendframetail(void) 
{
//	uint8_t send[4]={'a','b','c',':'};
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x80;
	send_buf[cnt++] = 0x7f;
	
	/* 将数据和帧尾打包发送 */
//	usb_transmit((uint8_t *)send, 4);
	usb_transmit((uint8_t *)send_buf, cnt);
	cnt = 0;// 每次发送完帧尾都需要清零
}
/**
***********************************************************************
* @brief      usb_demo(void)
* @param      NULL 
* @retval     void
* @details:   demo示例
***********************************************************************
**/
void usb_demo(void) 
{
	static float scnt = 0.0f;

	scnt += 0.01f;

	if(scnt >= 360.0f)
		scnt = 0.0f;

	float v1 = scnt;
	float v2 = sin((double)scnt / 180 * 3.14159) * 180 + 180;
	float v3 = sin((double)(scnt + 120) / 180 * 3.14159) * 180 + 180;
	float v4 = sin((double)(scnt + 240) / 180 * 3.14159) * 180 + 180;

	// Call the function to store the data in the buffer
	usb_send_data(0, v1);
	usb_send_data(1, v2);
	usb_send_data(2, v3);
	usb_send_data(3, v4);

	// Call the function to send the frame tail
	usb_sendframetail();
//		usb_transmit((uint8_t *)usb_buf,  usb_buf_len);
//		memset((uint8_t *)usb_buf, 0, sizeof(usb_buf));   // 整个数组清零
//		usb_buf_len = 0; 
}

/**
***********************************************************************
* @brief      usb_send_referee_data(void)
* @param      NULL 
* @retval     void
* @details:   简化版：直接发送裁判系统数据（帧头+数据+校验+帧尾）
***********************************************************************
**/
void usb_send_referee_data(void)
{
	uint8_t buf[32];
	uint16_t pos = 0;
	
	/* 帧头 */
	buf[pos++] = 0xAA;
	buf[pos++] = 0x55;
	
	/* 数据长度 */
	buf[pos++] = 8;
	
	/* 裁判数据（8字节） */
	buf[pos++] = gimbal_data.game_progress;
	buf[pos++] = (uint8_t)(gimbal_data.stage_remain_time & 0xFF);
	buf[pos++] = (uint8_t)(gimbal_data.stage_remain_time >> 8);
	buf[pos++] = gimbal_data.game_result;
	buf[pos++] = (uint8_t)(gimbal_data.current_HP & 0xFF);
	buf[pos++] = (uint8_t)(gimbal_data.current_HP >> 8);
	buf[pos++] = (uint8_t)(gimbal_data.shooter_17mm_heat & 0xFF);
	buf[pos++] = (uint8_t)(gimbal_data.shooter_17mm_heat >> 8);
	
	/* 校验和（简单累加） */
	uint8_t sum = 0;
	for (int i = 2; i < pos; i++) sum += buf[i];
	buf[pos++] = sum;
	
	/* 帧尾 */
	buf[pos++] = 0x0D;
	buf[pos++] = 0x0A;
	
	/* 发送 */
	CDC_Transmit_HS(buf, pos);
}

