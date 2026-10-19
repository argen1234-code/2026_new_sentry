#include "bsp_usart.h"
#include "string.h"
#include "usart.h"
#include "bsp_tfmini.h"
#include "gimbal_task.h"

//串口5，接收遥控数据



//串口1，接收裁判系统数据
uint8_t usart1_buf[2][USART_RX_BUF_LENGHT];
uint8_t gimbal_receive_data[CHASSIS_DATA_LENGTH];
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
unpack_data_t referee_unpack_obj;
fifo_s_t referee_fifo;

uint8_t remote_buff[SBUS_RX_BUF_NUM];
remoter_t remoter;

//串口7 板间通讯
//static uint8_t gimbal_data[USART_RX_BUF_LENGHT]={0};
uint8_t usart7_buf[ USART_RX_BUF_LENGHT ];//设置缓冲区
uint8_t data_send_from_pc[ USART_RX_BUF_LENGHT] = {0};
uint8_t data_send_from_chassis[ USART_RX_BUF_LENGHT] = {0};
uint8_t restart_array[10]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};//请求重新发送数据

//串口10 闲置
uint8_t usart10_buf[ USART_RX_BUF_LENGHT ];//设置缓冲区

//串口1发送函数
void USART1_Transmit_DMA(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit_DMA(&huart1, pData, Size);
}

//串口1发送函数
void USART1_Transmit_IT(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit_IT(&huart1, pData, Size);
}

//串口1发送函数
void USART1_Transmit(uint8_t *pData, uint16_t Size)
{
  HAL_UART_Transmit(&huart1, pData, Size, 50);
}

gimbal_data_t gimbal_data = {
    .game_progress      = 0,      /* 测试值：比赛进行中 */
    .stage_remain_time  = 0,    /* 测试值：剩余300秒 */
    .game_result        = 0,      /* 测试值：未结束 */
    .current_HP         = 400,   /* 测试值：血量2000 */
    .shooter_17mm_heat  = 0,    /* 测试值：热量128 */
	  .shoot_allowance_17mm = 0,
	  .shoot_speed = 0
};

/***** 云台发来的数据全都到这里面去了 *****/
chassis_data_t chassis_data;

//云台处理底盘发来的数据
void chassis_to_gimbal(uint8_t *data_chassis)
{
	
    static uint8_t game_progress;          // 游戏进度
    static uint16_t stage_remain_time;     // 阶段剩余时间
    static uint8_t game_result;            // 比赛结果
    static uint16_t current_HP;            // 当前血量
    static uint16_t shooter_17mm_heat;     // 17mm发射热量
    static float shoot_speed;              // 射击速度
    static uint16_t shoot_allowance_17mm;  // 17mm射击弹量

    // 2. 解析游戏进度 (1字节)
    game_progress = data_chassis[1];
//    gimbal_data.game_progress = game_progress;

    // 3. 解析阶段剩余时间 (2字节，小端序)
    stage_remain_time  = (uint16_t)data_chassis[2];       // 低字节
    stage_remain_time |= (uint16_t)data_chassis[3] << 8;  // 高字节
    gimbal_data.stage_remain_time = stage_remain_time;

    // 4. 解析比赛结果 (1字节)
    game_result = data_chassis[4];
    gimbal_data.game_result = game_result;

    // 5. 解析当前血量 (2字节，小端序)
    current_HP  = (uint16_t)data_chassis[5];       // 低字节
    current_HP |= (uint16_t)data_chassis[6] << 8;  // 高字节
    gimbal_data.current_HP = current_HP;

    // 6. 解析17mm发射热量 (2字节，小端序)
    shooter_17mm_heat  = (uint16_t)data_chassis[7];       // 低字节
    shooter_17mm_heat |= (uint16_t)data_chassis[8] << 8;  // 高字节
    gimbal_data.shooter_17mm_heat = shooter_17mm_heat;

    // 7. 解析射击速度 (4字节，IEEE 754，小端序)
    union {
        float f;
        uint8_t bytes[4];
    } speed_conv;
    speed_conv.bytes[0] = data_chassis[9];
    speed_conv.bytes[1] = data_chassis[10];
    speed_conv.bytes[2] = data_chassis[11];
    speed_conv.bytes[3] = data_chassis[12];
    shoot_speed = speed_conv.f;
    gimbal_data.shoot_speed = shoot_speed;

    // 8. 解析17mm射击弹量 (2字节，小端序)
    shoot_allowance_17mm  = (uint16_t)data_chassis[13];       // 低字节
    shoot_allowance_17mm |= (uint16_t)data_chassis[14] << 8;  // 高字节
    gimbal_data.shoot_allowance_17mm = shoot_allowance_17mm;
}

static void gimbal_memcpy_data_from_chassis(void)
{
	// 找到's'在源数组中的位置
	static int16_t start_index = -1;
	uint8_t temp[CHASSIS_DATA_LENGTH];
	
	for(uint8_t i = 0; i < CHASSIS_DATA_LENGTH; i++)
	{
		if(gimbal_receive_data[i] == 's')
		{
			start_index = i;
			break;
		}
	}
	
	// 如果找到's'，从该位置开始复制固定长度
	if (start_index != -1)
	{
		memcpy(temp, &gimbal_receive_data[start_index], CHASSIS_DATA_LENGTH);
	}
	
	chassis_to_gimbal(temp);
}

fp32 yaw_motor_relative_angle, GIMBAL_INS_yaw;
//底盘处理云台发来的数据
//void gimba_to_chassis(uint8_t *gimbal_data) {
//    fp32_to_bytes INS;
//	INS.bytes[0] = gimbal_data[1]; 
//	INS.bytes[1] = gimbal_data[2]; 
//	INS.bytes[2] = gimbal_data[3]; 
//	INS.bytes[3] = gimbal_data[4]; 
//	GIMBAL_INS_yaw = INS.fp32;
//	gimbal.INS_yaw = INS.fp32;
//	
//	fp32_to_bytes Motor_yaw;
//	Motor_yaw.bytes[0] = gimbal_data[5]; 
//	Motor_yaw.bytes[1] = gimbal_data[6]; 
//	Motor_yaw.bytes[2] = gimbal_data[7]; 
//	Motor_yaw.bytes[3] = gimbal_data[8]; 
//	yaw_motor_relative_angle = Motor_yaw.fp32;
//	gimbal.motor_yaw = Motor_yaw.fp32;
//    
//}

//串口7
/**
  * @brief  通过串口7发送数据
  * @param  pData: 要发送的数据缓冲区
  * @param  Size: 要发送的数据大小
  * @retval 无
  */
void USART7_Transmit(uint8_t *pData, uint16_t Size)
{
    HAL_UART_Transmit(&huart7, pData, Size, 10);
}
void USART10_Transmit(uint8_t *pData, uint16_t Size){
	HAL_UART_Transmit(&huart10, pData, Size, HAL_MAX_DELAY);
}

void USART10_Transmit_IT(uint8_t *pData, uint16_t Size){
	HAL_UART_Transmit_IT(&huart10, pData, Size);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART7) {
		// 处理接收到的数据
		// 例如，将接收到的数据存入缓冲区或触发某种事件

		// 继续接收下一个数据块
		HAL_UART_Receive_IT(&huart7, usart7_buf, USART_RX_BUF_LENGHT * 2);
	}
	else if (huart->Instance == USART10) {
		/* TFmini UART RX callback */
		tfmini_uart_rx_cplt_callback(huart);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
	if(huart->Instance == USART1)
	{
		/* removed: uart_dma_rx_event_handler */
//		if (Size <= USART_RX_BUF_LENGHT)
//		{
//			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_buf[0], USART_RX_BUF_LENGHT);	// 接收完毕后重启
//			fifo_s_puts(&referee_fifo, (char*)usart1_buf[0], USART_RX_BUF_LENGHT);
//		}
//		else 
//		{
//			HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_buf[1], USART_RX_BUF_LENGHT); // 接收完毕后重启
//			fifo_s_puts(&referee_fifo, (char*)usart1_buf[1], USART_RX_BUF_LENGHT);					   
//		}
		gimbal_memcpy_data_from_chassis();
	}
	if(huart->Instance == UART5)
	{
		if (Size <= RC_FRAME_LENGTH)
		{
			
			sbus_to_rc(remote_buff, &rc_ctrl, &remoter);
		}
		else if(Size > RC_FRAME_LENGTH) // 错误处理
		{	
			memset(remote_buff, 0, SBUS_RX_BUF_NUM);				   
		}

		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, remote_buff, SBUS_RX_BUF_NUM);
	}
	if(huart->Instance == UART7){

	}

	if(huart->Instance == USART10){

	}
	
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
	if(huart->Instance == USART1){
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, usart1_buf[0], USART_RX_BUF_LENGHT); // 接收发生错误后重启
		memset(usart1_buf, 0,  USART_RX_BUF_LENGHT * 2);		
	}
	if(huart->Instance == UART5)
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, remote_buff, SBUS_RX_BUF_NUM); // 接收发生错误后重启
		memset(remote_buff, 0, SBUS_RX_BUF_NUM);							   // 清除接收缓存		
	}
	if(huart->Instance == UART7){

	}
	if(huart->Instance == USART10){
		tfmini_uart_error_callback(huart);
	}
}

/* 转发 TX 完成回调 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* removed: uart_dma_tx_cplt_handler */
}

