#include "can_bsp.h"
#include "fdcan.h"
#include "string.h"

/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      CAN interrupt to receive motor data; CAN send to control motors.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];

FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];

uint8_t              chassis_can_send_data[8];
uint8_t              gimbal_can_send_data[8];
uint8_t              shoot_can_send_data[8];
uint8_t MOTOR_Data[8]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 电机数据
// 达妙电机命令
uint8_t MOTOR_Enable[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};   // 电机使能命令
uint8_t MOTOR_Failure[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};  // 电机失能命令
uint8_t MOTOR_Save_zero[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}; // 电机保存零点命令
uint8_t MOTOR_Clear_err[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB}; // 电机清除错误命令

uint8_t MOTOR_MIT_MODE[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFC};	//电机切换到MIT模式
uint8_t MOTOR_POS_MODE[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01, 0xFC};	//电机切换到位置模式
uint8_t MOTOR_VEL_MODE[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x02, 0xFC};	//电机切换到速度模式

uint8_t RS_MOTOR_PRE_MODE[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFD}; // 灵足电机私有模式
uint8_t RS_MOTOR_MIT_MODE[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x02, 0xFD}; // 灵足电机 MIT 模式
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

// MIT 速度滤波缓冲（抑制近似正弦噪声）
static float mit_vel_lpf[8] = {0.0f};

// dji motor data read（宏改为内联函数）
static inline void get_motor_measure(motor_measure_t *ptr, const uint8_t data[8])
{
    ptr->last_ecd = ptr->ecd;
    ptr->ecd = (uint16_t)((data[0] << 8) | data[1]);
    ptr->speed_rpm = (uint16_t)((data[2] << 8) | data[3]);
    ptr->given_current = (uint16_t)((data[4] << 8) | data[5]);
    ptr->temperate = data[6];
}

/**
 * @brief  MITFdbData: 获取 MIT 电机反馈数据（内联），含速度低通滤波
 * @note   vel 噪声近似正弦，使用一阶低通平滑
 */
static inline void MITFdbData(MITMeasure_t *MIT_measure, const uint8_t rx_data[8], uint8_t cnt)
{
    MIT_measure->fdb.id = (rx_data[0]) & 0x0F;
    MIT_measure->fdb.state = (rx_data[0]) >> 4;
    MIT_measure->fdb.p_int = ((rx_data[1] << 8) | rx_data[2]);
    MIT_measure->fdb.v_int = ((rx_data[3] << 4) | (rx_data[4] >> 4));
    MIT_measure->fdb.t_int = (((rx_data[4] & 0xF) << 8) | rx_data[5]);
    MIT_measure->fdb.pos = uint_to_float(MIT_measure->fdb.p_int, MIT_measure->param.P_MIN, MIT_measure->param.P_MAX, 16);

    const float vel_raw = uint_to_float(MIT_measure->fdb.v_int, MIT_measure->param.V_MIN, MIT_measure->param.V_MAX, 12);
    uint8_t idx = cnt;
    const float alpha = 0.15f; // 一阶低通滤波系数
    mit_vel_lpf[idx] = mit_vel_lpf[idx] + alpha * (vel_raw - mit_vel_lpf[idx]);
    MIT_measure->fdb.vel = mit_vel_lpf[idx];

    MIT_measure->fdb.tor = uint_to_float(MIT_measure->fdb.t_int, MIT_measure->param.T_MIN, MIT_measure->param.T_MAX, 12);
    MIT_measure->fdb.t_mos = (float)(rx_data[6]);
    MIT_measure->fdb.t_motor = (float)(rx_data[7]);
}

/*
  DJI 电机反馈帧结构体
  motor data: 0~3 chassis 3508; 4 yaw 6020; 5 pitch 6020; 6 trigger 2006
*/
motor_measure_t DJI_MOTOR_MEASURE[8];

/*
  MIT 电机反馈帧结构体
*/
MITMeasure_t MIT_MOTOR_MEASURE[8];

uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t can1_cnt = 0;
uint8_t can2_cnt = 0;
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET){
		if(hcan->Instance == FDCAN1){
			HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
			uint32_t fdb_time = HAL_GetTick();
			switch (RxHeader1.Identifier)
			{					
          case MIT_M1_ID:
					case MIT_M2_ID:
					case MIT_M3_ID:
					case MIT_M4_ID:
					case MIT_M5_ID:
					case MIT_M6_ID:
					case MIT_M7_ID:
					case MIT_M8_ID:	
					{
							//get motor id
							can1_cnt = RxHeader1.Identifier - MIT_M1_ID;
							if(can1_cnt > 7){
								can1_cnt = 0;
								return;
							}
							MITFdbData(&MIT_MOTOR_MEASURE[can1_cnt], g_Can1RxData, can1_cnt);
							MIT_MOTOR_MEASURE[can1_cnt].fdb.last_fdb_time = fdb_time;
							break;
					}
          
					case CAN_3508_M1_ID:
					case CAN_3508_M2_ID:
					case CAN_3508_M3_ID:
					case CAN_3508_M4_ID:
					case CAN_YAW_MOTOR_ID:
					case CAN_PIT_MOTOR_ID:
					case CAN_TRIGGER_MOTOR_ID:
					{
							
							//get motor id
							can1_cnt = RxHeader1.Identifier - CAN_3508_M1_ID;
							if(can1_cnt > 7){
								can1_cnt = 0;
								return;
							}
							get_motor_measure(&DJI_MOTOR_MEASURE[can1_cnt], g_Can1RxData);
							
							DJI_MOTOR_MEASURE[can1_cnt].last_fdb_time = fdb_time;
							break;
					}

					default:
					{
							break;
					}
			}
		}
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hcan, uint32_t RxFifo1ITs)
{

	if((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET){
		if(hcan->Instance == FDCAN2){
			HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO1, &RxHeader2, g_Can2RxData);
			uint32_t fdb_time = HAL_GetTick();
			switch (RxHeader2.Identifier)
			{					
          case MIT_M1_ID:
					case MIT_M2_ID:
					case MIT_M3_ID:
					case MIT_M4_ID:
					case MIT_M5_ID:
					case MIT_M6_ID:
					case MIT_M7_ID:
					case MIT_M8_ID:	
					{
							
							//get motor id
							can2_cnt = RxHeader2.Identifier - MIT_M1_ID;
							if(can2_cnt > 7){
								can2_cnt = 0;
								return;
							}
							MITFdbData(&MIT_MOTOR_MEASURE[can2_cnt], g_Can2RxData, can2_cnt);
							MIT_MOTOR_MEASURE[can2_cnt].fdb.last_fdb_time = fdb_time;
							break;
					}

					case CAN_3508_M1_ID:
					case CAN_3508_M2_ID:
					case CAN_3508_M3_ID:
					case CAN_3508_M4_ID:
					case CAN_YAW_MOTOR_ID:
					case CAN_PIT_MOTOR_ID:
					case CAN_TRIGGER_MOTOR_ID:
					{
							
							//get motor id
					
							can2_cnt = RxHeader2.Identifier - CAN_3508_M1_ID;
							if(can2_cnt > 7){
								can2_cnt = 0;
								return;
							}
							get_motor_measure(&DJI_MOTOR_MEASURE[can2_cnt], g_Can2RxData);
							DJI_MOTOR_MEASURE[can2_cnt].last_fdb_time = fdb_time;
							break;
					}

					default:
					{
							break;
					}
			}
		}
	}
}

void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
		canx_send_data(&GIMBAL_CAN, CAN_GIMBAL_ALL_ID, gimbal_can_send_data, 8);
}

const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &DJI_MOTOR_MEASURE[4];
}

const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &DJI_MOTOR_MEASURE[5];
}

const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &DJI_MOTOR_MEASURE[6];
}
const motor_measure_t *get_fric1_motor_measure_point(void){
	return &DJI_MOTOR_MEASURE[0];
}
const motor_measure_t *get_fric1__motor_measure_point(void){
	return &DJI_MOTOR_MEASURE[2];
}
const motor_measure_t *get_fric2_motor_measure_point(void){
	return &DJI_MOTOR_MEASURE[1];
}
const motor_measure_t *get_fric2__motor_measure_point(void){
	return &DJI_MOTOR_MEASURE[3];
}

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
		canx_send_data(&CHASSIS_CAN, CAN_3508_ALL_ID, chassis_can_send_data, 8);
}

void CAN_cmd_chassis_6020(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
		canx_send_data(&CHASSIS_CAN, CAN_6020_ALL_ID, chassis_can_send_data, 8);
}

void CAN_cmd_chassis_reset_ID(void)
{
    memset(chassis_can_send_data, 0, sizeof(chassis_can_send_data));
		canx_send_data(&CHASSIS_CAN, CAN_3508_ALL_ID, chassis_can_send_data, 8);
}

void CAN_cmd_friction(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    shoot_can_send_data[0] = motor1 >> 8;
    shoot_can_send_data[1] = motor1;
    shoot_can_send_data[2] = motor2 >> 8;
    shoot_can_send_data[3] = motor2;
    shoot_can_send_data[4] = motor3 >> 8;
    shoot_can_send_data[5] = motor3;
    shoot_can_send_data[6] = motor4 >> 8;
    shoot_can_send_data[7] = motor4;
		canx_send_data(&FRICTION_CAN, CAN_3508_ALL_ID, shoot_can_send_data, 8);
}





void CAN_cmd_MIT(FDCAN_HandleTypeDef *hcan,uint16_t id, MITMeasure_t motor)
 { 
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	pos_tmp = float_to_uint(motor.set.POS, motor.param.P_MIN, motor.param.P_MAX, 16);
	vel_tmp = float_to_uint(motor.set.VEL, motor.param.V_MIN, motor.param.V_MAX, 12);
	kp_tmp = float_to_uint(motor.set.KP, motor.param.KP_MIN, motor.param.KP_MAX, 12);
	kd_tmp = float_to_uint(motor.set.KD, motor.param.KD_MIN, motor.param.KD_MAX, 12);
	tor_tmp = float_to_uint(motor.set.TOR, motor.param.T_MIN, motor.param.T_MAX, 12);

	
	MOTOR_Data[0] = (pos_tmp >> 8);
	MOTOR_Data[1] = pos_tmp;
	MOTOR_Data[2] = (vel_tmp >> 4);
	MOTOR_Data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	MOTOR_Data[4] = kp_tmp;
	MOTOR_Data[5] = (kd_tmp >> 4);
	MOTOR_Data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	MOTOR_Data[7] = tor_tmp;
	
	canx_send_data(hcan, id + MIT_mode, MOTOR_Data, 8);
 }

void CAN_cmd_POS(FDCAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel)
{
    uint8_t *pbuf,*vbuf;
    pbuf=(uint8_t*)&_pos;
    vbuf=(uint8_t*)&_vel;

    MOTOR_Data[0] = *pbuf;
    MOTOR_Data[1] = *(pbuf+1);
    MOTOR_Data[2] = *(pbuf+2);
    MOTOR_Data[3] = *(pbuf+3);
    MOTOR_Data[4] = *vbuf;
    MOTOR_Data[5] = *(vbuf+1);
    MOTOR_Data[6] = *(vbuf+2);
    MOTOR_Data[7] = *(vbuf+3);

		canx_send_data(hcan, id + POS_mode, MOTOR_Data, 8);
}
 
void CAN_cmd_SPEED(FDCAN_HandleTypeDef* hcan, uint16_t id, float _vel)
{
    uint8_t *vbuf;
    vbuf=(uint8_t*)&_vel;

    MOTOR_Data[0] = *vbuf;
    MOTOR_Data[1] = *(vbuf+1);
    MOTOR_Data[2] = *(vbuf+2);
    MOTOR_Data[3] = *(vbuf+3);
		
		canx_send_data(hcan, id + SPEED_mode, MOTOR_Data, 4);
}

void RS_MOTOR_PRE(FDCAN_HandleTypeDef *hcan,uint16_t id){
	canx_send_data(hcan, id, RS_MOTOR_PRE_MODE, 8);
}

void RS_Motor_MIT(FDCAN_HandleTypeDef *hcan,uint16_t id){
	canx_send_data(hcan, id, RS_MOTOR_MIT_MODE, 8);
}

void Motor_enable(FDCAN_HandleTypeDef *hcan, uint16_t id)
{
	canx_send_data(hcan, id, MOTOR_Enable, 8);
}

void Motor_disable(FDCAN_HandleTypeDef *hcan, uint16_t id)
{
	canx_send_data(hcan, id, MOTOR_Failure, 8);
}

void Motor_save_zero(FDCAN_HandleTypeDef *hcan, uint16_t id)
{
	canx_send_data(hcan, id, MOTOR_Save_zero, 8);
}

void Motor_clear_err(FDCAN_HandleTypeDef *hcan, uint16_t id)
{
	canx_send_data(hcan, id, MOTOR_Clear_err, 8);
}

const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &DJI_MOTOR_MEASURE[(i & 0x07)];
}

MITMeasure_t *get_mit_motor_measure_point(uint8_t i)
{
    return &MIT_MOTOR_MEASURE[i];
}

void FDCAN1_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */	
	sFilterConfig.IdType = FDCAN_STANDARD_ID;//扩展ID不接收
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x00000000; // 
  sFilterConfig.FilterID2 = 0x00000000; // 
  if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
		
  /* global filter */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

	/* 开启RX FIFO0的新数据中断 */
  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
 
  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
}

void FDCAN2_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  /* Configure Rx filter */
  sFilterConfig.IdType =  FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 1;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  sFilterConfig.FilterID1 = 0x00000000;
  sFilterConfig.FilterID2 = 0x00000000;
  if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure global filter */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Activate Rx FIFO 1 new message notification */
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
}

uint32_t g_can_fail_len = 0xFFFF; 
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
	g_can_fail_len = len;
	FDCAN_TxHeaderTypeDef TxHeader;
	
	TxHeader.Identifier = id;                 // CAN ID
  TxHeader.IdType =  FDCAN_STANDARD_ID ;        
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;  
  if(len<=8)	
	{
	  TxHeader.DataLength = len<<16;    // 发送长度：8byte
	}
	else  if(len==12)	
	{
	   TxHeader.DataLength =FDCAN_DLC_BYTES_12;
	}
	else  if(len==16)	
	{
	  TxHeader.DataLength =FDCAN_DLC_BYTES_16;
	
	}
  else  if(len==20)
	{
		TxHeader.DataLength =FDCAN_DLC_BYTES_20;
	}		
	else  if(len==24)	
	{
	 TxHeader.DataLength =FDCAN_DLC_BYTES_24;	
	}else  if(len==48)
	{
	 TxHeader.DataLength =FDCAN_DLC_BYTES_48;
	}else  if(len==64)
   {
		 TxHeader.DataLength =FDCAN_DLC_BYTES_64;
	 }									
	TxHeader.ErrorStateIndicator =  FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;//比特率切换关闭，不适用于经典CAN
  TxHeader.FDFormat =  FDCAN_CLASSIC_CAN;            // CANFD
  TxHeader.TxEventFifoControl =  FDCAN_NO_TX_EVENTS;  
  TxHeader.MessageMarker = 0;//消息标记

   // 发送CAN指令
  if(HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
  {
       // 发送失败处理
//       Error_Handler();      
  }
//	 HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data);
	 return 0;

}

float uint_to_float(int x_int, float x_min, float x_max, int bits){
 float span = x_max - x_min;
 float offset = x_min;
 return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits){
 float span = x_max - x_min;
 float offset = x_min;
 return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
