#include "message_task.h"
#include "gimbal_task.h"
#include "chassis_power_control.h"

#include "main.h"

#include "cmsis_os.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "can_bsp.h"
#include "user_lib.h"
#include "referee.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "pid.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "auto_aim.h"
#include "robot_param.h"


//目前让所有的兵种共用同一个双板通信函数
#if ROBOT_BOARD  ==	chassis_board
//用于发送的数组
#define PACK_SIZE  15
uint8_t send[25];

void send_to_gimbal(void){
	//发送陀螺仪数据给云台
	static uint8_t send_to_gimbal[PACK_SIZE];
	uint8_t chassis_mode;
  fp32_to_bytes yaw, roll, pitch, chassis_vx, chassis_vy, chassis_wz;
  yaw.fp32 = chassis_move.chassis_yaw;
  pitch.fp32 = chassis_move.chassis_pitch;
  roll.fp32 = chassis_move.chassis_roll;
  send_to_gimbal[0] = 's';
  send_to_gimbal[1] = yaw.bytes[0];
  send_to_gimbal[2] = yaw.bytes[1];
  send_to_gimbal[3] = yaw.bytes[2];
  send_to_gimbal[4] = yaw.bytes[3];
  //
  send_to_gimbal[5] = pitch.bytes[0];
  send_to_gimbal[6] = pitch.bytes[1];
  send_to_gimbal[7] = pitch.bytes[2];
  send_to_gimbal[8] = pitch.bytes[3];
  //底盘旋转速度
  send_to_gimbal[9] = roll.bytes[0];
  send_to_gimbal[10] = roll.bytes[1];
  send_to_gimbal[11] = roll.bytes[2];
  send_to_gimbal[12] = roll.bytes[3];
	send_to_gimbal[13] = get_robot_id();
  send_to_gimbal[14] = 'e';
	
  USART1_Transmit_DMA(send_to_gimbal, PACK_SIZE);
}

#endif


#if ROBOT_BOARD  ==	gimbal_board 
//目前发送给底盘的数据：yaw电机由编码值转换的弧度值、imu的yaw值、pitch值、roll值、云台行为模式、射击模式
//0xFE为帧头，0xFD为帧尾
__attribute__((used))void send_data_to_chassis(void)
{
	static fp32 motor_yaw_now;							//yaw电机由编码值转换的弧度值
  static fp32* INS_now_ptr; 							//存储INS指针, 用于发送imu的yaw值和pitch值
	static uint8_t gimbal_behaviour_now;		//云台行为模式
	static uint8_t shoot_mode_now;					//射击模式
	
  motor_yaw_now = gimbal_control.gimbal_yaw_motor.radian_of_ecd;
  INS_now_ptr = get_INS_angle_point();	//0:yaw, 1:pitch, 2:roll 单位:rad
	extern gimbal_behaviour_e gimbal_behaviour;
	gimbal_behaviour_now = (uint8_t)gimbal_behaviour;
	extern shoot_control_t shoot_control; 
	shoot_mode_now = shoot_control.shoot_mode;
	
#if ROBOT_TYPE == Sentinel_robot	
	static int16_t order[4];
	static uint8_t mode[2];
	for(int8_t i=0;i<4;i++)
	{order[i] = gimbal_control.gimbal_rc_ctrl->rc.ch[i];}
  mode[0] = (uint8_t)gimbal_control.gimbal_rc_ctrl->rc.s[0];
	mode[1] = (uint8_t)gimbal_control.gimbal_rc_ctrl->rc.s[1];
	const static uint8_t length = 30;
#else	
	//length = 2(帧头和帧尾) + 4 + 4*3 + 1 + 1 = 20
	const static uint8_t length = 20;
#endif	
	static uint8_t data_chassis[length];		//发送给底盘的数据
	
	/*******************数据段*******************/
  data_chassis[0] = 0XFE;		//帧头
	
	// 提取每个字节
	uint32_t *ptr1 = (uint32_t*)&motor_yaw_now;
	data_chassis[1] = (*ptr1 >>  0) & 0xFF;			// 最低字节
	data_chassis[2] = (*ptr1 >>  8) & 0xFF;			// 第二个字节
	data_chassis[3] = (*ptr1 >> 16) & 0xFF;			// 第三个字节
	data_chassis[4] = (*ptr1 >> 24) & 0xFF;			// 最高字节
	
	uint32_t *ptr2 = (uint32_t*)INS_now_ptr;
	data_chassis[5] = (ptr2[0] >>  0) & 0xFF;		// 最低字节
	data_chassis[6] = (ptr2[0] >>  8) & 0xFF;		// 第二个字节
	data_chassis[7] = (ptr2[0] >> 16) & 0xFF;		// 第三个字节
	data_chassis[8] = (ptr2[0] >> 24) & 0xFF;		// 最高字节
  
	data_chassis[9]  = (ptr2[1] >>  0) & 0xFF;	// 最低字节
	data_chassis[10] = (ptr2[1] >>  8) & 0xFF;	// 第二个字节
	data_chassis[11] = (ptr2[1] >> 16) & 0xFF;	// 第三个字节
	data_chassis[12] = (ptr2[1] >> 24) & 0xFF;	// 最高字节
	
	data_chassis[13] = (ptr2[2] >> 	0) & 0xFF;	// 最低字节
	data_chassis[14] = (ptr2[2] >> 	8) & 0xFF;	// 第二个字节
	data_chassis[15] = (ptr2[2] >> 16) & 0xFF;	// 第三个字节
	data_chassis[16] = (ptr2[2] >> 24) & 0xFF;	// 最高字节
	
	data_chassis[17] = gimbal_behaviour_now;		// 单个字节
	
	data_chassis[18] = shoot_mode_now;					// 单个字节
	
#if ROBOT_TYPE == Sentinel_robot	
	
  uint16_t *ptr3 = (uint16_t*)order;
	data_chassis[19] = (ptr3[0] >>  0) & 0xFF;	// 最低字节
	data_chassis[20] = (ptr3[0] >>  8) & 0xFF;	// 第二个字节
	
	data_chassis[21] = (ptr3[1] >>  0) & 0xFF;	// 最低字节
	data_chassis[22] = (ptr3[1] >>  8) & 0xFF;	// 第二个字节
	
	data_chassis[23] = (ptr3[2] >>  0) & 0xFF;	// 最低字节
	data_chassis[24] = (ptr3[2] >>  8) & 0xFF;	// 第二个字节
	
	data_chassis[25] = (ptr3[3] >>  0) & 0xFF;	// 最低字节
	data_chassis[26] = (ptr3[3] >>  8) & 0xFF;	// 第二个字节
	
	data_chassis[27] = mode[0];
	data_chassis[28] = mode[1];
	
#endif

  data_chassis[length - 1] = 0XFD;  //帧尾
	/*******************************************/
	
	USART1_Transmit_DMA(data_chassis, length);	//DMA发送数据, 长度见上方 length
}

#endif

void message_task(void const *pvParameters)
{

    // 初始化延迟
    vTaskDelay(MESSAGE_TASK_INIT_TIME);
    while(1) {
      #if ROBOT_BOARD == gimbal_board
        send_data_to_chassis();
				vTaskDelay(MESSAGE_TASK_TIME);  // 每次循环的延迟 
      #endif // DEBUG
			
      #if ROBOT_BOARD == chassis_board
        send_to_gimbal();
				vTaskDelay(MESSAGE_TASK_TIME);
      #endif // DEBUG
      
    }
}

