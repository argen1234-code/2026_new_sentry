/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
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

#ifndef CAN_BSP_H
#define CAN_BSP_H

#include "struct_typedef.h"
#include "main.h"
#include "fdcan.h"

#define CHASSIS_CAN hfdcan1  //4p
#define GIMBAL_CAN hfdcan2   //can1 2p

//和底盘用一条can，但是摩擦轮控制在云台板上
#define FRICTION_CAN hfdcan1
extern uint8_t              chassis_can_send_data[8];
extern uint8_t              gimbal_can_send_data[8];
extern uint8_t							MOTOR_Data[8];


////需要根据每个电机的不同来选择
////所以建议在送入发送函数之前进行限幅，
//#define _T_MIN -100.0f			//转矩最大值
//#define _T_MAX 100.0f			//转矩最小值

/*
//mit电机发送数据帧是看slave(不同模式对应id偏移)
//master id是电机反馈帧的id，电机发给单片机的(不受电机模式影响)
//can_id是电机接收的id，是单片机发给电机的（实际id = can_id + slave）
//
//举个例子，MIT_mode == 0x00，电机在MIT模式下，can_id 设置为0x08,master_id为MIT_M1_ID
//那么在发送给电机的时候选择mit模式，id为0x08（代码会自动计算发送的id， 0x00 + 0x08）
//在can中断回调函数就需要接受id为MIT_M1_ID的数据
//
//防止和其他品牌（类似与翎控）这种有固定id偏移的电机冲突，此处的 的地址做了一个偏移
*/
/* CAN send and receive ID */
typedef enum
{
		//不同模式电机的id偏移
		MIT_mode = 0x00,
    POS_mode =0X100,
		SPEED_mode = 0X200,

		//底盘板上的两条can线使用
    CAN_3508_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	
		CAN_6020_ALL_ID = 0x1FF,
		CAN_6020_M1_ID = 0x205,
    CAN_6020_M2_ID = 0x206,
    CAN_6020_M3_ID = 0x207,
    CAN_6020_M4_ID = 0x208,
		
	//云台的pitch,yaw,拨蛋电机
		CAN_GIMBAL_ALL_ID = 0x1FF,
    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,

		//摩擦轮电机,和底盘电机id一致
		CAN_FRICTION_ID = 0X200,
		CAN_FRIC1_MOTOR_ID = 0x201,
		CAN_FRIC2_MOTOR_ID = 0x202,
		
		CAN_FRIC3_MOTOR_ID = 0x203,
		CAN_FRIC4_MOTOR_ID = 0x204,
		
		//MIT电机的反馈id
    MIT_M1_ID = 0x51,
    MIT_M2_ID = 0x52,
    MIT_M3_ID = 0x53,
    MIT_M4_ID = 0x54,
    MIT_M5_ID = 0x55,
    MIT_M6_ID = 0x56,
		MIT_M7_ID = 0x57,
    MIT_M8_ID = 0x58,
		//不够就自己再写，最多到0x100
		


} can_msg_id_e;

//rm motor data
//dji电机结构体
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
		uint32_t last_fdb_time;
} motor_measure_t;

// clang-format on
//MIT电机结构体
//注意：大喵电机在上位机修改模式后，ID会被重置，需要重新设置(截至在2025年10月10日，2325电机上发生该现象)
typedef struct
{
	__packed struct{
    //原始数据
    int id;
    int state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    //计算后的数据
    float pos;
    float vel;
    float tor; //电机反馈的力矩
    float Kp;
    float Kd;
    float t_mos; //mos温度
    float t_motor; //电机温度
		uint32_t last_fdb_time; //电机反馈时间
	}fdb;
	__packed struct
	{
		fp32 KP;
		fp32 KD;
		fp32 POS;
		fp32 VEL;
		fp32 TOR;
	}set;
	__packed struct
	{
	fp32	P_MIN;
	fp32	P_MAX;
	fp32	V_MIN;
	fp32	V_MAX;
	fp32	KP_MIN;
	fp32	KP_MAX;
	fp32	KD_MIN;
	fp32	KD_MAX;
	fp32	T_MIN;
	fp32	T_MAX;
	}param;
} MITMeasure_t;

/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
extern void CAN_cmd_friction(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          返回摩擦轮1电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_fric1_motor_measure_point(void);

/**
  * @brief          返回摩擦轮2电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_fric2_motor_measure_point(void);

/**
  * @brief          返回摩擦轮3电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_fric1__motor_measure_point(void);

/**
  * @brief          返回摩擦轮4电机 3508电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_fric2__motor_measure_point(void);
/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_chassis_6020(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
/**
 * @brief                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           MIT模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  motor  电机结构体
 */
extern void CAN_cmd_MIT(FDCAN_HandleTypeDef *hcan,uint16_t id, MITMeasure_t motor);
/**
 * @brief  位置速度模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _pos   位置给定
 * @param  _vel   速度给定
 */
extern void CAN_cmd_POS(FDCAN_HandleTypeDef* hcan, uint16_t id, float _pos, float _vel);
/**
 * @brief  速度模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  _vel   速度给定
 */
extern void CAN_cmd_SPEED(FDCAN_HandleTypeDef* hcan, uint16_t id, float _vel);

extern void Motor_enable(FDCAN_HandleTypeDef *hcan, uint16_t id);

extern void RS_MOTOR_PRE(FDCAN_HandleTypeDef *hcan,uint16_t id);

extern void RS_Motor_MIT(FDCAN_HandleTypeDef *hcan,uint16_t id);
/**
 * @brief  这里对ID电机进行失能
 * @param  hcan   can总线     	
 * @param  id     电机id 
 * @param  void    	
 * @param  void      	
 */
extern void Motor_disable(FDCAN_HandleTypeDef *hcan, uint16_t id);

/**
 * @brief  返回MIT电机结构指针     	
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针     	
 */
extern MITMeasure_t *get_mit_motor_measure_point(uint8_t i);
/**
 * @brief  这里对ID电机保存当前位置为零点
 * @param  hcan   can总线     	
 * @param  id     电机id      
 * @param  void    	
 * @param  void      	
 */
extern void Motor_save_zero(FDCAN_HandleTypeDef *hcan, uint16_t id);

/**
* @brief      	ClearErr: 清除电机错误函数
* @param[in]    hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]    motor_id: 电机ID，指定目标电机
* @param[in]    mode_id:  模式ID，指定要清除错误的模式
* @retval     	void
* @details    	通过CAN总线向特定电机发送清除错误的命令。
*/
extern void Motor_clear_err(FDCAN_HandleTypeDef *hcan, uint16_t id);

extern void FDCAN1_Config(void);
extern void FDCAN2_Config(void);
extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x, float x_min, float x_max, int bits);
#endif
