/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       protocol.h
  * @brief      RoboMaster裁判系统串口协议 V1.9.0
	*
	*     000000000000000     00               00    00     00   00         00 
	*           00     0      00                00    00   00     00       00  
	*       00  00000        00000000000000      00  000000000   00000000000000
	*       00  00          00           00    00    000000000     00     00   
	*      00000000000     00  000000    00     000     00           000000    
	*     00    00   0000     00    00   00      00   000000           00      
	*       00000000000       00    00   00           000000           00      
	*       0   00    0       0000000 00 00       00    00       00000000000000
	*       00000000000       00       000       00 00000000000        00      
	*           00            00                000 00000000000        00      
	*           00  00        00          0    000      00             00      
	*      000000000000        00        000  000       00          00 00      
	*       00        00        000000000000            00            00       
	********************************************************************************/
	
#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "main.h"

#define HEADER_SOF										0xA5
#define REF_PROTOCOL_HEADER_SIZE      5
#define REF_PROTOCOL_CMDID_SIZE       2
#define REF_PROTOCOL_CRC16_SIZE       2
#define REF_PROTOCOL_FRAME_MAX_SIZE   192
#define REF_HEADER_CRC_CMDID_LEN      (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + REF_PROTOCOL_CMDID_SIZE)


#pragma pack(push, 1)

typedef struct
{
  uint8_t 	SOF;					//数据帧起始字节，固定值为0xA5
  uint16_t	data_length;	//数据帧中data的长度
  uint8_t		seq;					//包序号
  uint8_t		CRC8;					//帧头CRC8 校验
	
} frame_header_struct_t;

typedef enum
{
	GAME_STATUS_CMD_ID                = 0x0001,
	GAME_RESULT_CMD_ID                = 0x0002,
	GAME_ROBOT_HP_CMD_ID              = 0x0003,
	EVENT_DATA_CMD_ID               	= 0x0101,
	REFEREE_WARNING_CMD_ID            = 0x0104,
	DART_INFO_CMD_ID                  = 0x0105,
	ROBOT_STATUS_CMD_ID               = 0x0201,
	POWER_HEAT_DATA_CMD_ID            = 0x0202,
	ROBOT_POS_CMD_ID                  = 0x0203,
	BUFF_CMD_ID       								= 0x0204,
	ROBOT_HURT_CMD_ID                 = 0x0206,
	SHOOT_DATA_CMD_ID             		= 0x0207,
	PROJECTILE_ALLOWANCE_CMD_ID       = 0x0208,
	RFID_STATUS_CMD_ID                = 0x0209,
	DART_CLIENT_CMD_ID                = 0x020A,
	GROUND_ROBOT_POSITION_CMD_ID      = 0x020B,
	RADAR_MARK_DATA_CMD_ID            = 0x020C,
	SENTRY_INFO_CMD_ID                = 0x020D,
	RADAR_INFO_CMD_ID                 = 0x020E,
	ROBOT_INTERACTIVE_DATA_CMD_ID  		= 0x0301,
	CUSTOM_CONTROLLER_CMD_ID          = 0x0302,
	MAP_COMMAND_CMD_ID                = 0x0303,
	REMOTE_CONTROL_CMD_ID             = 0x0304,
	MAP_ROBOT_DATA_CMD_ID             = 0x0305,
	CUSTOM_CLIENT_DATA_CMD_ID         = 0x0306,
	MAP_DATA_CMD_ID                   = 0x0307,
	CUSTOM_INFO_CMD_ID                = 0x0308,
	ROBOT_CUSTOM_DATA_CMD_ID          = 0x0309,
	
} referee_cmd_id_e;

typedef struct //0x0001
{
  uint8_t game_type : 4;         //比赛类型
  uint8_t game_progress : 4;     //当前比赛阶段
  uint16_t stage_remain_time;    //当前阶段剩余时间，单位：秒
  uint64_t SyncTimeStamp;        //UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效
} game_status_t;

typedef struct //0x0002
{
  uint8_t winner; //胜利方  0：平局；1：红方胜利；2：蓝方胜利
} game_result_t;

typedef struct //0x0003
{
  uint16_t red_1_robot_HP;    //红 1 英雄机器人血量
  uint16_t red_2_robot_HP;    //红 2 工程机器人血量
  uint16_t red_3_robot_HP;    //红 3 步兵机器人血量
  uint16_t red_4_robot_HP;    //红 4 步兵机器人血量
  uint16_t reserved_1;        //保留
  uint16_t red_7_robot_HP;    //红 7 哨兵机器人血量
  uint16_t red_outpost_HP;    //红方前哨站血量
  uint16_t red_base_HP;       //红方基地血量
  uint16_t blue_1_robot_HP;   //蓝 1 英雄机器人血量
  uint16_t blue_2_robot_HP;   //蓝 2 工程机器人血量
  uint16_t blue_3_robot_HP;   //蓝 3 步兵机器人血量
  uint16_t blue_4_robot_HP;   //蓝 4 步兵机器人血量
  uint16_t reserved_2;        //保留
  uint16_t blue_7_robot_HP;   //蓝 7 哨兵机器人血量
  uint16_t blue_outpost_HP;   //蓝方前哨站血量
  uint16_t blue_base_HP;      //蓝方基地血量
} game_robot_HP_t;

typedef struct //0x0101
{
  uint32_t event_data; //比赛时发生的事件
} event_data_t;

typedef struct //0x0104
{
  uint8_t level;              // 警告等级
  uint8_t offending_robot_id; // 违规机器人ID
  uint8_t count;              // 违规次数
} referee_warning_t;

typedef struct //0x0105
{
  uint8_t dart_remaining_time; // 飞镖剩余时间
  uint16_t dart_info;          // 飞镖信息
} dart_info_t;

typedef struct //0x0201
{
  uint8_t robot_id;                                 //本机器人 ID
  uint8_t robot_level;                              //机器人等级
  uint16_t current_HP;                              //机器人当前血量
  uint16_t maximum_HP;                              //机器人血量上限
  uint16_t shooter_barrel_cooling_value;            //机器人枪口热量每秒冷却值
  uint16_t shooter_barrel_heat_limit;               //机器人枪口热量上限
  uint16_t chassis_power_limit;                     //机器人底盘功率上限
  uint8_t power_management_gimbal_output : 1;       //gimbal口输出：0 为无输出，1 为 24V 输出
  uint8_t power_management_chassis_output : 1;      //chassis 口输出：0 为无输出，1 为 24V 输出
  uint8_t power_management_shooter_output : 1;      //shooter 口输出：0 为无输出，1 为 24V 输出
} robot_status_t;

typedef struct //0x0202
{
  uint16_t reserved_1;                    //保留
  uint16_t reserved_2;                    //保留
  float reserved_3;                       //保留
  uint16_t buffer_energy;                 //缓冲能量(单位: J)
  uint16_t shooter_17mm_1_barrel_heat;    //第1个17mm 发射机构的射击热量
  uint16_t shooter_17mm_2_barrel_heat;    //第2个17mm 发射机构的射击热量
  uint16_t shooter_42mm_barrel_heat;      //42mm 发射机构的射击热量
} power_heat_data_t;

typedef struct //0x0203
{
  float x;     //本机器人位置 x 坐标，单位：m
  float y;     //本机器人位置 y 坐标，单位：m
  float angle; //本机器人测速模块的朝向，单位：度。正北为 0 度
} robot_pos_t;

typedef struct //0x0204
{
  uint8_t recovery_buff;      //机器人回血增益（百分比）
  uint8_t cooling_buff;       //机器人射击热量冷却倍率（直接值）
  uint8_t defence_buff;       //机器人防御增益（百分比）
  uint8_t vulnerability_buff; //机器人负防御增益（百分比）
  uint16_t attack_buff;       //机器人攻击增益（百分比）
  uint8_t remaining_energy;   //机器人剩余能量值反馈
} buff_t;

typedef struct //0x0206
{
  uint8_t armor_id : 4;             //装甲模块或测速模块的ID编号
  uint8_t HP_deduction_reason : 4;  //血量变化类型
} hurt_data_t;

typedef struct //0x0207
{
  uint8_t bullet_type;          //弹丸类型: 1: 17mm 弹丸; 2: 42mm 弹丸
  uint8_t shooter_number;       //发射机构 ID: 1: 第1个17mm; 2: 第2个17mm; 3: 42mm
  uint8_t launching_frequency;  //弹丸射速（单位: Hz）
  float initial_speed;          //弹丸初速度（单位: m/s）
} shoot_data_t;

typedef struct //0x0208
{
  uint16_t projectile_allowance_17mm;  //17mm弹丸允许发弹量
  uint16_t projectile_allowance_42mm;  //42mm弹丸允许发弹量
  uint16_t remaining_gold_coin;        //剩余金币数量
  uint16_t projectile_allowance_fortress; //堡垒增益点提供的储备17mm弹丸允许发弹量
} projectile_allowance_t;

typedef struct //0x0209
{
  uint32_t rfid_status; //RFID模块状态
} rfid_status_t;

typedef struct //0x020A
{
  uint8_t dart_launch_opening_status; //飞镖发射站的状态
  uint8_t reserved;                   //保留位
  uint16_t target_change_time;        //切换击打目标时的比赛剩余时间，单位：秒
  uint16_t latest_launch_cmd_time;    //最后一次操作手确定发射指令时的比赛剩余时间，单位：秒
} dart_client_cmd_t;

typedef struct //0x020B
{
  float hero_x;           //已方英雄机器人位置x轴坐标，单位：m
  float hero_y;           //已方英雄机器人位置y轴坐标，单位：m
  float engineer_x;       //已方工程机器人位置x轴坐标，单位：m
  float engineer_y;       //已方工程机器人位置y轴坐标，单位：m
  float standard_3_x;     //已方3号步兵机器人位置x轴坐标，单位：m
  float standard_3_y;     //已方3号步兵机器人位置y轴坐标，单位：m
  float standard_4_x;     //已方4号步兵机器人位置x轴坐标，单位：m
  float standard_4_y;     //已方4号步兵机器人位置y轴坐标，单位：m
  float reserved_1;       //保留位
  float reserved_2;       //保留位
} ground_robot_position_t;

typedef struct //0x020C
{
  uint8_t mark_progress; //雷达标记进度数据
} radar_mark_data_t;

typedef struct //0x020D
{
  uint32_t sentry_info;   //哨兵自主决策信息
  uint16_t sentry_info_2; //哨兵自主决策信息2
} sentry_info_t;

typedef struct //0x020E
{
  uint8_t radar_info; //雷达自主决策信息
} radar_info_t;

typedef struct //0x0301
{
  uint16_t data_cmd_id;   //子内容 ID
  uint16_t sender_id;     //发送者 ID
  uint16_t receiver_id;   //接收者 ID
  uint8_t user_data[112]; //内容数据段，最大112字节
} robot_interaction_data_t;

typedef struct //0x0302
{
  uint8_t data[30]; //自定义数据
} custom_robot_data_t;

typedef struct //0x0303
{
  float target_position_x;    //目标位置 x 轴坐标，单位 m
  float target_position_y;    //目标位置 y 轴坐标，单位 m
  uint8_t cmd_keyboard;       //云台手按下的键盘按键通用键值
  uint8_t target_robot_id;    //对方机器人 ID
  uint16_t cmd_source;        //信息来源 ID
} map_command_t;

typedef struct //0x0304
{
  int16_t mouse_x;            //鼠标 x 轴移动速度
  int16_t mouse_y;            //鼠标 y 轴移动速度
  int16_t mouse_z;            //鼠标滚轮移动速度
  uint8_t left_button_down;   //鼠标左键是否按下
  uint8_t right_button_down;  //鼠标右键是否按下
  uint16_t keyboard_value;    //键盘按键信息
  uint16_t reserved;          //保留位
} remote_control_t;

typedef struct //0x0305
{
  uint16_t hero_position_x;       //英雄机器人 x 位置坐标，单位：cm
  uint16_t hero_position_y;       //英雄机器人 y 位置坐标，单位：cm
  uint16_t engineer_position_x;   //工程机器人 x 位置坐标，单位：cm
  uint16_t engineer_position_y;   //工程机器人 y 位置坐标，单位：cm
  uint16_t infantry_3_position_x; //3号步兵机器人 x 位置坐标，单位：cm
  uint16_t infantry_3_position_y; //3号步兵机器人 y 位置坐标，单位：cm
  uint16_t infantry_4_position_x; //4号步兵机器人 x 位置坐标，单位：cm
  uint16_t infantry_4_position_y; //4号步兵机器人 y 位置坐标，单位：cm
  uint16_t infantry_5_position_x; //5号步兵机器人 x 位置坐标，单位：cm
  uint16_t infantry_5_position_y; //5号步兵机器人 y 位置坐标，单位：cm
  uint16_t sentry_position_x;     //哨兵机器人 x 位置坐标，单位：cm
  uint16_t sentry_position_y;     //哨兵机器人 y 位置坐标，单位：cm
} map_robot_data_t;

typedef struct //0x0306
{
  uint16_t key_value;         //键盘键值
  uint16_t x_position : 12;   //鼠标 X 轴像素位置
  uint16_t mouse_left : 4;    //鼠标左键状态
  uint16_t y_position : 12;   //鼠标 Y 轴像素位置
  uint16_t mouse_right : 4;   //鼠标右键状态
  uint16_t reserved;          //保留位
} custom_client_data_t;

typedef struct //0x0307
{
  uint8_t intention;          //意图：1:攻击; 2:防守; 3:移动
  uint16_t start_position_x;  //路径起点 x 轴坐标，单位：dm
  uint16_t start_position_y;  //路径起点 y 轴坐标，单位：dm
  int8_t delta_x[49];         //路径点 x 轴增量数组，单位：dm
  int8_t delta_y[49];         //路径点 y 轴增量数组，单位：dm
  uint16_t sender_id;         //发送者 ID
} map_data_t;

typedef struct //0x0308
{
  uint16_t sender_id;         //发送者的 ID
  uint16_t receiver_id;       //接收者的 ID
  uint8_t user_data[30];      //字符数据，以 utf-16 格式编码
} custom_info_t;

typedef struct //0x0309
{
  uint8_t data[30]; //自定义数据
} robot_custom_data_t;

#pragma pack(pop)


#endif // ROBOMASTER_PROTOCOL_H
