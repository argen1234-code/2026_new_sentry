#include "referee.h"
#include "string.h"
#include "stdio.h"
#include "CRC8_CRC16.h"
#include "protocol.h"


/* 结构体变量声明区 */
frame_header_struct_t				referee_receive_header;
frame_header_struct_t				referee_send_header;
game_status_t								game_status;							//0x0001
game_result_t								game_result;              //0x0002
game_robot_HP_t							game_robot_HP;            //0x0003
event_data_t								event_data;               //0x0101
referee_warning_t						referee_warning;          //0x0104
dart_info_t									dart_info;                //0x0105
robot_status_t							robot_status;             //0x0201
power_heat_data_t						power_heat_data;          //0x0202
robot_pos_t									robot_pos;                //0x0203
buff_t											buff;                     //0x0204
hurt_data_t									hurt_data;                //0x0206
shoot_data_t								shoot_data;               //0x0207
projectile_allowance_t			projectile_allowance;     //0x0208
rfid_status_t								rfid_status;              //0x0209
dart_client_cmd_t						dart_client_cmd;          //0x020A
ground_robot_position_t			ground_robot_position;    //0x020B
radar_mark_data_t						radar_mark_data;          //0x020C
sentry_info_t								sentry_info;              //0x020D
radar_info_t								radar_info;               //0x020E
robot_interaction_data_t		robot_interaction_data;   //0x0301
custom_robot_data_t					custom_robot_data;        //0x0302
map_command_t								map_command;              //0x0303
remote_control_t						remote_control;           //0x0304
map_robot_data_t						map_robot_data;           //0x0305
custom_client_data_t				custom_client_data;       //0x0306
map_data_t									map_data_;                //0x0307
custom_info_t								custom_info;              //0x0308
robot_custom_data_t					robot_custom_data;        //0x0309





/**
	*	@brief				初始化裁判系统数据
	*/
void init_referee_data(void)
{
	memset(&referee_receive_header,		0,	sizeof(frame_header_struct_t		));
	memset(&referee_send_header,			0,  sizeof(frame_header_struct_t		));
	memset(&game_status,							0,  sizeof(game_status_t						));		//0x0001
	memset(&game_result,							0,  sizeof(game_result_t						));   //0x0002
	memset(&game_robot_HP, 						0,  sizeof(game_robot_HP_t					));   //0x0003
	memset(&event_data,								0,  sizeof(event_data_t							));   //0x0101
	memset(&referee_warning,					0,  sizeof(referee_warning_t				));   //0x0104
	memset(&dart_info,								0,  sizeof(dart_info_t							));   //0x0105
	memset(&robot_status,							0,  sizeof(robot_status_t						));   //0x0201
	memset(&power_heat_data,					0,  sizeof(power_heat_data_t				));   //0x0202
	memset(&robot_pos,								0,  sizeof(robot_pos_t							));   //0x0203
	memset(&buff,											0,  sizeof(buff_t										));   //0x0204
	memset(&hurt_data,								0,  sizeof(hurt_data_t							));   //0x0206
	memset(&shoot_data,								0,  sizeof(shoot_data_t							));   //0x0207
	memset(&projectile_allowance,			0,	sizeof(projectile_allowance_t		));   //0x0208
	memset(&rfid_status,							0,	sizeof(rfid_status_t						));   //0x0209
	memset(&dart_client_cmd,					0,	sizeof(dart_client_cmd_t				));   //0x020A
	memset(&ground_robot_position,		0,	sizeof(ground_robot_position_t	));   //0x020B
	memset(&radar_mark_data,					0,	sizeof(radar_mark_data_t				));   //0x020C
	memset(&sentry_info,							0,	sizeof(sentry_info_t						));   //0x020D
	memset(&radar_info,								0,	sizeof(radar_info_t							));   //0x020E
	memset(&robot_interaction_data,		0,	sizeof(robot_interaction_data_t	));   //0x0301
	memset(&custom_robot_data,				0,	sizeof(custom_robot_data_t			));   //0x0302
	memset(&map_command,							0,	sizeof(map_command_t						));   //0x0303
	memset(&remote_control,						0,	sizeof(remote_control_t					));   //0x0304
	memset(&map_robot_data,						0,	sizeof(map_robot_data_t					));   //0x0305
	memset(&custom_client_data,				0,	sizeof(custom_client_data_t			));   //0x0306
	memset(&map_data_,								0,	sizeof(map_data_t								));   //0x0307
	memset(&custom_info,							0,	sizeof(custom_info_t						));   //0x0308
	memset(&robot_custom_data,				0,	sizeof(robot_custom_data_t			));   //0x0309
}

/**
	*	@brief				裁判系统数据处理
	*	@param[in]		frame 指向从裁判系统接收到的原始数据帧的指针
	*/
void referee_handle_data(uint8_t *frame)
{
	uint16_t cmd_id = 0;
	uint8_t index = 0;

	memcpy(&referee_receive_header, frame, REF_PROTOCOL_HEADER_SIZE);

	index += REF_PROTOCOL_HEADER_SIZE;

	memcpy(&cmd_id, frame + index, REF_PROTOCOL_CMDID_SIZE);
	index += REF_PROTOCOL_CMDID_SIZE;

	switch(cmd_id)
	{
		case(GAME_STATUS_CMD_ID             ):{ memcpy(&game_status,						frame + index,	sizeof(game_status_t						)); break;}
		case(GAME_RESULT_CMD_ID             ):{ memcpy(&game_result,						frame + index,	sizeof(game_result_t						)); break;}
		case(GAME_ROBOT_HP_CMD_ID           ):{ memcpy(&game_robot_HP, 					frame + index,	sizeof(game_robot_HP_t					)); break;}
		case(EVENT_DATA_CMD_ID            	):{ memcpy(&event_data,							frame + index,	sizeof(event_data_t							)); break;}
		case(REFEREE_WARNING_CMD_ID         ):{ memcpy(&referee_warning,				frame + index,	sizeof(referee_warning_t				)); break;}
		case(DART_INFO_CMD_ID               ):{ memcpy(&dart_info,							frame + index,	sizeof(dart_info_t							)); break;}
		case(ROBOT_STATUS_CMD_ID            ):{ memcpy(&robot_status,						frame + index,	sizeof(robot_status_t						)); break;}
		case(POWER_HEAT_DATA_CMD_ID         ):{ memcpy(&power_heat_data,				frame + index,	sizeof(power_heat_data_t				)); break;}
		case(ROBOT_POS_CMD_ID               ):{ memcpy(&robot_pos,							frame + index,	sizeof(robot_pos_t							)); break;}
		case(BUFF_CMD_ID               			):{ memcpy(&buff,										frame + index,	sizeof(buff_t										)); break;}
		case(ROBOT_HURT_CMD_ID              ):{ memcpy(&hurt_data,							frame + index,	sizeof(hurt_data_t							)); break;}
		case(SHOOT_DATA_CMD_ID              ):{ memcpy(&shoot_data,							frame + index,	sizeof(shoot_data_t							)); break;}
		case(PROJECTILE_ALLOWANCE_CMD_ID    ):{ memcpy(&projectile_allowance,		frame + index,	sizeof(projectile_allowance_t		)); break;}
		case(RFID_STATUS_CMD_ID             ):{ memcpy(&rfid_status,						frame + index,	sizeof(rfid_status_t						)); break;}
		case(DART_CLIENT_CMD_ID             ):{ memcpy(&dart_client_cmd,				frame + index,	sizeof(dart_client_cmd_t				)); break;}
		case(GROUND_ROBOT_POSITION_CMD_ID   ):{ memcpy(&ground_robot_position,	frame + index,	sizeof(ground_robot_position_t	)); break;}
		case(RADAR_MARK_DATA_CMD_ID         ):{ memcpy(&radar_mark_data,				frame + index,	sizeof(radar_mark_data_t				)); break;}
		case(SENTRY_INFO_CMD_ID             ):{ memcpy(&sentry_info,						frame + index,	sizeof(sentry_info_t						)); break;}
		case(RADAR_INFO_CMD_ID              ):{ memcpy(&radar_info,							frame + index,	sizeof(radar_info_t							)); break;}
		case(ROBOT_INTERACTIVE_DATA_CMD_ID	):{ memcpy(&robot_interaction_data,	frame + index,	sizeof(robot_interaction_data_t	)); break;}
		case(CUSTOM_CONTROLLER_CMD_ID       ):{ memcpy(&custom_robot_data,			frame + index,	sizeof(custom_robot_data_t			)); break;}
		case(MAP_COMMAND_CMD_ID             ):{ memcpy(&map_command,						frame + index,	sizeof(map_command_t						)); break;}
		case(REMOTE_CONTROL_CMD_ID          ):{ memcpy(&remote_control,					frame + index,	sizeof(remote_control_t					)); break;}
		case(MAP_ROBOT_DATA_CMD_ID          ):{ memcpy(&map_robot_data,					frame + index,	sizeof(map_robot_data_t					)); break;}
		case(CUSTOM_CLIENT_DATA_CMD_ID      ):{ memcpy(&custom_client_data,			frame + index,	sizeof(custom_client_data_t			)); break;}
		case(MAP_DATA_CMD_ID                ):{ memcpy(&map_data_,							frame + index,	sizeof(map_data_t								)); break;}
		case(CUSTOM_INFO_CMD_ID             ):{ memcpy(&custom_info,						frame + index,	sizeof(custom_info_t						)); break;}
		case(ROBOT_CUSTOM_DATA_CMD_ID       ):{ memcpy(&robot_custom_data,			frame + index,	sizeof(robot_custom_data_t			)); break;}
		
		default:
		{
			break;
		}
	}
}

/**
	*	@brief				获取底盘当前功率与缓冲能量
	*	@param[out]		power		底盘当前最大可用功率
	*	@param[out]		buffer	底盘剩余缓冲能量
	*/
void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
	*power = (fp32)robot_status.chassis_power_limit;
	*buffer = (fp32)power_heat_data.buffer_energy;
}

/**
	*	@brief				获取机器人ID
	*/
uint8_t get_robot_id(void)
{
	return robot_status.robot_id;
}

/**
	*	@brief				获取底盘当前功率与缓冲能量
	*	@param[out]		heat_limit	机器人枪口热量上限
	*	@param[out]		heat				17mm 发射机构的枪口热量
	*/
void get_shoot_heat0_limit_and_heat0(uint16_t *heat_limit, uint16_t *heat)
{
	*heat_limit = robot_status.shooter_barrel_heat_limit;
	*heat = power_heat_data.shooter_17mm_1_barrel_heat;
}

/**
	*	@brief				获取底盘当前功率与缓冲能量
	*	@param[out]		heat_limit	机器人枪口热量上限
	*	@param[out]		heat				42mm 发射机构的枪口热量
	*/
void get_shoot_heat1_limit_and_heat1(uint16_t *heat_limit, uint16_t *heat)
{
	*heat_limit = robot_status.shooter_barrel_heat_limit;
	*heat = power_heat_data.shooter_42mm_barrel_heat;
}
