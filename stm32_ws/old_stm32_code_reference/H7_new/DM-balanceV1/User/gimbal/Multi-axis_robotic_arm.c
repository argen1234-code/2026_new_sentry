#include "Multi-axis_robotic_arm.h"
#include "cmsis_os.h"
#if ROBOT_GIMBAL == multi_axis_robotic_arm
/*
机械臂电机选型
                    J0： RS03
                    J1： RS04
                    J2： DM4340
                    J3： RS00
                    J4：
                    J5：
                    末端：

机械臂相关的一些方向定义
    - 机械臂水平向前时设置J0关节为0位置
    - 机械臂竖直向上时设置J1 J2 J3关节为0位置

    - 定义机械臂水平向前时的J0关节位置为0，从上往下看，逆时针为正方向
    - 定义机械臂竖直向上时的J1 J2关节位置为0，从机械臂右侧看，逆时针为正方向（注：J1 J2关节的合位置为联动位置）
    - 定义机械臂J3水平(同步带位于两侧)时的J3关节位置为0，从吸盘方向看，逆时针为正方向
    - 定义J4为末端机构右侧（上视，J3向前）电机，J5为末端机构左侧电机

    - 定义虚拟J4关节用来衡量末端机构的pitch, 虚拟J5关节用来衡量末端机构的roll
    - 定义J4正方向为：当J3归中时，从机械臂右侧看，逆时针为正方向
    - 定义J5正方向为：当J3归中时，从机械臂前方看，逆时针为正方向
    - vj4和j4 j5的关系：vj4 = (j4 - j5)/2

机械臂控制
    左拨杆 
          上：自定义控制器跟随模式
          中：安全模式
          下：RC模式
    右拨杆
          上：J1转动(rs04)
          中：J2转动(4340)
          下：J3转动(rs00)
*/
#include <string.h>
#include "remote_control.h"
#include "INS_task.h"
#include "can_bsp.h"
#include "arm_math.h"

// MIT command IDs (slave IDs) for each joint; adjust to wiring
#ifndef ARM_MIT_CMD_ID0
#define ARM_MIT_CMD_ID0 0x00
#define ARM_MIT_CMD_ID1 0x01
#define ARM_MIT_CMD_ID2 0x02
#define ARM_MIT_CMD_ID3 0x03
#define ARM_MIT_CMD_ID4 0x04
#define ARM_MIT_CMD_ID5 0x05
#endif

#define JOINT_TORQUE_MORE_OFFSET              ((uint8_t)1 << 0)  // 关节电机输出力矩过大偏移量
#define CUSTOM_CONTROLLER_DATA_ERROR_OFFSET   ((uint8_t)1 << 1)  // 自定义控制器数据异常偏移量
#define DBUS_ERROR_OFFSET    ((uint8_t)1 << 2)  // dbus错误偏移量
#define IMU_ERROR_OFFSET     ((uint8_t)1 << 3)  // imu错误偏移量
#define FLOATING_OFFSET      ((uint8_t)1 << 4)  // 悬空状态偏移量
#define MOTOR_OFFLINE      ((uint8_t)1 << 5)  // 电机掉线
#define RAPAM_ERROR      ((uint8_t)1 << 6)  // 设置参数错误


arm_joint_e ARM_JOINT_BEHAVIOUR;
typedef enum
{
    ARM_MODE_IDLE = 0,
    ARM_MODE_HOLD,
    ARM_MODE_RC,
    ARM_MODE_SELF
} arm_mode_e;

static const uint16_t arm_cmd_id_table[ARM_JOINT_NUM] = {
    ARM_MIT_CMD_ID0, ARM_MIT_CMD_ID1, ARM_MIT_CMD_ID2,
    ARM_MIT_CMD_ID3, ARM_MIT_CMD_ID4, ARM_MIT_CMD_ID5};
static const fp32 motor_limit_table[ARM_JOINT_NUM][2] = {
	J0_MIN_ANLE, J0_MAX_ANGLE,
	J1_MIN_ANLE, J1_MAX_ANGLE,
	J2_MIN_ANLE, J2_MAX_ANGLE,
	J3_MIN_ANLE, J3_MAX_ANGLE,
	J4_MIN_ANLE, J4_MAX_ANGLE,
	J5_MIN_ANLE, J5_MAX_ANGLE,											 
};

static inline float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

//自定义遥控器或DT7输入的数据
void arm_update_control(gimbal_control_t *add_angle);

//更新数据
__attribute__((used)) void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    (void)feedback_update;
    //从顶部开始计算重力补偿力矩
     for(int16_t i = ARM_JOINT_NUM - 1; i >= 0; i--)
    {
        //检测电机是否在线
        if(HAL_GetTick() - feedback_update->joint_motor[i].motor_measure->last_fdb_time > TIMEOUT)
        {
            feedback_update->joint_motor[i].online = OFFLINE;
            feedback_update->error_code |= MOTOR_OFFLINE;
        }
        else
        {
            feedback_update->joint_motor[i].online = ONLINE;
        }
//        //计算当前臂重力补偿 mg * l * cos
//        feedback_update->multi_arm_set[i][2] = feedback_update->multi_arm_params[i][0] * feedback_update->multi_arm_params[i][1] * arm_cos_f32(feedback_update->joint_motor[i].motor_measure->pos - feedback_update->joint_motor[i].zero_offset);

//        //重力补偿,f=mg*l*cos,把所有的电机全部加起来给下面那个电机力矩
//        for(int16_t j = ARM_JOINT_NUM - 1; j > i; j--)
//        {
//            feedback_update->multi_arm_set[i][2] += feedback_update->multi_arm_set[j][2];
//        }
    }

    //遥控器掉线检测
    if((HAL_GetTick() - feedback_update->gimbal_rc_ctrl->last_fdb) > TIMEOUT)
    {
        feedback_update->error_code |= DBUS_ERROR_OFFSET;
    }
    //电机力矩过大检测
    if(feedback_update->joint_motor[0].motor_measure->tor > 15.0f || feedback_update->joint_motor[1].motor_measure->tor > 15.0f)
    {
        feedback_update->error_code |= JOINT_TORQUE_MORE_OFFSET;
    }
    //参数检测,若位置不为0，但是KD为0，会导致电机震荡
    for(uint8_t i = 0 ; i < ARM_JOINT_NUM; i++)
    {
        if(!feedback_update->joint_pid[i].Kd && !feedback_update->multi_arm_set[i][0])
        {
            feedback_update->error_code |= RAPAM_ERROR;
        }
    }
    //初始化检测，速度低于阈值之后开始计时
    if(ARM_JOINT_BEHAVIOUR == ARM_INIT)
    {
        if(fabsf(feedback_update->joint_motor[0].motor_measure->vel) < INIT_VEL * 0.6f &&
           fabsf(feedback_update->joint_motor[0].motor_measure->vel) < INIT_VEL * 0.6f  )
        {
            feedback_update->init_time++;
        }
        else
        {
            feedback_update->init_time = 0;
        }
    }
        
}
__attribute__((used)) void gimbal_init(gimbal_control_t *init)
{
    (void)init;
    osDelay(1000);
    static fp32 pid_table[ARM_JOINT_NUM][3] = 
    {
        J0_KP, 0.0f, J0_KD,
        J1_KP, 0.0f, J1_KD,
        J2_KP, 0.0f, J2_KD,
        J3_KP, 0.0f, J3_KD,
        J4_KP, 0.0f, J4_KD,
        J5_KP, 0.0f, J5_KD,
    };
    //获取遥控器数据
    init->gimbal_rc_ctrl = get_remote_control_point();
    //获取IMU的数据
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();
    init->init_flag = 0;
    init->init_time = 0;
    //电机初始化
    for(uint8_t i = 0; i <  ARM_JOINT_NUM; i++)
    {
         init->joint_motor[i].online = ONLINE;
        //获取电机结构体指针
        init->joint_motor[i].motor_measure = get_mit_motor_measure_point(i);
				init->joint_motor[i].motor_measure = get_mit_motor_measure_point(i);
        //初始化电机PID,不需要使用这个PID计算，只需要发送即可
        PID_init(&init->joint_pid[i], PID_POSITION, pid_table[i], 0.0f, 0.0f);
			
				//设置电机限幅
				init->joint_motor[i].min_angle = motor_limit_table[i][0];
        init->joint_motor[i].max_angle = motor_limit_table[i][1];
        init->joint_motor[i].zero_offset = (init->joint_motor[i].max_angle + init->joint_motor[i].min_angle) / 2;
        //电机使能
        if(init->joint_motor[i].motor_measure->id == 0x00)
        {
            Motor_enable(&GIMBAL_CAN, i + 1);
        }
        else
        {
            Motor_enable(&GIMBAL_CAN, init->joint_motor[i].motor_measure->id + 1);
        }
				//初始化set
        init->multi_arm_set[i][0] = init->multi_arm_set[i][1] = init->multi_arm_set[i][2] = 0.0f;
				init->multi_arm_cmd[i][0] = init->multi_arm_cmd[i][1] = init->multi_arm_cmd[i][2] = 0.0f;
    }
    //gimbal_feedback_update(init);
    
}
//设置机械臂模式，获取控制值
__attribute__((used)) void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (!set_mode)
    {
        ARM_JOINT_BEHAVIOUR = ARM_NONE;
        return;
    }

    //开关控制 云台状态
    if (switch_is_mid(set_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]) )
    {
        ARM_JOINT_BEHAVIOUR = ARM_NONE;
    }
	else if(switch_is_down(set_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
	{
		ARM_JOINT_BEHAVIOUR = ARM_RC;
	}
    else if (switch_is_up(set_mode->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        ARM_JOINT_BEHAVIOUR = ARM_HOLD;
    }
    //有错误马上关
//    if(set_mode->error_code)
//    {
//        ARM_JOINT_BEHAVIOUR = ARM_NONE;
//    }

//    //如果模式不是无力模式，初始化未完成进入初始化模式
//    if(ARM_JOINT_BEHAVIOUR != ARM_NONE && !set_mode->init_flag )
//    {
//        ARM_JOINT_BEHAVIOUR = ARM_INIT;
//    }
    //完成初始化之后，初始化标志位置1，退出初始化模式
    if(ARM_JOINT_BEHAVIOUR == ARM_INIT && set_mode->init_time > 200)
    {
        set_mode->init_flag = 1;
    }
    //获取不同模式的控制值
    arm_update_control(set_mode);
}
//切换状态机
__attribute__((used)) void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    (void)gimbal_mode_change;
    static arm_joint_e last_ARM_JOINT_BEHAVIOUR;
    //模式切换后全部控制数据清空,防止出现奇奇怪怪的数值
    if(last_ARM_JOINT_BEHAVIOUR != ARM_NONE && ARM_JOINT_BEHAVIOUR == ARM_NONE)
    {
        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        {
            //角度设为0.0,速度也是0
            gimbal_mode_change->multi_arm_set[i][0] = 0.0f;
            gimbal_mode_change->multi_arm_set[i][1] = 0.0f;
        }
    }
    if(last_ARM_JOINT_BEHAVIOUR != ARM_RC && ARM_JOINT_BEHAVIOUR == ARM_RC)
    {
        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        {
            //角度设为0.0,速度也是0
            gimbal_mode_change->multi_arm_set[i][0] = 0.0f;
            gimbal_mode_change->multi_arm_set[i][1] = 0.0f;
        }
    }
    if(last_ARM_JOINT_BEHAVIOUR != ARM_SELF && ARM_JOINT_BEHAVIOUR == ARM_SELF)
    {
        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        {
            //角度设为0.0,速度也是0
            gimbal_mode_change->multi_arm_set[i][0] = 0.0f;
            gimbal_mode_change->multi_arm_set[i][1] = 0.0f;
        }
    }
	last_ARM_JOINT_BEHAVIOUR = ARM_JOINT_BEHAVIOUR;
	
}

//计算控制量set_control->multi_arm_set,不操作的话直接return
__attribute__((used)) void gimbal_set_control(gimbal_control_t *set_control)
{
    (void)set_control;
    //任务执行周期
    static fp32 dt = GIMBAL_CONTROL_TIME /1000.0f;
    if(ARM_JOINT_BEHAVIOUR == ARM_NONE)
    {
        return;
    }
    else if(ARM_JOINT_BEHAVIOUR == ARM_HOLD)
    {
        return;
    }
    else if(ARM_JOINT_BEHAVIOUR == ARM_INIT)
    {
        return;
    }
    else if(ARM_JOINT_BEHAVIOUR == ARM_RC)
    {
         // for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        // {
        //     //速度为0，此时电机KD纯阻尼
        //     set_control->multi_arm_set[i][1] = 0.0f;
        // }
        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        {
            //限幅
            //set_control->multi_arm_set[i][0] = clampf(set_control->multi_arm_set[i][0], set_control->joint_motor[i].min_angle, set_control->joint_motor[i].max_angle);
						
						set_control->multi_arm_cmd[i][1] = set_control->multi_arm_set[i][1];
            set_control->multi_arm_cmd[i][0] = clampf(set_control->multi_arm_set[i][0] + set_control->joint_motor[i].motor_measure->pos, set_control->joint_motor[i].min_angle, set_control->joint_motor[i].max_angle);
        }
        
    }
    else if(ARM_JOINT_BEHAVIOUR == ARM_SELF)
    {

    }

}
__attribute__((used)) void gimbal_send_cmd(gimbal_control_t *control_send)
{
//    //电机离线或者控制模式设置
//    if(ARM_JOINT_BEHAVIOUR ==ARM_NONE)
//    {
//        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
//        {
//            CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
//        }

//    }
//    //保持机械臂姿态，方便测出限位位置
//    else if (ARM_JOINT_BEHAVIOUR == ARM_HOLD)
//    {
//        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
//        {
//            CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[i], 0.0f, 0.0f, 0.0f, 0.0f, control_send->multi_arm_set[i][2]);
//        }
//    }
//    else if(ARM_JOINT_BEHAVIOUR == ARM_SELF)
//    {
//        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
//        {
//            CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[i], control_send->multi_arm_set[i][0], control_send->multi_arm_set[i][1], control_send->joint_pid[i].Kp, control_send->joint_pid[i].Kd, control_send->multi_arm_set[i][2]);
//        }
//    }
//    else if(ARM_JOINT_BEHAVIOUR == ARM_RC)
//    {
//        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
//        {
//            CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[i], control_send->multi_arm_set[i][0], control_send->multi_arm_set[i][1], control_send->joint_pid[i].Kp, control_send->joint_pid[i].Kd, control_send->multi_arm_set[i][2]);
//        }
//    }
//    else if(ARM_JOINT_BEHAVIOUR == ARM_INIT)
//    {
//        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
//        {
//            CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[i], 0.0f, control_send->multi_arm_set[i][1], 0.0f, ARM_KD_DEFAULT, control_send->multi_arm_set[i][2]);
//        }
//    }

    if(ARM_JOINT_BEHAVIOUR ==ARM_NONE)
    {
				CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[1], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
				CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[2], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		}
		else if(ARM_JOINT_BEHAVIOUR == ARM_RC)
    {                                                                                                                                                                                                               
						CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[1], control_send->multi_arm_cmd[1][0], control_send->multi_arm_cmd[1][1], control_send->joint_pid[1].Kp, control_send->joint_pid[1].Kd, control_send->multi_arm_cmd[1][2]);
            CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[2], control_send->multi_arm_cmd[2][0], control_send->multi_arm_cmd[2][1], control_send->joint_pid[2].Kp, control_send->joint_pid[2].Kd, control_send->multi_arm_cmd[2][2]);
    }
//				CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[1], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
//				CAN_cmd_MIT(&GIMBAL_CAN, arm_cmd_id_table[2], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

__attribute__((used)) void gimbal_control_loop(gimbal_control_t *control_loop)
{
    (void)control_loop;

}

__attribute__((used)) void set_cali_gimbal_hook(const uint16_t yaw_offset,
                                                const uint16_t pitch_offset,
                                                const fp32 max_yaw,
                                                const fp32 min_yaw,
                                                const fp32 max_pitch,
                                                const fp32 min_pitch)
{

}

__attribute__((used)) bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset,
                                                  uint16_t *pitch_offset,
                                                  fp32 *max_yaw,
                                                  fp32 *min_yaw,
                                                  fp32 *max_pitch,
                                                  fp32 *min_pitch)
{

}
void arm_update_control(gimbal_control_t *add_angle)
{
    if(ARM_JOINT_BEHAVIOUR == ARM_NONE)
    {
        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        {
            //add角度设为0.0,速度也是0
            add_angle->multi_arm_set[i][0] = 0.0f;
        }
        return;
    }
    else if(ARM_JOINT_BEHAVIOUR == ARM_RC)
    {  
        //设置不同电机的位置,速度在set_mode里面计算
        if(switch_is_down(gimbal_control.gimbal_rc_ctrl->rc.s[ARM_RC_MODE_CHANNEL]))
        {
            add_angle->multi_arm_set[0][0] = (gimbal_control.gimbal_rc_ctrl->rc.ch[HORI_CHANNEL] * 0.001f);
            add_angle->multi_arm_set[1][0] = (gimbal_control.gimbal_rc_ctrl->rc.ch[VERT_CHANNEL] * 0.001f);
        }                                                                                             
        if(switch_is_mid(gimbal_control.gimbal_rc_ctrl->rc.s[ARM_RC_MODE_CHANNEL]))                   
        {                                                                                             
            add_angle->multi_arm_set[2][0] = (gimbal_control.gimbal_rc_ctrl->rc.ch[HORI_CHANNEL] * 0.001f);
            add_angle->multi_arm_set[3][0] = (gimbal_control.gimbal_rc_ctrl->rc.ch[VERT_CHANNEL] * 0.001f);
        }                                                                                             
        if(switch_is_up(gimbal_control.gimbal_rc_ctrl->rc.s[ARM_RC_MODE_CHANNEL]))                    
        {                                                                                             
            add_angle->multi_arm_set[4][0] = (gimbal_control.gimbal_rc_ctrl->rc.ch[HORI_CHANNEL] * 0.001f);
            add_angle->multi_arm_set[5][0] = (gimbal_control.gimbal_rc_ctrl->rc.ch[VERT_CHANNEL] * 0.001f);
        }
				return;

    }
    else if(ARM_JOINT_BEHAVIOUR == ARM_SELF)
    {
				return;
    }
    else if(ARM_JOINT_BEHAVIOUR == ARM_INIT)
    {
        for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
        {
            add_angle->multi_arm_set[i][0] = 0.0f;
            add_angle->multi_arm_set[i][1] = INIT_VEL;
        }
				return;
    }


    // for(uint8_t i = 0; i < ARM_JOINT_NUM; i++)
    //     {
    //         add_angle->multi_arm_set[i] = 0.0f;
    //     }
}
#endif  // ROBOT_GIMBAL == multi_axis_robotic_arm
