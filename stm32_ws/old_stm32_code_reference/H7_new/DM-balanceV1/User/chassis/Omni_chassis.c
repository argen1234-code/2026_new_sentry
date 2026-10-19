#include "Omni_chassis.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"
#if ROBOT_CHASSIS == Omni_wheel  
#include "observer.h"
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "can_bsp.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
//#include "iwdg.h"
#include "slip_control.h"
#include "robot_param.h"


/**
  * @brief          "chassis_move" valiable initialization, include pid initialization, remote control data point initialization, 3508 chassis motors
  *                 data point initialization, gimbal motor data point initialization, and gyro sensor angle point initialization.
  * @param[out]     chassis_move_init: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
__attribute__((used))void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //chassis motor speed PID
    //底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    
    //chassis angle PID
    //底盘角度pid值
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    uint8_t i;

    //in beginning， chassis mode is raw 
    //底盘开机状态为原始
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //get remote control point
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    //get gyro sensor euler angle point
    //获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //get gimbal motor data point
    //获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    
    //get chassis motor data point,  initialize motor speed PID
    //获取底盘电机数据指针，初始化PID 
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
        PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //initialize angle PID
    //初始化角度PID
    PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    
    //first order low-pass filter  replace ramp function
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //max and min speed
    //最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //update data
    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          set chassis control mode, mainly call 'chassis_behaviour_mode_set' function
  * @param[out]     chassis_move_mode: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制模式，主要在'chassis_behaviour_mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
__attribute__((used))void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
    //in file "chassis_behaviour.c"
    chassis_behaviour_mode_set(chassis_move_mode);
}

/**
  * @brief          when chassis mode change, some param should be changed, suan as chassis yaw_set should be now chassis yaw
  * @param[out]     chassis_move_transit: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
__attribute__((used))void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    //change to follow gimbal angle mode
    //切入跟随云台模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //change to follow chassis yaw angle
    //切入跟随底盘角度模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //change to no follow angle
    //切入不跟随云台模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

/**
  * @brief          chassis some measure data updata, such as motor speed, euler angle， robot speed
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
__attribute__((used))void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //update motor speed, accel is differential of speed PID
        //更新电机速度，加速度是速度的PID微分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    //calculate vertical speed, horizontal speed ,rotation speed, left hand rule 
    //更新底盘纵向速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
		chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //calculate chassis euler angle, if chassis add a new gyro sensor,please change this code
		//计算底盘姿态角度, 
//		//底盘上无陀螺仪
//    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
//    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
//    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
		//底盘有陀螺仪
		chassis_move_update->chassis_yaw = rad_format( *(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));
    chassis_move_update->chassis_roll = rad_format( *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET));
}

/**
  * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
  * @param[out]     chassis_move_update: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
fp32 chassis_angle_set = 0.0f;
__attribute__((used))void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    fp32 vx_set = 0.0f, vy_set = 0.0f;
    //get three control set-point, 获取三个控制设置值
    chassis_behaviour_control_set(&vx_set, &vy_set, &chassis_angle_set, chassis_move_control);
		chassis_angle_set = - chassis_angle_set;
    //follow gimbal mode
    //跟随云台模式
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //rotate chassis direction, make sure vertial direction follow gimbal 
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //set control relative angle  set-point
        //设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = rad_format(yaw_motor_relative_angle - PI /3);
        //calculate ratation speed
        //计算旋转PID角速度
        chassis_move_control->wz_set = -PID_Calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
        //speed limit
        //速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
		//底盘角度控制闭环
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        fp32 delat_angle = 0.0f;
        //set chassis yaw angle set-point
        //设置底盘控制的角度
        chassis_move_control->chassis_yaw_set = rad_format(chassis_angle_set);
        delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
        //calculate rotation speed
        //计算旋转的角速度
        //chassis_move_control->wz_set = PID_Calc(&chassis_move_control->chassis_angle_pid, 0.0f, delat_angle);
				chassis_move_control->wz_set = PID_Calc(&chassis_move_control->chassis_angle_pid, delat_angle, 0.0f);
        //speed limit
        //速度限幅
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
		//底盘有旋转速度控制
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //"angle_set" is rotation speed set-point
        //“angle_set” 是旋转速度控制
        chassis_move_control->wz_set = chassis_angle_set;
        //chassis_move_control->wz_set = PID_Calc(&chassis_move_control->chassis_angle_pid, yaw_motor_relative_angle, chassis_angle_set);
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
				
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //in raw mode, set-point is sent to CAN bus
        //在原始模式，设置值是发送到CAN总线
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = chassis_angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
    else if(chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW){
        chassis_move_control->wz_set = -PID_Calc(&chassis_move_control->chassis_angle_pid, chassis_angle_set, yaw_motor_relative_angle);
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }

}

/**
  * @brief          four mecanum wheels speed is calculated by three param. 
  * @param[in]      vx_set: vertial speed
  * @param[in]      vy_set: horizontal speed
  * @param[in]      wz_set: rotation speed
  * @param[out]     wheel_speed: four mecanum wheels speed
  * @retval         none
  */
/**
  * @brief          四个全向轮速度是通过三个参数计算出来的
  * @param[in]      vx_set: 纵向速度
  * @param[in]      vy_set: 横向速度
  * @param[in]      wz_set: 旋转速度
  * @param[out]     wheel_speed: 四个电机速度
  * @retval         none
  */
fp32 gimbal_angle;
fp32 spin_time = 0;
__attribute__((used))void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
	  /********直接计算方法********/
	  fp32 sin_ang, cos_ang;
    fp32 speed_X = vx_set;
    fp32 speed_Y = vy_set;
    //在小陀螺旋转时，角度是一个sin函数，所以减掉yaw电机的角度来保持平衡,
		if(chassis_behaviour_mode == CHASSIS_SPIN){
			//注意，此时是世界坐标系，因为是采用陀螺仪的数据
			gimbal_angle = rad_format(chassis_move.chassis_yaw + PI * 3 / 4); 
			//变速小陀螺
      spin_time += 0.01f;
      if (spin_time){
        SPIN_SPEED = 0.7f + sin(spin_time) / 10.0f;
        if(spin_time > 2 * PI ){
          spin_time = 0.0002f;
        }
      }
		}
		else{
			//这个时候是云台坐标系
			gimbal_angle = rad_format( -yaw_motor_relative_angle + PI * 5 / 6 );	
		}
		
		sin_ang = sin(gimbal_angle);
		cos_ang = cos(gimbal_angle);
		//速度分解到电机
	  wheel_speed[0] = ((-cos_ang - sin_ang) * speed_X + (-sin_ang + cos_ang) * speed_Y +(CHASSIS_WZ_SET_SCALE + 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set) / sqrt(2);
		wheel_speed[1] = ((-cos_ang + sin_ang) * speed_X + (-sin_ang - cos_ang) * speed_Y +(CHASSIS_WZ_SET_SCALE + 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set) / sqrt(2);
		wheel_speed[2] = ((cos_ang + sin_ang) * speed_X + (sin_ang - cos_ang) * speed_Y +(CHASSIS_WZ_SET_SCALE + 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set) / sqrt(2);
		wheel_speed[3] = ((cos_ang - sin_ang) * speed_X + (sin_ang + cos_ang) * speed_Y + (CHASSIS_WZ_SET_SCALE + 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set)/ sqrt(2);

}
/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sentto motor
  * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
  * @retval         none
  */
/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
__attribute__((used))void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;

    //mecanum wheel speed calculation
    //全向轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    if (chassis_move_control_loop->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(wheel_speed[i]);
        }
        //in raw mode, derectly return
        //raw控制直接返回
        return;
    }
		
		//打滑抑制
		//slip_control(chassis_move_control_loop);
    //calculate the max speed in four wheels, limit the max speed
    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];
        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;
        }
    }

    //calculate pid
    //计算pid
    for (i = 0; i < 4; i++)
    {
        PID_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }

    //功率控制
    chassis_power_control(chassis_move_control_loop);


    //赋值电流值
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}

__attribute__((used))void chassis_send_cmd(chassis_move_t *chassis_send_cmd)
{

    //when remote control is offline, chassis motor should receive zero current. 
    //当遥控器掉线的时候，发送给底盘电机零电流.
    if (toe_is_error(DBUS_TOE))
    {
        CAN_cmd_chassis(0, 0, 0, 0);
    }
    else
    {						
        //send control current
        //发送控制电流
        CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                        chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
				//HAL_IWDG_Refresh(&hiwdg);
    }

}

// 全向轮底盘观测：归一化合速度 / 合位移
// 支持 3 轮(120°) 或 4 轮(45°)，用宏切换
#define RADIAN           0.01745329252f
__attribute__((used))
void observer(chassis_move_t *chassis_move, KalmanFilter_t *vxEstimateKF, KalmanFilter_t *vyEstimateKF)
{
    /* 1. 轮子角度表（相对机体 X 前向，单位 deg） */
    static const float angle[4] = {
#if OMNI_WHEEL_NUM == 3
        0.0f, 120.0f, 240.0f
#else
        45.0f, 135.0f, 225.0f, 315.0f
#endif
    };

    float sinA[4], cosA[4];
    for (uint8_t i = 0; i < OMNI_WHEEL_NUM; i++) 
		{
        sinA[i] = arm_sin_f32(angle[i] * RADIAN);
        cosA[i] = arm_cos_f32(angle[i] * RADIAN);
    }

    /* 2. 轮速采集（顺时针为正） */
    float v_wheel[4] = {0};
    for (uint8_t i = 0; i < OMNI_WHEEL_NUM; i++) 
		{
        float r_perp = (i & 1) ? -MOTOR_DISTANCE_TO_CENTER : MOTOR_DISTANCE_TO_CENTER;   // 轮距半值，按实际改
        float w_rel  = -chassis_move->motor_chassis[i].speed
                     + INS.Gyro[0] * r_perp;         // 减去自转牵连
        v_wheel[i] = w_rel * OMNI_RADIUS;
    }

    /* 3. 最小二乘逆解 vx、vy 独立 */
    float ATA[2][2] = {0};
    float ATb[2]    = {0};
    for (uint8_t i = 0; i < OMNI_WHEEL_NUM; i++) 
		{
        float c = cosA[i], s = sinA[i];
        ATA[0][0] += c * c;  ATA[0][1] += c * s;
        ATA[1][0] += s * c;  ATA[1][1] += s * s;
        ATb[0]    += c * v_wheel[i];
        ATb[1]    += s * v_wheel[i];
    }
    float det = ATA[0][0] * ATA[1][1] - ATA[0][1] * ATA[1][0];
    float vx_obs, vy_obs;
    if (fabsf(det) > 1e-6f) {
        vx_obs = (ATA[1][1] * ATb[0] - ATA[0][1] * ATb[1]) / det;
        vy_obs = (ATA[0][0] * ATb[1] - ATA[1][0] * ATb[0]) / det;
    }
		else 
		{
        vx_obs = vy_obs = 0.0f;
    }

    /* 4. 卡尔曼融合（各自轴） */
    xvEstimateKF_Update(vxEstimateKF, INS.MotionAccel_n[0], vx_obs);
    xvEstimateKF_Update(vyEstimateKF, INS.MotionAccel_n[1], vy_obs);

    /* 5. 输出到 chassis 结构体 */
    chassis_move->vx = vx_obs;
    chassis_move->vy = vy_obs;
    float dt = (float)OBSERVE_TIME * 0.001f;
    /* 6. 原地旋转强制归零（可选） */
    if (fabsf(INS.Gyro[0]) > 0.5f &&
        fabsf(vx_obs) < 0.05f && fabsf(vy_obs) < 0.05f) {
        chassis_move->vx = 0.0f;
        chassis_move->vy = 0.0f;
    }
}

#endif
