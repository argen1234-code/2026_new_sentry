#include "Balance_chassis.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"

#if ROBOT_CHASSIS == Balance_wheel
#include "cmsis_os.h"
#include "bsp_usart.h"
#include "arm_math.h"
#include "pid.h"
#include "can_bsp.h"
#include "detect_task.h"
#include "INS_task.h"
#include "chassis_power_control.h"
#include "kalman_filter.h"
#include "observer.h"
#include "slip_control.h"
#include "robot_param.h"
#include "VMC_calc.h"
#include "bsp_vofa+.h"
//电机挂载的can总线
#define JOINT_CAN hfdcan2
#define WHEEL_CAN hfdcan1

//腿长PID
//#define LEG_PID_KP  1200.0f
//#define LEG_PID_KI  0.5f
//#define LEG_PID_KD  5000.0f
//#define LEG_PID_MAX_OUT  150.0f //90牛
//#define LEG_PID_MAX_IOUT 30.0f

#define LEG_PID_KP  500.0f
#define LEG_PID_KI  0.0f
#define LEG_PID_KD  3500.0f
#define LEG_PID_MAX_OUT  120.0f //90牛
#define LEG_PID_MAX_IOUT 0.0f

#define TP_PID_KP 10.0f
#define TP_PID_KI 0.0f 
#define TP_PID_KD 0.1f
#define TP_PID_MAX_OUT  2.0f
#define TP_PID_MAX_IOUT 0.0f

//转向PID，最大力矩输出3NM
#define TURN_PID_KP 1.2f
#define TURN_PID_KI 0.0f 
#define TURN_PID_KD 0.2f
#define TURN_PID_MAX_OUT  2.0f
#define TURN_PID_MAX_IOUT 0.0f

//ROLL支撑
#define ROLL_PID_KP 100.0f
#define ROLL_PID_KI 0.0f 
#define ROLL_PID_KD 0.0f
#define ROLL_PID_MAX_OUT  3.5f
#define ROLL_PID_MAX_IOUT 0.0f

#define Max_T 20.0f
#define Min_T -Max_T

#define MAX_WHEEL_T 10.0f
#define MIN_WHEEL_T -MAX_WHEEL_T 

#define MAX_LEG 0.4f
#define MIN_LEG 0.15f

//1/4的整车重量 MG, 单位N
//#define Mg (33 / 4 * 9.80f)
#define M 33/2.0f
#define G 9.80f
#define Mg M * G
//离地检测的阈值
#define ground_detection Mg/8.0f

//板凳模型下关节电机的位置
#define JOINT1_POS 1.0418f
#define JOINT2_POS -0.9762f
#define JOINT3_POS 1.0529f
#define JOINT4_POS -1.0717f

//自己算一下电机最大角速度
//电机打滑速度，最大51.4rad/s，可以自行修改
#define Max_speed 38.0f
#define Min_speed -Max_speed

//驱动轮半径
#define WHEEL_RAD 0.055f

//超时时限，单位MS
#define TIMEOUT 500

#define JOINT_TORQUE_MORE_OFFSET             ((uint8_t)1 << 0)  // 关节电机输出力矩过大
#define DBUS_ERROR_OFFSET   				 ((uint8_t)1 << 1)  // dbus错误
#define WHEEL_TORQUE_OFFSET 				 ((uint8_t)1 << 2) // 电机扭矩过大
#define WHEEL_SPEED_OFFSET 				 	 ((uint8_t)1 << 3) // 轮电机速度过大
#define MOTOR_OFFLINE      					 ((uint8_t)1 << 4)  // 关节电机掉线
#define WHEEL_OFFLINE    					 ((uint8_t)1 << 5)  // 轮电机掉线
#define RAPAM_ERROR      					 ((uint8_t)1 << 6)  // 设置参数错误

//板凳模型开关
#define DEBUG 0



//lqr矩阵,在matlab里面仿真，串联腿的L1 = 0
float Poly_Coefficient[12][4]={	
 {-162.809066,180.941813,-83.579986,-0.477955},
 {-1.146484,2.548645,-6.526898,0.187617},
 {-36.291851,36.631927,-12.900660,-0.206949},
 {-38.826772,39.610562,-14.764869,-0.370617},
 {-45.251421,63.234619,-34.056056,8.517446},
 {-5.264800,8.239754,-4.951971,1.465911},
 {123.637761,-88.819306,6.621940,9.927918},
 {20.720743,-20.110973,6.240901,0.266868},
 {-28.851503,39.652414,-20.896494,4.961603},
 {-39.870024,51.325770,-25.602378,5.934290},
 {301.300534,-304.152958,107.397736,-0.030207},
 {49.041678,-50.288653,18.238960,-0.508942}
								};	

													
float POS[4] = {
	//前          后
	JOINT1_POS, JOINT2_POS,  //右
	JOINT3_POS, JOINT4_POS  //左
};
																
//放置lqr的参数，初始化无所谓，主要是看上面的矩阵																
float LQR_K_R[12]=
									{ 
									-16.6271,	-1.5394,	-0.7001,	-1.4681,	4.7675,	0.4977,
									19.6639,	0.7449,	0.4414,	0.8771,	29.8843,	1.2325
									};


float LQR_K_L[12]=
									{ 
									-14.7333,	-1.1901,	-0.6917,	-1.4383,	5.8373,	0.5445,
									20.0303,	0.8189,	0.65443,	1.3019,	27.8958,	1.0854
									};
//每个参数得到的值，暂存在这里方便调试
float LQR_WHEEL_R[6] = {0};
//每个参数得到的值，暂存在这里方便调试
float LQR_WHEEL_L[6] = {0};

//每个参数得到的值，暂存在这里方便调试
float LQR_JOINT_R[6] = {0};
//每个参数得到的值，暂存在这里方便调试
float LQR_JOINT_L[6] = {0};


//函数声明
__attribute__((used))void chassis_feedback_update(chassis_move_t *chassis_move_update);
void jump_loop_r(chassis_move_t *chassis,vmc_leg_t *vmcr,pid_type_def *leg);
void jump_loop_l(chassis_move_t *chassis,vmc_leg_t *vmcl,pid_type_def *leg);
void chassis_motor_init(void);
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
__attribute__((used))void chassis_init(chassis_move_t *chassis_move_init){
	//等待上电稳定
	osDelay(5000);
	//初始化腿长PID
	const static float legr_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
	const static float legl_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
	const static float roll[3] = {ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD};
	chassis_motor_init();
//	chassis_motor_zero();
	
	xvEstimateKF_Init(&vxEstimateKF);
	xvEstimateKF_Init(&vyEstimateKF);
	xvEstimateKF_Init(&vaEstimateKF);
	
	//获取IMU的六轴数据
	chassis_move_init->acc_point = get_gyro_data_point();
	chassis_move_init->chassis_INS_angle = get_INS_angle_point();
	for(uint8_t i = 0; i < 4; i++)
	{
		chassis_move_init->joint_motor[i] = get_mit_motor_measure_point(i);
		#if DEBUG == 1
		chassis_move_init->joint_motor[i]->set.KP = 10.0f;
		chassis_move_init->joint_motor[i]->set.KD = 0.7f;
		chassis_move_init->joint_motor[i]->set.POS = POS[i];
		chassis_move_init->joint_motor[i]->set.VEL = 0.0f;
		chassis_move_init->joint_motor[i]->set.TOR = 0;
		#else 
		chassis_move_init->joint_motor[i]->set.KP = 0.0f;
		chassis_move_init->joint_motor[i]->set.KD = 0.0f;
		chassis_move_init->joint_motor[i]->set.POS = 0.0f;
		chassis_move_init->joint_motor[i]->set.VEL = 0.0f;
		chassis_move_init->joint_motor[i]->set.TOR = 0;
		#endif

		chassis_move_init->joint_motor[i]->param.P_MAX = P_MAX_03;
		chassis_move_init->joint_motor[i]->param.P_MIN = P_MIN_03;
		chassis_move_init->joint_motor[i]->param.V_MIN  = V_MIN_03;
		chassis_move_init->joint_motor[i]->param.V_MAX  = V_MAX_03;
		chassis_move_init->joint_motor[i]->param.KP_MIN = KP_MIN_03;
		chassis_move_init->joint_motor[i]->param.KP_MAX = KP_MAX_03;
		chassis_move_init->joint_motor[i]->param.KD_MIN = KD_MIN_03;
		chassis_move_init->joint_motor[i]->param.KD_MAX = KD_MAX_03;
		chassis_move_init->joint_motor[i]->param.T_MIN  = TP_MIN_03;
		chassis_move_init->joint_motor[i]->param.T_MAX  = TP_MAX_03;
	
}
	for(uint8_t i = 0; i < 2; i++)
	{
		chassis_move_init->wheel_motor[i] = get_mit_motor_measure_point(i + 4);
		chassis_move_init->wheel_motor[i]->set.KP = 0;
		chassis_move_init->wheel_motor[i]->set.KD = 0;
		chassis_move_init->wheel_motor[i]->set.POS = 0;
		chassis_move_init->wheel_motor[i]->set.VEL = 0;
		chassis_move_init->wheel_motor[i]->set.TOR = 0;
		chassis_move_init->wheel_motor[i]->param.P_MAX  = P_MIN_3519   ;
		chassis_move_init->wheel_motor[i]->param.P_MIN  = P_MAX_3519   ;
		chassis_move_init->wheel_motor[i]->param.V_MIN  = V_MIN_3519   ;
		chassis_move_init->wheel_motor[i]->param.V_MAX  = V_MAX_3519   ;
		chassis_move_init->wheel_motor[i]->param.KP_MIN = KP_MIN_3519  ;
		chassis_move_init->wheel_motor[i]->param.KP_MAX = KP_MAX_3519  ;
		chassis_move_init->wheel_motor[i]->param.KD_MIN = KD_MIN_3519  ;
		chassis_move_init->wheel_motor[i]->param.KD_MAX = KD_MAX_3519  ;
		chassis_move_init->wheel_motor[i]->param.T_MIN  = TP_MAX_3519  ;
		chassis_move_init->wheel_motor[i]->param.T_MAX  = TP_MIN_3519  ;
		
		
	}
	    //获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
	
	//in beginning， chassis mode is raw 
    //底盘开机状态为原始
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //get remote control point
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
	//VMC初始化,
	VMC_init(&chassis_move_init->left);//给杆长赋值
	VMC_init(&chassis_move_init->right);//给杆长赋值
	
	//初始化腿长PID
	PID_init(&chassis_move_init->LegR_Pid, PID_POSITION,legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid
	PID_init(&chassis_move_init->LegL_Pid, PID_POSITION,legl_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid

	//防劈叉PID
	PID_init(&chassis_move_init->Tp_Pid, PID_POSITION, tp_pid, TP_PID_MAX_OUT,TP_PID_MAX_IOUT);
	//转向PID
	PID_init(&chassis_move_init->Turn_Pid, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);
	//ROLL轴防止高低肩
	PID_init(&chassis_move_init->RollR_Pid, PID_POSITION, roll, ROLL_PID_MAX_OUT,ROLL_PID_MAX_IOUT);
	chassis_move_init -> wheel_motor_timeout = 0;
	chassis_move_init -> joint_motor_flag = 0;
	chassis_move_init -> leg_set = chassis_move_init -> last_leg_set = MIN_LEG;
	chassis_move_init -> v_set = 0.0f;
	chassis_move_init -> x_set = 0.0f;
	chassis_move_init -> roll_set = 0.0f;
	chassis_move_init -> jump_flag = 0;
    //max and min speed
    //最大 最小速度
    chassis_move_init-> vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init-> vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init-> vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init-> vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_feedback_update(chassis_move_init);
	osDelay(100);
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
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && 
		chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
		//应该设置为与云台yaw轴电机相关
		//chassis_move_transit->chassis_yaw_set = yaw_motor_angle;
    }
    //change to follow chassis yaw angle
    //切入跟随底盘角度模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && 
			chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //change to no follow angle
    //切入不跟随云台模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) &&
				 chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
			  //应该设置为与云台yaw轴电机相关
			  //chassis_move_transit->chassis_yaw_set = yaw_motor_angle;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}


__attribute__((used))void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    fp32 vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;
    //get three control set-point, 获取三个控制设置值
		//chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_move_control);
		chassis_behaviour_control_set(&vx_set, &vy_set, &wz_set, chassis_move_control);
		chassis_move_control->chassis_yaw_set = wz_set + chassis_move_control->chassis_yaw;
			
			
		//右侧摇杆水平方向控制底盘旋转
		//chassis_move_control->chassis_yaw_set +=  vy_set * 0.0004f;	
		//修改腿长的值
		//chassis_move_control->leg_set += vy_set * 0.0008f;
		chassis_move_control->leg_set = 0.3f;
		chassis_move_control->leg_set = fp32_constrain(chassis_move_control->leg_set, MIN_LEG, MAX_LEG);
		
		chassis_move_control->leg_left_set = chassis_move_control->leg_set;	
		chassis_move_control->leg_right_set = chassis_move_control->leg_set;	
		if(fabsf(chassis_move_control->last_leg_left_set - chassis_move_control->leg_left_set) > 0.002f || fabsf(chassis_move_control->last_leg_right_set - chassis_move_control->leg_right_set) > 0.002f)
		{
				//遥控器控制腿长在变化
				chassis_move_control->right.leg_flag=1;	//为1标志着腿长在主动伸缩(不包括自适应伸缩)，根据这个标志可以不进行离地检测，因为当腿长在主动伸缩时，离地检测会误判端为离地了
				chassis_move_control->left.leg_flag=1;	 			
		}
		
		//保存腿长
		chassis_move_control->last_leg_set = chassis_move_control->leg_set;	
		chassis_move_control->last_leg_left_set = chassis_move_control->leg_left_set;
		chassis_move_control->last_leg_right_set = chassis_move_control->leg_right_set;
		
    //follow gimbal mode
    //跟随云台模式
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //rotate chassis direction, make sure vertial direction follow gimbal 
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        // sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
        // cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
		sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw);
        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw);
        chassis_move_control->v_set = cos_yaw * vx_set + sin_yaw * vy_set;
        //set control relative angle  set-point
        //设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = rad_format(yaw_motor_relative_angle - PI /3);
        //calculate ratation speed
        //计算旋转PID角速度
        chassis_move_control->wz_set = wz_set;
        //speed limit
        //速度限幅
        chassis_move_control->v_set = fp32_constrain(chassis_move_control->v_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    }
		//底盘角度控制闭环
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        fp32 delat_angle = 0.0f;
        //set chassis yaw angle set-point
        //设置底盘控制的角度
        chassis_move_control->chassis_yaw_set = rad_format(chassis_move_control->chassis_angle_set);
        delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
        //calculate rotation speed
        //计算旋转的角速度
        //chassis_move_control->wz_set = PID_Calc(&chassis_move_control->chassis_angle_pid, 0.0f, delat_angle);
		chassis_move_control->wz_set = wz_set;
        //speed limit
        //速度限幅
        chassis_move_control->v_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    }
		//底盘有旋转速度控制
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        //"angle_set" is rotation speed set-point
        //“angle_set” 是旋转速度控制
        chassis_move_control->wz_set = chassis_move_control->chassis_angle_set;
        //chassis_move_control->wz_set = PID_Calc(&chassis_move_control->chassis_angle_pid, yaw_motor_relative_angle, chassis_angle_set);
        chassis_move_control->v_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
				
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        //in raw mode, set-point is sent to CAN bus
        //在原始模式，设置值是发送到CAN总线
        chassis_move_control->v_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);;
        chassis_move_control->wz_set = chassis_move_control->chassis_angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
    }
    else if(chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW){
        chassis_move_control->wz_set = - PID_Calc(&chassis_move_control->Turn_Pid, chassis_move_control->chassis_angle_set, yaw_motor_relative_angle);
        chassis_move_control->v_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    }

}


__attribute__((used))void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
		chassis_move_update->x_set += chassis_move_update->v_set * CHASSIS_CONTROL_TIME_MS /1000.0f; 
//		chassis_move_update->x_set += chassis_move_update->v_set * (2 /1000.0f); 

		chassis_move_update->right.phi1=PI/2.0f + (chassis_move_update->joint_motor[0]->fdb.pos * MOTOR_REDUCTION);
		chassis_move_update->right.phi4=PI/2.0f + (chassis_move_update->joint_motor[1]->fdb.pos * MOTOR_REDUCTION);
		
		chassis_move_update->left.phi1=PI/2.0f + (chassis_move_update->joint_motor[2]->fdb.pos * MOTOR_REDUCTION);
		chassis_move_update->left.phi4=PI/2.0f + (chassis_move_update->joint_motor[3]->fdb.pos * MOTOR_REDUCTION);
		chassis_move_update->chassis_yaw   = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));
    chassis_move_update->chassis_roll  = rad_format(*(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET));
	
		observer(&chassis_move,&vxEstimateKF, &vyEstimateKF);
	
		//更新角度和角速度
		chassis_move_update->myPithR = chassis_move_update->chassis_pitch;
		chassis_move_update->myPithGyroR = *(chassis_move_update->acc_point + INS_GYRO_X_ADDRESS_OFFSET);
		
		chassis_move_update->myPithL= 0.0f - chassis_move_update->chassis_pitch;
		chassis_move_update->myPithGyroL=0.0f - *(chassis_move_update->acc_point + INS_GYRO_X_ADDRESS_OFFSET);
		 
		chassis_move_update->total_yaw = get_YawTotalAngle();
//		chassis_move_update->chassis_yaw = rad_format(chassis_move_update->total_yaw); 

		//	chassis->total_yaw=ins->YawTotalAngle;
		chassis_move_update->roll = chassis_move_update->chassis_roll ;
		chassis_move_update->theta_err = 0.0f - (chassis_move_update->right.theta+chassis_move_update->left.theta);
		chassis_move_update->last_leg_set = chassis_move_update->leg_set;
		chassis_move_update->last_leg_left_set = chassis_move_update->leg_left_set;
		chassis_move_update->last_leg_right_set = chassis_move_update->leg_right_set;
		//点击B键启动跳跃
		if(chassis_move_update->chassis_RC->key.v & KEY_JUMP)
		{
			//直接开始准备起跳
			chassis_move_update -> jump_flag = 1;
		}
		if(chassis_move_update->chassis_pitch < (3.1415926f / 6.0f) && chassis_move_update->chassis_pitch > (-3.1415926f / 6.0f))
		{//根据pitch角度判断倒地自起是否完成
			chassis_move_update->recover_flag = 0;
		}

		if(chassis_move_update->recover_flag==0
			&&((chassis_move_update->chassis_pitch < ((-3.1415926f)/4.0f) && chassis_move_update->chassis_pitch > ((-3.1415926f)/2.0f))
		  ||(chassis_move_update->chassis_pitch > (3.1415926f/4.0f) && chassis_move_update->chassis_pitch < (3.1415926f/2.0f))))
		{
		  chassis_move_update->recover_flag=1;//需要自起
		}

		if((HAL_GetTick() - chassis_move_update -> wheel_motor[0]->fdb.last_fdb_time) >  TIMEOUT ||\
			(HAL_GetTick() - chassis_move_update -> wheel_motor[1]->fdb.last_fdb_time) > TIMEOUT  )
		{
			chassis_move_update->wheel_motor_timeout = 1;
			chassis_move_update->error_code |= WHEEL_OFFLINE; 
		}
		else
		{
			chassis_move_update->wheel_motor_timeout = 0;
		}

		for(uint8_t i = 0; i  < 4; i++)
		{
			if((HAL_GetTick() - chassis_move_update -> joint_motor[i]->fdb.last_fdb_time) > TIMEOUT )
			{
				chassis_move_update->error_code |= MOTOR_OFFLINE; 
				chassis_move_update->joint_motor_flag = 1;
			}
			if(chassis_move_update -> joint_motor[i]->fdb.tor > Max_T || chassis_move_update -> joint_motor[i]->fdb.tor < Min_T)
			{
				chassis_move_update->error_code |= JOINT_TORQUE_MORE_OFFSET;
			}
		}
		if(HAL_GetTick() - chassis_move_update->chassis_RC->last_fdb >TIMEOUT )
		{
			chassis_move_update->error_code |= DBUS_ERROR_OFFSET;
		}
				//超出一定转速我们认为轮毂打滑
		if((chassis_move_update->wheel_motor[0]->fdb.vel > Max_speed || chassis_move_update->wheel_motor[0]->fdb.vel < Min_speed) ||\
			 (chassis_move_update->wheel_motor[1]->fdb.vel > Max_speed || chassis_move_update->wheel_motor[1]->fdb.vel < Min_speed))
		{
			chassis_move_update->wheel_motor[0]->set.TOR  = 0;
			chassis_move_update->wheel_motor[1]->set.TOR  = 0;
			chassis_move_update->error_code |= WHEEL_SPEED_OFFSET; 
		}
		
}
__attribute__((used))void chassis_control_loop(chassis_move_t *chassis_move_control_loop){
	
		VMC_calc_1_right(&chassis_move_control_loop->right,&INS,((float)CHASSIS_CONTROL_TIME_MS) /1000.0f);//计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是0.002秒
		VMC_calc_1_left(&chassis_move_control_loop->left,&INS,((float)CHASSIS_CONTROL_TIME_MS) /1000.0f);//计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是0.002秒
		
		for(int i=0;i<12;i++)
		{
			LQR_K_R[i]=LQR_K_calc(&Poly_Coefficient[i][0],chassis_move_control_loop->right.L0 );	
			LQR_K_L[i]=LQR_K_calc(&Poly_Coefficient[i][0],chassis_move_control_loop->left.L0 );	
		}

		//转向力矩
		//chassis_move_control_loop->turn_T=chassis_move_control_loop->Turn_Pid.Kp * (chassis_move_control_loop->turn_set - chassis_move_control_loop->total_yaw) - chassis_move_control_loop->Turn_Pid.Kd*INS.Gyro[2];//这样计算更稳一点
//		chassis_move_control_loop->turn_T = chassis_move_control_loop->Turn_Pid.Kp * (chassis_move_control_loop->chassis_yaw_set - chassis_move_control_loop->chassis_yaw) 
//											- chassis_move_control_loop->Turn_Pid.Kd*INS.Gyro[2];//这样计算更稳一点
		chassis_move_control_loop->turn_T = 0.0f;

		chassis_move_control_loop->leg_tp = PID_Calc(&chassis_move_control_loop->Tp_Pid, chassis_move_control_loop->theta_err,0.0f);//防劈叉pid计算
		
		//theta前倾为-，后倾为+  picth前倾为+，后倾为-
		//wheel为负，右边电机逆时针转动，机体向前
		LQR_WHEEL_R[0] = LQR_K_R[0]*(chassis_move_control_loop->right.theta - 0.0f);
		LQR_WHEEL_R[1] = LQR_K_R[1]*(chassis_move_control_loop->right.d_theta - 0.0f);
//		LQR_WHEEL_R[2] = LQR_K_R[2]*(chassis_move_control_loop->x_filter - chassis_move_control_loop->x_set);
//		LQR_WHEEL_R[3] = LQR_K_R[3]*( chassis_move_control_loop->v_filter - chassis_move_control_loop->v_set);
		LQR_WHEEL_R[2] = LQR_K_R[2]*(chassis_move_control_loop->x_filter - 0.0f);
		LQR_WHEEL_R[3] = LQR_K_R[3]*( chassis_move_control_loop->v_filter - 0.0f);
		LQR_WHEEL_R[4] = LQR_K_R[4]*(chassis_move_control_loop->myPithR - 0.00f);
		LQR_WHEEL_R[5] = LQR_K_R[5]*(chassis_move_control_loop->myPithGyroR - 0.0f);
		chassis_move_control_loop->wheel_motor[1]->set.TOR = (
																		+LQR_WHEEL_R[0]
																		+LQR_WHEEL_R[1]
																		+LQR_WHEEL_R[2]
																		+LQR_WHEEL_R[3]
																		+LQR_WHEEL_R[4]
																		+LQR_WHEEL_R[5]
		);
		
		LQR_JOINT_R[0] = LQR_K_R[6]*(chassis_move_control_loop->right.theta - 0.00f);
		LQR_JOINT_R[1] = LQR_K_R[7]*(chassis_move_control_loop->right.d_theta - 0.0f);
//		LQR_JOINT_R[2] = LQR_K_R[8]*(chassis_move_control_loop->x_filter - chassis_move_control_loop->x_set);
//		LQR_JOINT_R[3] = LQR_K_R[9]*(chassis_move_control_loop->v_filter - chassis_move_control_loop->v_set);
		LQR_JOINT_R[2] = LQR_K_R[8]*(chassis_move_control_loop->x_filter - 0.0f);
		LQR_JOINT_R[3] = LQR_K_R[9]*(chassis_move_control_loop->v_filter - 0.0f);
		LQR_JOINT_R[4] = LQR_K_R[10]*(chassis_move_control_loop->myPithR - 0.00f);
		LQR_JOINT_R[5] = LQR_K_R[11]*(chassis_move_control_loop->myPithGyroR - 0.0f);

		//右边髋关节输出力矩				
		chassis_move_control_loop->right.Tp =(
																						LQR_JOINT_R[0]
																					+ LQR_JOINT_R[1]
																					+ LQR_JOINT_R[2]
																					+ LQR_JOINT_R[3]
																					+ LQR_JOINT_R[4]
																					+ LQR_JOINT_R[5]
																					);
						
						
						
			//theta前倾为正，后倾为-  picth前倾为-，后倾为+
			//wheel为正，左边电机顺时针转动，机体向前
			LQR_WHEEL_L[0] = LQR_K_L[0]*(chassis_move_control_loop->left.theta-0.0f);
			LQR_WHEEL_L[1] = LQR_K_L[1]*(chassis_move_control_loop->left.d_theta-0.0f);
//			LQR_WHEEL_L[2] = LQR_K_L[2]*(chassis_move_control_loop->x_set - chassis_move_control_loop->x_filter);
//			LQR_WHEEL_L[3] = LQR_K_L[3]*(chassis_move_control_loop->v_set - chassis_move_control_loop->v_filter);
			LQR_WHEEL_L[2] = LQR_K_L[2]*(0.0f - chassis_move_control_loop->x_filter);
			LQR_WHEEL_L[3] = LQR_K_L[3]*(0.0f - chassis_move_control_loop->v_filter);
			//	LQR_WHEEL_L[4] = -LQR_K[4]*(chassis->myPithL-0.0f);
			LQR_WHEEL_L[4] = LQR_K_L[4]*(chassis_move_control_loop->myPithL-0.00f) ;
			LQR_WHEEL_L[5] = LQR_K_L[5]*(chassis_move_control_loop->myPithGyroL-0.0f);
			chassis_move_control_loop->wheel_motor[0]->set.TOR = (
																				+LQR_WHEEL_L[0]
																				+LQR_WHEEL_L[1]
																				+LQR_WHEEL_L[2]
																				+LQR_WHEEL_L[3]
																				+LQR_WHEEL_L[4]
																				+LQR_WHEEL_L[5]
			);
			
			LQR_JOINT_L[0] =  LQR_K_L[6]*(chassis_move_control_loop->left.theta-0.0f);
			LQR_JOINT_L[1] = 	LQR_K_L[7]*(chassis_move_control_loop->left.d_theta-0.0f);
//			LQR_JOINT_L[2] = 	LQR_K_L[8]*(chassis_move_control_loop->x_set - chassis_move_control_loop->x_filter);
//			LQR_JOINT_L[3] = 	LQR_K_L[9]*(chassis_move_control_loop->v_set - chassis_move_control_loop->v_filter);
			LQR_JOINT_L[2] = 	LQR_K_L[8]*(0.0f - chassis_move_control_loop->x_filter);
			LQR_JOINT_L[3] = 	LQR_K_L[9]*(0.0f - chassis_move_control_loop->v_filter);
			LQR_JOINT_L[4] = 	LQR_K_L[10]*(chassis_move_control_loop->myPithL - 0.00f);
			LQR_JOINT_L[5] = 	LQR_K_L[11]*(chassis_move_control_loop->myPithGyroL-0.0f);
			//左髋关节输出力矩		
			chassis_move_control_loop->left.Tp=(
							  LQR_JOINT_L[0]
							+ LQR_JOINT_L[1]
							+ LQR_JOINT_L[2]
							+ LQR_JOINT_L[3]
							+ LQR_JOINT_L[4]
							+ LQR_JOINT_L[5]
																											);
																											
//		chassis_move_control_loop->wheel_motor[0]->set.TOR  = chassis_move_control_loop->wheel_motor[0]->set.TOR  + chassis_move_control_loop->turn_T;	//轮毂电机输出力矩
//		chassis_move_control_loop->wheel_motor[1]->set.TOR  = chassis_move_control_loop->wheel_motor[1]->set.TOR  + chassis_move_control_loop->turn_T;	//轮毂电机输出力矩
																						 
		//轮毂电机限幅                            
		chassis_move_control_loop->wheel_motor[0]->set.TOR  = fp32_constrain(chassis_move_control_loop->wheel_motor[0]->set.TOR , MIN_WHEEL_T, MAX_WHEEL_T);	
		chassis_move_control_loop->wheel_motor[1]->set.TOR  = fp32_constrain(chassis_move_control_loop->wheel_motor[1]->set.TOR , MIN_WHEEL_T, MAX_WHEEL_T);	
		

		//防止劈叉,用左右腿的位置的差值拿来计算
		chassis_move_control_loop->right.Tp = chassis_move_control_loop->right.Tp - chassis_move_control_loop->leg_tp;//髋关节输出力矩
		chassis_move_control_loop->left.Tp = chassis_move_control_loop->left.Tp + chassis_move_control_loop->leg_tp;//髋关节输出力矩

		//防止高低肩，目前还不能手动控制
//		chassis_move_control_loop->now_roll_set = PID_Calc(&chassis_move_control_loop->RollR_Pid, chassis_move_control_loop->roll, chassis_move_control_loop->roll_set);
		chassis_move_control_loop->now_roll_set = -PID_Calc(&chassis_move_control_loop->RollR_Pid, chassis_move_control_loop->roll, 0);
//		chassis_move_control_loop->now_roll_set = 0;

		jump_loop_r(chassis_move_control_loop, &chassis_move_control_loop->right, &chassis_move_control_loop->LegR_Pid);
		jump_loop_l(chassis_move_control_loop, &chassis_move_control_loop->left, &chassis_move_control_loop->LegL_Pid);
			
		chassis_move_control_loop->right_flag = ground_detectionR(&chassis_move_control_loop->right, &INS, ground_detection);//右腿离地检测
		chassis_move_control_loop->left_flag = ground_detectionL(&chassis_move_control_loop->left, &INS, ground_detection);//左腿离地检测

		 if(chassis_move_control_loop->recover_flag==0)		
		 {
			 //倒地自起不需要检测是否离地	 
			if( (chassis_move_control_loop->right_flag==1 || chassis_move_control_loop->left_flag==1) &&
				chassis_move_control_loop->right.leg_flag == 0 && chassis_move_control_loop->right.leg_flag == 0)
			{
				//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
				//轮电机的力矩修改为0
				chassis_move_control_loop->wheel_motor[0]->set.TOR  = 0.0f;
				chassis_move_control_loop->wheel_motor[1]->set.TOR  = 0.0f;
				//只保证杆垂直向下
				chassis_move_control_loop->right.Tp = LQR_K_R[6]*(chassis_move_control_loop->right.theta - 0.0f) + LQR_K_R[7] * (chassis_move_control_loop->right.d_theta - 0.0f);
				chassis_move_control_loop->left.Tp = LQR_K_L[6]*(chassis_move_control_loop->left.theta - 0.0f) + LQR_K_L[7] * (chassis_move_control_loop->left.d_theta - 0.0f);

				//离地清除位置信息
				chassis_move_control_loop->x_filter = 0.0f;
				chassis_move_control_loop->x_set = chassis_move_control_loop->x_filter;
				//防止转向
				chassis_move_control_loop->chassis_yaw_set = chassis_move_control_loop->chassis_yaw;
				// chassis_move_control_loop->turn_set = chassis_move_control_loop->total_yaw;
				//防止劈叉
				chassis_move_control_loop->right.Tp = chassis_move_control_loop->right.Tp + chassis_move_control_loop->leg_tp;		
				chassis_move_control_loop->left.Tp = chassis_move_control_loop->left.Tp + chassis_move_control_loop->leg_tp;		
				
			}
			else
			{//没有离地
				chassis_move_control_loop->right.leg_flag = 0;//置为0
				chassis_move_control_loop->left.leg_flag = 0;//置为0
			}
		 }
		 //倒地自起中
		 else if(chassis_move_control_loop->recover_flag == 1)
		 {
			 chassis_move_control_loop->right.Tp = 0.0f;
			 chassis_move_control_loop->left.Tp = 0.0f;
		 }	 
		 
		chassis_move_control_loop->right.F0 = fp32_constrain(chassis_move_control_loop->right.F0, -Mg, Mg);//限幅
		chassis_move_control_loop->left.F0 = fp32_constrain(chassis_move_control_loop->left.F0, -Mg, Mg);//限幅
		 
		VMC_calc_2(&chassis_move_control_loop->right);//计算期望的关节输出力矩
		VMC_calc_2(&chassis_move_control_loop->left);//计算期望的关节输出力矩
		 
		//额定扭矩
		chassis_move_control_loop->right.torque_set[1] = fp32_constrain(chassis_move_control_loop->right.torque_set[1], Min_T, Max_T);	
		chassis_move_control_loop->right.torque_set[0] = fp32_constrain(chassis_move_control_loop->right.torque_set[0], Min_T, Max_T);
		chassis_move_control_loop->left.torque_set[1] = fp32_constrain(chassis_move_control_loop->left.torque_set[1], Min_T, Max_T);	
		chassis_move_control_loop->left.torque_set[0] = fp32_constrain(chassis_move_control_loop->left.torque_set[0], Min_T, Max_T);

}
fp32 lf = 0.0f;
fp32 lb = 0.0f;
fp32 rf = 0.0f;
fp32 rb = 0.0f;
fp32 theta_l_debug = 0.0f;
fp32 theta_r_debug = 0.0f;
__attribute__((used))void chassis_send_cmd(chassis_move_t *chassis_send_cmd)
{
	
	
/*
	
																前
左                               															  右
CAN_id:8	rev:DM_M3_ID	     	开发板io				rev:DM_M1_ID  CAN_id:6
WHEEL1:rev:DM_M6_ID  CAN_id:10							WHEEL0:rev:DM_M5_ID  CAN_id:11										
	
CAN_id:9 	rev:DM_M4_ID			   开发板can				rev:DM_M2_ID  CAN_id:7

																后


*/
	
		chassis_send_cmd->wheel_motor[1]->set.TOR = (chassis_send_cmd->wheel_motor[1]->set.TOR >= 0 ? chassis_send_cmd->wheel_motor[1]->set.TOR + WHEEL_MOTOR1_FRE : chassis_send_cmd->wheel_motor[1]->set.TOR - WHEEL_MOTOR1_FRE);
	  chassis_send_cmd->wheel_motor[0]->set.TOR = (chassis_send_cmd->wheel_motor[0]->set.TOR >= 0 ? chassis_send_cmd->wheel_motor[0]->set.TOR + WHEEL_MOTOR2_FRE : chassis_send_cmd->wheel_motor[0]->set.TOR - WHEEL_MOTOR2_FRE);
	
//		if(chassis_send_cmd->error_code || chassis_behaviour_mode == CHASSIS_NO_MOVE)
//		if(chassis_behaviour_mode == CHASSIS_NO_MOVE)
		if(chassis_send_cmd->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
		{	
			chassis_send_cmd->joint_motor[0]->set.TOR = 0.0f;
			chassis_send_cmd->joint_motor[1]->set.TOR = 0.0f;
																					 
			chassis_send_cmd->joint_motor[2]->set.TOR = 0.0f;
			chassis_send_cmd->joint_motor[3]->set.TOR = 0.0f;
			#if DEBUG == 1
			
			chassis_send_cmd->wheel_motor[1]->set.TOR = 0.0f;
			chassis_send_cmd->wheel_motor[0]->set.TOR = 0.0f;
			
			#else
			
			chassis_send_cmd->wheel_motor[1]->set.TOR = 0.0f;
			chassis_send_cmd->wheel_motor[0]->set.TOR = 0.0f;
			#endif
			
		}
		else
		{
			//数据赋值
//			chassis_send_cmd->joint_motor[0]->set.TOR = 0.0f;
//			chassis_send_cmd->joint_motor[1]->set.TOR = 0.0f;
//			chassis_send_cmd->joint_motor[2]->set.TOR = 0.0f;
//			chassis_send_cmd->joint_motor[3]->set.TOR = 0.0f;		

			
//				chassis_send_cmd->wheel_motor[1]->set.TOR = 0.0f;
//				chassis_send_cmd->wheel_motor[0]->set.TOR = 0.0f;

				chassis_send_cmd->joint_motor[0]->set.TOR = chassis_send_cmd->right.torque_set[0];
				chassis_send_cmd->joint_motor[1]->set.TOR = chassis_send_cmd->right.torque_set[1];	
				chassis_send_cmd->joint_motor[2]->set.TOR = chassis_send_cmd->left.torque_set[0];
				chassis_send_cmd->joint_motor[3]->set.TOR = chassis_send_cmd->left.torque_set[1];
		
				rf = chassis_send_cmd->right.torque_set[0];
				rb = chassis_send_cmd->right.torque_set[1];
				lf = chassis_send_cmd->left.torque_set[0];
				lb = chassis_send_cmd->left.torque_set[1];
			theta_l_debug = chassis_send_cmd->right.theta;
			theta_r_debug = chassis_send_cmd->left.theta;

		}
	
		    #if ROBOT_MODE  == debug
				//打印调试信息
				fp32 t4[15] = {lf,lb,rf,rb, 0.0f, 0.0f, chassis_send_cmd->chassis_pitch*57.29f, chassis_send_cmd->left.Tp, chassis_send_cmd->right.Tp, 0.0f, 0.0f, 0.0f, 0.0f};
				t4[7] = chassis_send_cmd->wheel_motor[0]->set.TOR;
				t4[8] = chassis_send_cmd->wheel_motor[1]->set.TOR;
//					t4[9] =  LQR_JOINT_L[0];
//        	t4[10] = LQR_JOINT_R[0];
//        	t4[11] = LQR_JOINT_L[4];
//        	t4[12] = LQR_JOINT_R[4];
					
				
//				t4[9] = chassis_send_cmd->joint_motor[0]->fdb.pos;
//				t4[10] = chassis_send_cmd->joint_motor[1]->fdb.pos;
//				t4[11] = chassis_send_cmd->joint_motor[2]->fdb.pos;
//				t4[12] =  chassis_send_cmd->joint_motor[3]->fdb.pos;
//				t4[13] =  chassis_send_cmd->v_filter;
//				t4[14] =  chassis_send_cmd->x_filter;	
					t4[0] = LQR_WHEEL_R[0];
					t4[1] = LQR_WHEEL_R[1];
					t4[2] = LQR_WHEEL_R[2];
					t4[3] = LQR_WHEEL_R[3];
					t4[4] = LQR_WHEEL_R[4];
					t4[5] = LQR_WHEEL_R[5];
					t4[9] = LQR_WHEEL_L[0];
					t4[10] = LQR_WHEEL_L[1];
					t4[11] = LQR_WHEEL_L[2];
					t4[12] = LQR_WHEEL_L[3];
					t4[13] = LQR_WHEEL_L[4];
					t4[14] = LQR_WHEEL_L[5];
				
				static uint8_t cnt = 0;
				if(cnt > 10)
				{
					Vofa_JustFloat_AUTO(t4);
//					Vofa_JustFloat(t4, 5);
					cnt = 0;
				}
				cnt++;
				#endif
				
				CAN_cmd_MIT(&JOINT_CAN, 0x06, *chassis_send_cmd->joint_motor[0]);  
				CAN_cmd_MIT(&JOINT_CAN, 0x07, *chassis_send_cmd->joint_motor[1]);                              
				CAN_cmd_MIT(&JOINT_CAN, 0x08, *chassis_send_cmd->joint_motor[2]);
				CAN_cmd_MIT(&JOINT_CAN, 0x09, *chassis_send_cmd->joint_motor[3]);
				
				CAN_cmd_MIT(&WHEEL_CAN, 0x10, *chassis_send_cmd->wheel_motor[0]);
				CAN_cmd_MIT(&WHEEL_CAN, 0x11, *chassis_send_cmd->wheel_motor[1]);
				return;
}

fp32 theta_r = 0.0f;
fp32 pid_leg_r = 0.0f;
//跳跃，首先将腿长下降到最低，持续到jump_time个MS，然后再升到最大腿长，最后回复到设置的腿长
void jump_loop_r(chassis_move_t *chassis,vmc_leg_t *vmcr,pid_type_def *leg)
{
	uint8_t jump_time = 20; 
	if(chassis->jump_flag == 1)
	{
		if(chassis->jump_status_r == 0)
		{
			//机体下压
			vmcr->F0 = M / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, MIN_LEG) ;//前馈+pd
			if(vmcr->L0 < MIN_LEG + 0.08f)
			{
				chassis->jump_time_r++;
			}
			if(chassis->jump_time_r>= jump_time && chassis->jump_time_l>= jump_time)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 1;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 1;
			}
		}
		else if(chassis->jump_status_r == 1)
		{
			//起跳
			vmcr->F0 = M / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, MAX_LEG) ;//前馈+pd
			if(vmcr->L0 > MAX_LEG - 0.08f)
			{
				chassis->jump_time_r++;
			}
			if(chassis->jump_time_r >= jump_time && chassis->jump_time_l >= jump_time)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 2;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 2;
			}
		}
		else if(chassis->jump_status_r == 2)
		{
			//腿长回复正常,跳跃完成
			vmcr->F0=M/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) ;//前馈+pd
			if(vmcr->L0<(chassis->leg_right_set + 0.01f))
			{
				chassis->jump_time_r++;
			}
			if(chassis->jump_time_r>=10&&chassis->jump_time_l >= 10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 3;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 3;
			}
		}
		else
		{
			vmcr->F0=M/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) ;//前馈+pd
		}

		if(chassis->jump_status_r == 3&&chassis->jump_status_l == 3)
		{
			chassis->jump_flag = 0;
			chassis->jump_time_r = 0;
			chassis->jump_status_r = 0;
			chassis->jump_time_l = 0;
			chassis->jump_status_l = 0;
		}
	}
	else
	{
		theta_r = M / arm_cos_f32(vmcr->theta);
		pid_leg_r = PID_Calc(leg,vmcr->L0,chassis->leg_right_set);
		vmcr->F0 = theta_r + pid_leg_r + chassis->now_roll_set;
		//vmcr->F0=M/arm_cos_f32(vmcr->theta) + PID_Calc(leg,vmcr->L0,chassis->leg_right_set) - chassis->now_roll_set;//前馈+pd
	}
}
fp32 theta_l = 0.0f;
fp32 pid_leg = 0.0f;
void jump_loop_l(chassis_move_t *chassis,vmc_leg_t *vmcl,pid_type_def *leg)
{
	if(chassis->jump_flag == 1)
	{
		if(chassis->jump_status_l == 0)
		{
			vmcl->F0 = M / arm_cos_f32(vmcl->theta) + PID_Calc(leg, vmcl->L0, MIN_LEG) ;//前馈+pd
			if(vmcl->L0<0.3f)
			{
				chassis->jump_time_l++;
			}
		}
		else if(chassis->jump_status_l == 1)
		{
			vmcl->F0 = M / arm_cos_f32(vmcl->theta) + PID_Calc(leg, vmcl->L0, MAX_LEG) ;//前馈+pd
			if(vmcl->L0>0.5f)
			{
				chassis->jump_time_l++;
			}
		}
		else if(chassis->jump_status_l == 2)
		{
			vmcl->F0 = M / arm_cos_f32(vmcl->theta) + PID_Calc(leg, vmcl->L0, chassis->leg_left_set) ;//前馈+pd
			if(vmcl->L0<(chassis->leg_left_set+0.01f))
			{
				chassis->jump_time_l++;
			}
		}
		else
		{
			vmcl->F0 = M / arm_cos_f32(vmcl->theta) + PID_Calc(leg, vmcl->L0, chassis->leg_left_set) ;//前馈+pd
		}

	}
	else
	{
//		vmcl->F0 = M / arm_cos_f32(vmcl->theta) + PID_Calc(leg, vmcl->L0, chassis->leg_left_set) + chassis->now_roll_set;//前馈+pd
		theta_l = M / arm_cos_f32(vmcl->theta);
		pid_leg = PID_Calc(leg, vmcl->L0, chassis->leg_left_set);
		vmcl->F0 = theta_l + pid_leg + chassis->now_roll_set;
		
	}
}
//平衡底盘观测速度函数
__attribute__((used))void observer(chassis_move_t *chassis_move, KalmanFilter_t *vxEstimateKF, KalmanFilter_t *vyEstimateKF){
		static float wr,wl=0.0f;
		static float vrb,vlb=0.0f;
		static float aver_v=0.0f;
//		static float vel_acc[2] = {0.0f};
		wr = -chassis_move->wheel_motor[1]->fdb.vel + INS.Gyro[0] + chassis_move->right.d_alpha;//右边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		vrb = wr * WHEEL_RAD + chassis_move->right.L0*chassis_move->right.d_theta*arm_cos_f32(chassis_move->right.theta) + chassis_move->right.d_L0 * arm_sin_f32(chassis_move->right.theta);//机体b系的速度
		
		wl = -chassis_move->wheel_motor[0]->fdb.vel - INS.Gyro[0] + chassis_move->left.d_alpha;//左边驱动轮转子相对大地角速度，这里定义的是顺时针为正
		vlb = wl * WHEEL_RAD + chassis_move->left.L0 * chassis_move->left.d_theta * arm_cos_f32(chassis_move->left.theta) + chassis_move->left.d_L0 * arm_sin_f32(chassis_move->left.theta);//机体b系的速度
		
		aver_v = (vrb - vlb)/2.0f;//取平均
    xvEstimateKF_Update(vxEstimateKF, INS.MotionAccel_n[1], aver_v);
		
		//原地自转的过程中v_filter和x_filter应该都是为0
		chassis_move->v_filter = vel_acc[0];//得到卡尔曼滤波后的速度
		chassis_move->x_filter = chassis_move->x_filter + chassis_move->v_filter * ((float)OBSERVE_TIME / 1000.0f);
		//如果想直接用轮子速度，不做融合的话可以这样
		//chassis_move.v_filter=(chassis_move.wheel_motor[0]->para.vel-chassis_move.wheel_motor[1]->para.vel)*(-0.0603f)/2.0f;//0.0603是轮子半径，电机反馈的是角速度，乘半径后得到线速度，数学模型中定义的是轮子顺时针为正，所以要乘个负号
		//chassis_move.x_filter=chassis_move.x_filter+chassis_move.x_filter+chassis_move.v_filter*((float)OBSERVE_TIME/1000.0f);
		
}

void chassis_motor_init(void)
{
	//使能关节电机,同canid的电机是中心对称的
	Motor_enable(&JOINT_CAN, 0x06);
	osDelay(1);
	Motor_enable(&JOINT_CAN, 0x07);
	osDelay(1);
	Motor_enable(&JOINT_CAN, 0x08);
	osDelay(1);
	Motor_enable(&JOINT_CAN, 0x09);
	osDelay(1);
	
	//使能轮毂电机
	Motor_enable(&WHEEL_CAN,0x10);
	osDelay(1);
	Motor_enable(&WHEEL_CAN,0x11);
	osDelay(1);
	
	
//  RS_MOTOR_PRE(&JOINT_CAN, 0x06);	
//	osDelay(1);
//	RS_MOTOR_PRE(&JOINT_CAN, 0x07);
//	osDelay(1);
//	RS_MOTOR_PRE(&JOINT_CAN, 0x08);
//	osDelay(1);
//	RS_MOTOR_PRE(&JOINT_CAN, 0x09);

	
//	Motor_save_zero(&JOINT_CAN, 0x06);
//	osDelay(1);	
//	Motor_save_zero(&JOINT_CAN, 0x07);	
//	osDelay(1);
//	Motor_save_zero(&JOINT_CAN, 0x08);
//	osDelay(1);
//	Motor_save_zero(&JOINT_CAN, 0x09);
//	osDelay(1);

}

#endif
