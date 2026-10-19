#ifndef __OBSERVE_TASK_H
#define __OBSERVE_TASK_H

#include "stdint.h"
#include "INS_task.h"
#include "chassis_task.h"
#include "main.h"
#define OBSERVE_TIME 4 //任务周期是2ms
extern KalmanFilter_t vaEstimateKF;

extern KalmanFilter_t vxEstimateKF;	   // 卡尔曼滤波器结构体
extern KalmanFilter_t vyEstimateKF;	   // 卡尔曼滤波器结构体

extern void Observe_task(void const * argument);
extern void xvEstimateKF_Init(KalmanFilter_t *EstimateKF);
extern void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel);
extern float vel_acc[2]; 
#endif




