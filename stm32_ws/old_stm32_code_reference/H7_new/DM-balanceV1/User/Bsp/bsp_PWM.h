/**
  ******************************************************************************
  * @file	 bsp_PWM.h
  * @author  Wang Hongxi
  * @version V1.0.0
  * @date    2020/3/1
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef __BSP_IMU_PWM_H
#define __BSP_IMU_PWM_H
#include "struct_typedef.h"
#include "stdint.h"
#include "tim.h"

//旋转到0度，关弹舱
#define Bomb_bay_off 500  
//旋转到90度，开弹舱
#define Bomb_bay_on 1300 

extern void Bombbay_on(void);
extern void Bombbay_off(void);
extern void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value);
#endif
