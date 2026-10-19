/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-07-09 21:22:21
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-07-14 16:19:12
 * @FilePath: \smartcare:\X15\stm32\my_example\RM\H7\DM-balanceV1\User\Bsp\bsp_PWM.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
  ******************************************************************************
  * @file	 bsp_PWM.c
  * @author  Wang Hongxi
  * @version V1.0.0
  * @date    2020/3/1
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "bsp_PWM.h"
extern TIM_HandleTypeDef htim4;
void Bombbay_off(void)
{
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, Bomb_bay_off);
}

void Bombbay_on(void)
{
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, Bomb_bay_on);

}

void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value)
{
    if (value > tim_pwmHandle->Instance->ARR)
        value = tim_pwmHandle->Instance->ARR;

    switch (Channel)
    {
    case TIM_CHANNEL_1:
        tim_pwmHandle->Instance->CCR1 = value;
        break;
    case TIM_CHANNEL_2:
        tim_pwmHandle->Instance->CCR2 = value;
        break;
    case TIM_CHANNEL_3:
        tim_pwmHandle->Instance->CCR3 = value;
        break;
    case TIM_CHANNEL_4:
        tim_pwmHandle->Instance->CCR4 = value;
        break;
    }
}
