/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-07-13 17:52:17
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-07-14 16:18:21
 * @FilePath: \smartcare:\X15\stm32\my_example\RM\H7\DM-balanceV1\User\Bsp\bsp_PWM.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "bsp_buzzer.h"
#include "bsp_PWM.h"
#include "main.h"
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim4;

void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim12, psc);
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, pwm);

}
void buzzer_off(void)
{
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, 0);
}
