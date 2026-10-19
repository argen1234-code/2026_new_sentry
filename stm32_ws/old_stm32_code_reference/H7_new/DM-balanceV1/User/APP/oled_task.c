/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       oled_task.c/h
  * @brief      OLED show error value.oled屏幕显示错误码
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "oled_task.h"
#include "main.h"
#include "oled.h"

#include "cmsis_os.h"
#include "detect_task.h"
//#include "voltage_task.h"

#define OLED_CONTROL_TIME 10
#define REFRESH_RATE    10

const error_t *error_list_local;

uint8_t other_toe_name[4][4] = {"GYR\0","ACC\0","MAG\0","REF\0"};

uint8_t last_oled_error = 0;
uint8_t now_oled_errror = 0;
static uint8_t refresh_tick = 0;



 fp32 calc_battery_percentage(float voltage)
{
		voltage = (adc_val[0]*3.3f/65535)*11.0f;
    fp32 percentage;
    fp32 voltage_2 = voltage * voltage;
    fp32 voltage_3 = voltage_2 * voltage;
    if(voltage < 19.5f)
    {
        percentage = 0.0f;
    }
    else if(voltage < 21.9f)
    {
        percentage = 0.005664f * voltage_3 - 0.3386f * voltage_2 + 6.765f * voltage - 45.17f;
    }
    else if(voltage < 25.5f)
    {
        percentage = 0.02269f * voltage_3 - 1.654f * voltage_2 + 40.34f * voltage - 328.4f;
    }
    else
    {
        percentage = 1.0f;
    }
    if(percentage < 0.0f)
    {
        percentage = 0.0f;
    }
    else if(percentage > 1.0f)
    {
        percentage = 1.0f;
    }
    //another formulas
    //另一套公式
//    if(voltage < 19.5f)
//    {
//        percentage = 0.0f;
//    }
//    else if(voltage < 22.5f)
//    {
////        percentage = 0.05776f * (voltage - 22.5f) * (voltage_2 - 39.0f * voltage + 383.4f) + 0.5f;
//        percentage = 0.05021f * voltage_3 - 3.075f * voltage_2 + 62.77f * voltage - 427.02953125f;
//    }
//    else if(voltage < 25.5f)
//    {
////        percentage = 0.01822f * (voltage - 22.5f) * (voltage_2 - 52.05f * voltage + 637.0f) + 0.5f;
//        percentage = 0.0178f * voltage_3 - 1.292f * voltage_2 + 31.41f * voltage - 254.903125f;
//    }
//    else
//    {
//        percentage = 1.0f;
//    }

    return percentage;
}

uint16_t get_battery_percentage(void)
{
    return (uint16_t)(calc_battery_percentage(voltage) * 100.0f);
}

/**
  * @brief          oled task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          oled任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void oled_task(void const * argument)
{
    uint8_t i;
    uint8_t show_col, show_row;
    error_list_local = get_error_list_point();
    osDelay(1000);
    OLED_init();
    OLED_LOGO();
    i = 100;
    while(i--)
    {
        if(OLED_check_ack())
        {
            detect_hook(OLED_TOE);
        }
        osDelay(10);
    }
    while(1)
    {
        //use i2c ack to check the oled
        if(OLED_check_ack())
        {
            detect_hook(OLED_TOE);
        }

        now_oled_errror = toe_is_error(OLED_TOE);
        //oled init
        if(last_oled_error == 1 && now_oled_errror == 0)
        {
            OLED_init();
        }

        if(now_oled_errror == 0)
        {
            refresh_tick++;
            //10Hz refresh
            if(refresh_tick > configTICK_RATE_HZ / (OLED_CONTROL_TIME * REFRESH_RATE))
            {
                refresh_tick = 0;
                OLED_operate_gram(PEN_CLEAR);
                OLED_show_graphic(0, 1, &battery_box);

                if(get_battery_percentage() < 10)
                {
                    OLED_printf(9, 2, "%d", get_battery_percentage());
                }
                else if(get_battery_percentage() < 100)
                {
                    OLED_printf(6, 2, "%d", get_battery_percentage());
                }
                else
                {
                    OLED_printf(3, 2, "%d", get_battery_percentage());
                }

                OLED_show_string(90, 27, "DBUS");
                OLED_show_graphic(115, 27, &check_box[error_list_local[DBUS_TOE].error_exist]);
                for(i = CHASSIS_MOTOR1_TOE; i < TRIGGER_MOTOR_TOE + 1; i++)
                {
                    show_col = ((i-1) * 32) % 128;
                    show_row = 15 + (i-1) / 4 * 12;
                    OLED_show_char(show_col, show_row, 'M');
                    OLED_show_char(show_col + 6, show_row, '0'+i);
                    OLED_show_graphic(show_col + 12, show_row, &check_box[error_list_local[i].error_exist]);
                }

                for(i = BOARD_GYRO_TOE; i < REFEREE_TOE + 1; i++)
                {
                    show_col = (i * 32) % 128;
                    show_row = 15 + i / 4 * 12;
                    OLED_show_string(show_col, show_row, other_toe_name[i - BOARD_GYRO_TOE]);
                    OLED_show_graphic(show_col + 18, show_row, &check_box[error_list_local[i].error_exist]);

                }

                OLED_refresh_gram();
            }
        }



        last_oled_error = now_oled_errror;
        osDelay(OLED_CONTROL_TIME);
    }
}

