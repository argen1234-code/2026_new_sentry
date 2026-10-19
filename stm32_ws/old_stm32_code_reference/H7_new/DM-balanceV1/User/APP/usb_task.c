#include "usb_task.h"
#include "main.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"

#include "arm_math.h"

#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "chassis_power_control.h"
#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "bsp_usb.h"
#include "shoot.h"
//#include "voltage_task.h"
#include "CRC8_CRC16.h"
#include "usbd_cdc_if.h"
void usb_update();
void usb_loop();
void usb_send();
void usb_task(){
	osDelay(USB_INIT_TIME);
	//已经初始化到main.c里面了
	while(1){
		//更新信息
		usb_update();
		//计算
		usb_loop();
		//发送消息
		usb_send();
		
	}
	
	
}
