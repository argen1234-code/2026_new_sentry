/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_usart.h"
#include "can_bsp.h"
#include "usart.h"
#include "ins_task.h"
#include "remote_control.h"
#include "chassis_power_control.h"
#include "calibrate_task.h"
#include "message_task.h"
#include "referee_usart_task.h"
#include "robot_param.h"
#include "bsp_usb.h"
#include "observer.h"
#include "UI.h"
#include "auto_aim.h"
#include "comm_app.h"
#include "task_profiler_sampling.h"
#include "robot_param.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
osThreadId calibrate_tast_handle;
osThreadId chassisTaskHandle;
osThreadId detect_handle;
osThreadId gimbalTaskHandle;
osThreadId imuTaskHandle;
osThreadId led_RGB_flow_handle;
osThreadId oled_handle;
osThreadId referee_usart_task_handle;
osThreadId usb_task_handle;
osThreadId battery_voltage_handle;
osThreadId servo_task_handle;
osThreadId message_task_handle;
osThreadId idle_task_handle;
osThreadId testHandle;
osThreadId observer_Handle;
osThreadId UI_Handle;
osThreadId auto_aim_Handle;
osThreadId comm_app_handle;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

		
    #if ROBOT_BOARD == chassis_board
		
    osThreadDef(ChassisTask, chassis_task, osPriorityHigh , 0, 1024);
    chassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);
		
    osThreadDef(REFEREE, referee_usart_task, osPriorityNormal, 0, 128);
    referee_usart_task_handle = osThreadCreate(osThread(REFEREE), NULL);
		
		osThreadDef(UI, UI_task, osPriorityNormal, 0, 128);
    UI_Handle = osThreadCreate(osThread(UI), NULL);
		
//		osThreadDef(Observer, Observe_task, osPriorityNormal, 0, 128);
//    observer_Handle = osThreadCreate(osThread(Observer), NULL);
		
    #else 
		
    osThreadDef(gimbalTask, gimbal_task, osPriorityHigh, 0, 1024);
    gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);
		
		osThreadDef(cali, calibrate_task, osPriorityNormal, 0, 512);
    calibrate_tast_handle = osThreadCreate(osThread(cali), NULL);
		
		//自瞄进程预留
		osThreadDef(auto_aim, auto_aim_task, osPriorityNormal, 0, 256);
    auto_aim_Handle = osThreadCreate(osThread(auto_aim), NULL);
		
		 osThreadDef(COMM_APP, comm_app_task, osPriorityBelowNormal, 0, 640);
		comm_app_handle = osThreadCreate(osThread(COMM_APP), NULL);
		
		#endif 
		
		osThreadDef(message, message_task, osPriorityNormal, 0, 128);
		message_task_handle = osThreadCreate(osThread(message), NULL);
		
    osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 1024);
    imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);
		


  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* Communication hello world demo */

	
	
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
