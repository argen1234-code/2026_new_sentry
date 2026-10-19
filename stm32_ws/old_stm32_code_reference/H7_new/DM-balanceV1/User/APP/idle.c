#include "idle.h"
#include "main.h"
#include "cmsis_os.h"

volatile unsigned long idleTime = 0;

void idle_task(void const * argument){
	uint32_t lastWakeTime = osKernelSysTick();
    while (1) {
        // 记录进入空闲任务的时间
        uint32_t enterIdleTime = osKernelSysTick();
				
        // 执行空闲任务的相关操作
        // 这里可以添加您需要在空闲时间执行的代码
			
        // 计算空闲时间（以滴答为单位）
        uint32_t currentTime = osKernelSysTick();
        idleTime += (currentTime - enterIdleTime);

    }
}
