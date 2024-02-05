#include "debug_task.h"
#include "cmsis_os.h"
#include "data_scope.h"
#include "stdint.h"

uint8_t debug_wave = 1;

void DataWavePkg(void)
{
	switch(debug_wave)
	{
		case 1:
		{
//			DataScope_Get_Channel_Data(imu_data.pitch);
//			DataScope_Get_Channel_Data(imu_data.wy);
			break;
		}
		default:break;
		
	}
}

/* 串口上位机数据发送任务 */
void debug_task(void const *argu)
{
	uint32_t thread_wake_time = osKernelSysTick();
	for(;;)
	{
		taskENTER_CRITICAL();
		DataWave(&huart5);
		taskEXIT_CRITICAL();
		osDelayUntil(&thread_wake_time, 1);
	}
}
