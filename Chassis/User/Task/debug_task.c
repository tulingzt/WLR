#include "debug_task.h"
#include "cmsis_os.h"
#include "data_scope.h"
#include "stdint.h"

#include "wlr.h"
#include "leg_vmc.h"
#include "math_calcu.h"
#include "Timu_driver.h"

uint8_t debug_wave = 2;

void DataWavePkg(void)
{
	switch(debug_wave)
	{
		case 1:
		{
			DataScope_Get_Channel_Data(pid_leg_length[0].i_out);
            DataScope_Get_Channel_Data(pid_leg_length[0].ref);
			DataScope_Get_Channel_Data(pid_leg_length[0].fdb);
            DataScope_Get_Channel_Data(vmc[0].q_fdb[0]);
			break;
		}
        case 2:
        {
            DataScope_Get_Channel_Data(wlr.yaw_set);
            DataScope_Get_Channel_Data(imu.yaw);
            DataScope_Get_Channel_Data(Circle_Error(&wlr.yaw_set, &imu.yaw, 2 * PI));
            DataScope_Get_Channel_Data(wlr.wz_set);
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
