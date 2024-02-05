//模块使用的定时器，需要通过设置预分频系数，使得其每1us改变一次
//使用方法：
//    0. 配置好1us计数一次的定时器，修改.h文件中的 UST_PRECISION、UST_HTIM、UST_UST_TIM_CNT_SIZE
//    1. 在溢出中断中放入如下代码，修改成相应的定时器
//        if (htim->Instance == TIM5)
//        {
//            prv_ust.overflow_cnt++;
//        }
//    2. 在 main() 中调用 ust_tim_start(); 使能定时器及其中断
//    3. 在需要计时/延时的文件中声明变量，如： ust_t ust_user;
//    4. 计时
//        区间计时：用于统计某段代码的执行时间
//            ust_interval_test_start(&ust_user);
//                ......
//            ust_interval_test_end(&ust_user);
//        周期计时：用于统计某处代码的执行周期
//            ust_period_test(&ust_user);
//                ......
//    5. 查看计时结果 ust_user.dt
//    6. 如使用完定时器，可以使用 ust_tim_end(); 失能定时器
#ifndef __US_TIME_H
#define __US_TIME_H

#include "stdint.h"
#include "stm32f4xx_hal.h"

//模块参数设置
#define US_TIME_PRECISION			1		//定时器精度，单位：us 计数值加一的一次时间
#define US_TIME_HTIM				htim5	//使用的定时器句柄 stm32f4只有tim2 tim5是32位，其余都是16位
#define US_TIME_CNT_SIZE	32		//定时器ARR寄存器位数,可通过查芯片手册时钟树获得

#if (US_TIME_CNT_SIZE == 32)
	#define US_TIME_PERIOD 0xffffffff		//定时器溢出周期数值
	#define US_TIME_TYPE uint32_t
#elif (US_TIME_CNT_SIZE == 16)
	#define US_TIME_PERIOD 0xffff			//定时器溢出周期数值
	#define US_TIME_TYPE uint16_t
#endif

typedef struct
{
	US_TIME_TYPE last_time;
	US_TIME_TYPE last_cnt;
	US_TIME_TYPE now_time;
	US_TIME_TYPE now_cnt;
	float dt;							//单位：ms
	uint8_t interval_start_flag;		//区间测时启动标志
} us_time_t;

typedef struct
{
	__IO US_TIME_TYPE overflow_cnt;			//当前定时器溢出次数
	__IO US_TIME_TYPE predict_overflow_cnt;	//预测延时结束时定时器溢出次数
	__IO US_TIME_TYPE predict_time;			//预测延时结束时定时器的计数值
} prv_us_time_t;

extern prv_us_time_t prv_us_time;
extern TIM_HandleTypeDef US_TIME_HTIM;

void usTime_Start(void);
void usTime_End(void);
float usTime_Period_Test(us_time_t* us_time);
void usTime_Interval_Test_Start(us_time_t* us_time);
float usTime_Interval_Test_End(us_time_t* us_time);
void usTime_Delay(US_TIME_TYPE us);

#endif
