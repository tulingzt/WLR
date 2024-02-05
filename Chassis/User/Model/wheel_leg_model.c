#include "wheel_leg_model.h"
#include "arm_math.h"
#include "math_calcu.h"
#include "kalman_filter.h"

kalman1_param_t kal_v0_fdb;
kalman1_param_t kal_w0_fdb;
kalman1_param_t kal_gnd_roll;
twm_t twm;
tlm_t tlm;

void TwoLegModel_Init(tlm_t* p, float Lmax, float Lmin, float Width)
{
	p->leg_max = Lmax;
	p->leg_min = Lmin;
	p->leg_width = Width;
	Kalman1_Filter_Create(&kal_gnd_roll, 1, 100);
}

void TwoLegModel_Gnd_Roll_Calc(tlm_t* p, float imu_roll, float Ll_fdb, float Lr_fdb)
{
	//解算地面倾角
	p->l_fdb[0] = Ll_fdb;
	p->l_fdb[1] = Lr_fdb;
	p->imu_roll_fdb = imu_roll;//获取imu绝对倾角
	p->leg_roll_fdb = atanf((p->l_fdb[0] - p->l_fdb[1]) / p->leg_width);
	p->gnd_roll_fdb = p->imu_roll_fdb - p->leg_roll_fdb;
	p->gnd_roll_fdb = Kalman1_Filter_Calc(&kal_gnd_roll, p->gnd_roll_fdb);
}

void TwoLegModel_Leg_Length_Calc(tlm_t* p, float leg_length_ref, float imu_roll_ref)
{
	p->leg_length_ref = leg_length_ref;
	//计算腿长补偿值
	if (ABS(p->gnd_roll_fdb) > 0.1745f)	//当倾斜角大于10°时，roll控制为水平
		p->imu_roll_ref = 0;
	else								//当在接近水平的地面上时，进行过弯补偿/翻滚角自定控制
		p->imu_roll_ref = imu_roll_ref;
	p->leg_length_offset = p->leg_width * tanf(p->imu_roll_ref - p->gnd_roll_fdb);//计算补偿腿长
	//腿长设定值计算
	p->l_ref[0] = p->leg_length_ref + p->leg_length_offset / 2;
	p->l_ref[1] = p->leg_length_ref - p->leg_length_offset / 2;
	//腿长设定值超限分配处理 以保持角度为主，超限改变目标高度但不能超过限幅值
	if(ABS(p->leg_length_offset) >= p->leg_max - p->leg_min)//角度过大，超过可调整范围，用限幅值
	{
		if(p->leg_length_offset > 0)
		{
			p->l_ref[0] = p->leg_max;
			p->l_ref[1] = p->leg_min;
		}
		else
		{
			p->l_ref[0] = p->leg_min;
			p->l_ref[1] = p->leg_max;
		}
	}
	else//在可调整范围内，以保持角度为主
	{
		if(p->l_ref[0] > p->leg_max)//左腿长超上限
		{
			p->leg_length_err = p->l_ref[0] - p->leg_max;
			p->l_ref[1] -= p->leg_length_err;
			p->l_ref[0] = p->leg_max;
		}
		else if(p->l_ref[1] > p->leg_max)//右腿长超上限
		{
			p->leg_length_err = p->l_ref[1] - p->leg_max;
			p->l_ref[0] -= p->leg_length_err;
			p->l_ref[1] = p->leg_max;
		}
		else if(p->l_ref[0] < p->leg_min)//左腿长超下限
		{ 
			p->leg_length_err = p->leg_min - p->l_ref[0];
			p->l_ref[1] += p->leg_length_err;
			p->l_ref[0] = p->leg_min;
		}
		else if(p->l_ref[1] < p->leg_min)//右腿长超下限
		{ 
			p->leg_length_err = p->leg_min - p->l_ref[1];
			p->l_ref[0] += p->leg_length_err;
			p->l_ref[1] = p->leg_min;
		}
	}
}

void TwoWheelModel_Init(twm_t* twm, float width, float radius)
{
	twm->wheel_width = width;
	twm->wheel_radius = radius;
	Kalman1_Filter_Create(&kal_v0_fdb, 1, 100);
	Kalman1_Filter_Create(&kal_w0_fdb, 1, 100);
}

void TwoWheelModel_Feedback(twm_t* twm, float wl_fdb, float wr_fdb, float imu_wz)
{
	twm->w_fdb[0] = wl_fdb;
	twm->w_fdb[1] = wr_fdb;
	twm->w0_imu_fdb = Kalman1_Filter_Calc(&kal_w0_fdb, imu_wz);
	
	twm->v_fdb[0] = -twm->w_fdb[0] * twm->wheel_radius;
	twm->v_fdb[1] = -twm->w_fdb[1] * twm->wheel_radius;
	twm->w0_fdb = (twm->v_fdb[1] - twm->v_fdb[0]) / twm->wheel_width;
	twm->v0_fdb = Kalman1_Filter_Calc(&kal_v0_fdb, (twm->v_fdb[0] + twm->v_fdb[1]) / 2);
	
	if(ABS(twm->w0_imu_fdb) < 1e-2f || ABS(twm->v0_fdb) < 0.3f)//转弯半径过大，向心加速度为零
	{
		twm->r_fdb = 0;
		twm->a_fdb = 0;
	}
	else
	{
		twm->r_fdb = -twm->v0_fdb / twm->w0_imu_fdb;
		twm->a_fdb  = powf(twm->w0_imu_fdb, 2) * twm->r_fdb;
	}
	twm->gravity_compensate_angle = -atanf(twm->a_fdb/GRAVITY); // 补偿过大容易失控
}

void TwoWheelModel_Reference(twm_t* twm, float v0_ref, float w0_ref)
{
	twm->v0_ref = v0_ref;
	twm->w0_ref = w0_ref;
	
	twm->v_ref[0] = twm->v0_ref - twm->wheel_width * twm->w0_ref / 2;
	twm->v_ref[1] = twm->v0_ref + twm->wheel_width * twm->w0_ref / 2;
	
	twm->w_ref[0] = -twm->v_ref[0] / twm->wheel_radius;
	twm->w_ref[1] = -twm->v_ref[1] / twm->wheel_radius;
	
	if(ABS(twm->w0_ref) < 1e-2f)//转弯半径过大，向心加速度为零
	{
		twm->r_ref = 0;
		twm->a_ref = 0;
	}
	else
	{
		twm->r_ref = -twm->v0_ref / twm->w0_ref;
		twm->a_ref = powf(twm->w0_ref, 2) * twm->r_ref;
	}
}
