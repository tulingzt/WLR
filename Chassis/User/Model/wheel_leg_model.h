#ifndef __WHEEL_LEG_MODEL_H
#define __WHEEL_LEG_MODEL_H

#define GRAVITY 9.81f

//两腿模型
typedef struct
{
	float leg_max, leg_min, leg_width;				//腿长限幅以及两腿间距
	float leg_roll_fdb, imu_roll_fdb, gnd_roll_fdb;	//各翻滚角反馈,旋转向量向前为正方向

	float imu_roll_ref;								//目标roll角度
	float leg_length_ref;
	float leg_length_offset;
	float leg_length_err;
	float l_fdb[2], l_ref[2];
} tlm_t;

//两轮模型
typedef struct
{
	float wheel_width;				//两轮间距
	float wheel_radius;				//轮子半径

	float w_fdb[2], w_ref[2];		//轮子转速(rad/s),旋转向量向右为正方向
	float v_fdb[2], v_ref[2];		//左右轮中心线速度(m/s),向前为正方向
	float v0_ref, v0_fdb;			//机体中心线速度(m/s),向前为正方向
	float w0_ref, w0_fdb;			//机体中心角速度(rad/s),旋转向量向上为正方向
	float w0_imu_fdb;				//机体中心角速度(rad/s),旋转向量向上为正方向

	float r_fdb, r_ref;				//底盘瞬时旋转半径(m),向左为正方向
	float a_fdb, a_ref;				//底盘转弯向心加速度(m/s^2),向左为正方向
	float gravity_compensate_angle; //重力补偿角度,旋转向量向前为正方向
} twm_t;

extern tlm_t tlm;
extern twm_t twm;

void TwoLegModel_Init(tlm_t* p, float Lmax, float Lmin, float Width);
void TwoLegModel_Gnd_Roll_Calc(tlm_t* p, float imu_roll, float Ll_fdb, float Lr_fdb);
void TwoLegModel_Leg_Length_Calc(tlm_t* p, float leg_length_ref, float imu_roll_ref);
void TwoWheelModel_Init(twm_t* twm, float width, float radius);
void TwoWheelModel_Feedback(twm_t* twm, float wr_fdb, float wl_fdb, float imu_wz);
void TwoWheelModel_Reference(twm_t* twm, float v0_ref, float w0_ref);

#endif
