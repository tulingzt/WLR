#include "wlr.h"
#include "leg_vmc.h"
#include "wheel_leg_model.h"
#include "pid.h"
#include "kalman_filter.h"
#include "math_calcu.h"
#include "math_matrix.h"

#define WLR_SIGN(x) ((x) > 0? (1): (-1))

#define CHASSIS_PERIOD_DU 2

const float LegLengthParam[5] = {0.150f, 0.270f, 0.270f, 0.150f, 0.150f};
float mb = 5.8f, ml = 2.09f, mw = 0.715f;//机体质量 腿部质量 轮子质量 14.5
const float BodyWidth = 0.42f;//两轮间距
const float WheelRadius = 0.06f;//轮子半径
const float LegLengthMax = 0.30f, LegLengthMin = 0.15f;

const float LegLengthJump1 = 0.15f;//压腿
const float LegLengthJump2 = 0.35f;//蹬腿
const float LegLengthJump3 = 0.24f;//收腿
const float LegLengthJump4 = 0.22f;//落地
const float LegLengthFly = 0.20f;//腾空
const float LegLengthHigh = 0.30f;//长腿
const float LegLengthNormal = 0.20f;//正常
float LegCanChange = 0.30f;

float x3_balance_zero = 0.03, x5_balance_zero = -0.075f;//腿摆角角度偏置 机体俯仰角度偏置

//								位移  速度	角度	角速度  角度	角速度
float K_Array_Wheel[2][6] =		{{60, 30, 80, 8, 300, 10}, 
                                { -0, -0.7, -8, -1, 3, 2}};
float K_Array_Leg[2][6] =		{{6.08535, 12.6636, 49.1418, 7.57203, 54.8073, 
  12.7387}, {-0.793527, -1.75976, -13.1544, -1.78789, 4.8796, 
  1.66868}};
float K_Array_Fly[2][6] =		{{0, 0, 80, 10, 0, 0}, { 0, 0, 0, 0, 0, 0}};
float K_Array_test[2][6];
float K_Array_test2[2][6];
float K_Array_List[12][4] = 
{{7.68012,-1.69684,-43.5662,60.9093},
{16.6469,-10.8331,-65.3342,99.5026},
{34.3366,210.496,-865.873,917.825},
{7.23228,7.25271,-34.6399,34.5094},
{36.9872,98.2124,-20.8582,-124.761},
{5.55616,45.199,-46.6075,0.556525},
{-0.607925,-0.846185,-0.984613,2.89844},
{-1.50394,-0.423433,-6.27517,10.0366},
{-6.2951,-44.5144,57.9034,-34.2354},
{-1.42809,0.334929,-13.2375,12.7807},
{6.72881,-7.2875,-15.4735,28.3466},
{2.56242,-5.24235,3.84217,0.0953686}};
float K_Array_List_lq[12][4][4] =
    {{{3.77231,0.10665,-0.000734326,1.08329*10e-6},{-22.5131,0.379771,-0.00223925,0},{-51.5864,0.0626309,0,0},{95.7525,0,0,0}},
    {{6.55654,0.28768,-0.00194083,2.62203*10e-6},{-62.2379,0.656906,-0.00396376,0},{-15.6985,0.149801,0,0},{69.5323,0,0,0}},
    {{-26.7089,1.73478,-0.0121138,0.0000179585},{-93.9237,4.28115,-0.0246058,0},{-718.719,0.461446,0,0},{1050.1,0,0,0}},
    {{4.92196,0.0971406,-0.000767493,1.71201*10e-6},{-45.9094,0.774072,-0.00447654,0},{24.9386,0.0919802,0,0},{-40.5398,0,0,0}},
    {{77.7259,-1.09158,0.00685009,-6.07801*10e-6},{130.173,1.17482,-0.00577295,0},{-309.242,-0.331598,0,0},{197.951,0,0,0}},
    {{16.8212,-0.32015,0.00202891,-1.98462*10e-6},{68.4338,0.245127,-0.00104885,0},{-174.855,-0.137915,0,0},{165.462,0,0,0}},
    {{-1.11788,0.0132055,-0.0000830464,7.39729*10e-8},{-1.0292,-0.0117914,0.0000574456,0},{0.821395,0.00360428,0,0},{1.83391,0,0,0}},
    {{-2.45163,0.0211518,-0.000130714,9.59569*10e-8},{0.137466,0.00400322,-0.0000253428,0},{-12.238,0.00114359,0,0},{22.3991,0,0,0}},
    {{-4.48261,-0.0828074,0.00090738,-3.13362*10e-6},{57.2848,-2.16611,0.012055,0},{82.6549,-0.0445601,0,0},{-91.5498,0,0,0}},
    {{-1.99144,0.00696216,-0.0000312743,-5.68663*10e-8},{-0.0260269,0.0885546,-0.000479525,0},{-31.0747,-0.00850917,0,0},{42.4909,0,0,0}},
    {{4.41499,0.0769525,-0.000542991,9.09041*10e-7},{-32.4936,0.342664,-0.00204812,0},{11.2181,0.0678215,0,0},{3.03071,0,0,0}},
    {{2.76005,0.00439248,-0.0000540965,2.46472*10e-7},{-14.1031,0.104712,-0.00063767,0},{19.2294,0.0256985,0,0},{-21.4065,0,0,0}}};

wlr_t wlr;
lqr_t lqr[2];

kalman1_param_t kal_leg_q0[2], kal_leg_w0[2], kal_fn[2];

pid_t pid_leg_length[2];
pid_t pid_leg_length_fast[2];
pid_t pid_q0, pid_roll, pid_wz;

static float WLR_Fn_Calc(float az, float Fy_fdb, float T0_fdb, float L0[3], float theta[3])
{
	float Fwy = Fy_fdb * cosf(theta[0]) + T0_fdb * sinf(theta[0]) / L0[0];//轮子受到腿部机构竖直方向的作用力
	float yw_ddot = az
					- L0[2] * cosf(theta[0])
					+ 2 * L0[1] * theta[1] * sinf(theta[0])
					+ L0[0] * theta[2] * sinf(theta[0])
					+ L0[0] * powf(theta[1], 2) * cosf(theta[0]);//轮子竖直方向的加速度
	return Fwy + mw * GRAVITY + mw * yw_ddot;
}

void K_Array_Update(float K[2][6], float high_set)
{
    for(int i = 0; i < 2; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            K[i][j] = K_Array_List[i * 6 + j][0] + 
                      K_Array_List[i * 6 + j][1] * high_set +
                      K_Array_List[i * 6 + j][2] * high_set * high_set +
                      K_Array_List[i * 6 + j][3] * high_set * high_set * high_set;
        }
    }
}

void K_Array_Updata2(float high_fdb, float q0_fdb)
{
    for(int i = 0; i < 2; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            K_Array_test2[i][j] = 0;
            for(int x = 0; x <= 3; x++)
            {
                for(int y = 0; y <= 3; y++)
                {
                    K_Array_test2[i][j] += (K_Array_List_lq[i * 6 + j][x][y] * powf(high_fdb, x) * powf(q0_fdb, y));
                }
            }
        }
    }
}

//float test_p=500,test_i=1.5,test_d=8000,test_iout=10,test_oout=20;

void WLR_Init(void)
{
	wlr.max_speed_error = 1.2f;
	wlr.max_stop_angle = 10;
	wlr.max_wz_error = 0.6f;
	wlr.high_set = LegLengthNormal;
	wlr.roll_set = 0;
	wlr.q0_set = PI / 2;
	
	TwoWheelModel_Init(&twm, BodyWidth, WheelRadius);
	TwoLegModel_Init(&tlm, LegLengthMax, LegLengthMin, BodyWidth);
	for(int i = 0; i < 2; i++)
	{
		//腿部长度初始化
		VMC_Init(&vmc[i], LegLengthParam);
		//卡尔曼滤波器初始化
		Kalman1_Filter_Create(&kal_leg_q0[i], 1, 20);
		Kalman1_Filter_Create(&kal_leg_w0[i], 1, 5);
		Kalman1_Filter_Create(&kal_fn[i], 1, 200);
		//PID参数初始化
		PID_Init(&pid_leg_length[i], 1000, 1.5f, 10000, 10, 20);
		PID_Init(&pid_leg_length_fast[i], 1000, 0, 10000, 30, 50);
	}
	//卡尔曼滤波器初始化
	
	//PID参数初始化
	PID_Init(&pid_wz, 2.0f, 0, 10.0f, 0, 2.5f);//与LQR的速度控制协同
	PID_Init(&pid_q0, 60, 0, 100, 0, 10);//与LQR的虚拟腿摆角控制拮抗 60 0 120
	PID_Init(&pid_roll, 500, 0, 4000, 0, 15);//与VMC的腿长控制协同  1000 0 3500
}

void WLR_Protest(void)
{
	pid_leg_length[0].i_out = 0;
	pid_leg_length[1].i_out = 0;
}

float vmc_q0_diff;

//轮子：位移、速度   摆角：角度、角速度   机体俯仰：角度、角速度
void WLR_Control(void)
{
	//------------------------反馈数据更新------------------------//
	//更新两轮模型
	TwoWheelModel_Feedback(&twm, wlr.side[0].wy, wlr.side[1].wy, wlr.wz_fdb);//输入左右轮子转速
	TwoWheelModel_Reference(&twm, wlr.v_set, wlr.wz_set);//计算两侧轮腿模型的设定速度
	//两侧轮腿分别更新数据
	for(int i = 0; i < 2; i++)
	{
		//更新腿部VMC模型
		VMC_Forward_Solution(&vmc[i], wlr.side[i].q1, wlr.side[i].q4, wlr.side[i].w1, \
									  wlr.side[i].w4, wlr.side[i].t1, wlr.side[i].t4);
//		wlr.side[i].q0_kal = Kalman1_Filter_Calc(&kal_leg_q0[i], vmc[i].q_fdb[0]);
//		wlr.side[i].w0_kal = Kalman1_Filter_Calc(&kal_leg_w0[i], vmc[i].V_fdb.e.w0_fdb);
		//LQR输入反馈值
		lqr[i].last_x2 = lqr[i].X_fdb[1];
		lqr[i].X_fdb[1] = -wlr.side[i].wy * WheelRadius;
		lqr[i].X_fdb[0] += (lqr[i].X_fdb[1] + lqr[i].last_x2) / 2 * CHASSIS_PERIOD_DU * 0.001f;//使用梯形积分速度求位移
		lqr[i].X_fdb[4] = x5_balance_zero + wlr.pit_fdb;
		lqr[i].X_fdb[5] = wlr.wy_fdb;
		lqr[i].X_fdb[2] = x3_balance_zero + (PI / 2 - lqr[i].X_fdb[4] - vmc[i].q_fdb[0]);
		lqr[i].X_fdb[3] = lqr[i].X_fdb[5] - vmc[i].V_fdb.e.w0_fdb;
		lqr[i].dot_x4 = (lqr[i].X_fdb[3] - lqr[i].last_x4) / (CHASSIS_PERIOD_DU * 0.001f); //腿倾角加速度(状态变量x4的dot)计算
		lqr[i].last_x4 = lqr[i].X_fdb[3];
		Data_Limit(&lqr[i].X_fdb[0], 0.5f, -0.5f);//位移限幅  位移系数主要起到一个适应重心的作用 不用太大
//		if(ABS(wlr.v_set) > 1e-3f || ABS(lqr[i].X_fdb[1]) > 0.2f)//有输入速度 或 轮子速度还比较高时 将位移反馈置0  不发挥作用
//			lqr[i].X_fdb[0] = 0;
        vmc_q0_diff = vmc[0].q_fdb[0] - vmc[1].q_fdb[0];
        if(ABS(wlr.v_set) > 1e-3f || ABS(wlr.wz_set) > 0.1f || ABS(vmc[0].q_fdb[0] - vmc[1].q_fdb[0]) > 0.01f)//有输入速度 或 轮子速度还比较高时 将位移反馈置0  不发挥作用
			lqr[i].X_fdb[0] = 0;
		//支持力解算
		float L0_array[3] = {vmc[i].L_fdb, vmc[i].V_fdb.e.vy0_fdb, vmc[i].Acc_fdb.L0_ddot};
		float theta_array[3] = {lqr[i].X_fdb[2], lqr[i].X_fdb[3], lqr[i].dot_x4};
		wlr.side[i].Fn_fdb = WLR_Fn_Calc(wlr.az_fdb, vmc[i].F_fdb.e.Fy_fdb, vmc[i].F_fdb.e.T0_fdb, L0_array, theta_array);
		wlr.side[i].Fn_kal = Kalman1_Filter_Calc(&kal_fn[i], wlr.side[i].Fn_fdb);
		//离地检测
		if(wlr.side[i].Fn_kal < 17.0f)
			wlr.side[i].fly_flag = 1;
		else
			wlr.side[i].fly_flag = 0;
	}
	//高度选择 跳跃状态改变
	if(wlr.jump_flag == 1)//跳跃起跳状态 先压腿
	{
		wlr.high_set = LegLengthJump1;
		if(vmc[0].L_fdb < LegLengthJump1 && vmc[1].L_fdb < LegLengthJump1)
			wlr.jump_flag = 2;
	}
	else if(wlr.jump_flag == 2)//起跳 弹腿
	{
		wlr.high_set = LegLengthJump2;
		if(vmc[0].L_fdb > LegLengthJump2 && vmc[1].L_fdb > LegLengthJump2)
			wlr.jump_flag = 3;
	}
	else if(wlr.jump_flag == 3)//收腿
	{
		wlr.high_set = LegLengthJump3;
		if(vmc[0].L_fdb < LegLengthJump3 && vmc[1].L_fdb < LegLengthJump3 && !wlr.side[0].fly_flag && !wlr.side[1].fly_flag)
			wlr.jump_flag = 4;
	}
	else if(wlr.jump_flag == 4)//落地
	{
		wlr.high_set = LegLengthJump4;
		wlr.jump_cnt++;
		if(wlr.jump_cnt > 200)
		{
			wlr.jump_flag = 0;
			wlr.jump_cnt = 0;
		}
	}
	else if(wlr.side[0].fly_flag && wlr.side[1].fly_flag)//腾空
		wlr.high_set = LegLengthFly;
	else 
		if(wlr.high_mode)//大高度
		wlr.high_set = LegCanChange;
	else
		wlr.high_set = LegLengthNormal;
	//更新两腿模型
	TwoLegModel_Gnd_Roll_Calc(&tlm, -wlr.roll_fdb, vmc[0].L_fdb, vmc[1].L_fdb);//计算地形倾角
	if(wlr.jump_flag != 0 || (wlr.side[0].fly_flag && wlr.side[1].fly_flag))
		tlm.l_ref[0] = tlm.l_ref[1] = wlr.high_set;
	else
		TwoLegModel_Leg_Length_Calc(&tlm, wlr.high_set, 0);//计算腿长设定值
	//------------------------状态选择------------------------//
	//根据当前状态选择合适的控制矩阵
	if(wlr.side[0].fly_flag && wlr.side[1].fly_flag)//腾空
	{
		aMartix_Cover(lqr[0].K, (float*)K_Array_Fly, 2, 6);
		aMartix_Cover(lqr[1].K, (float*)K_Array_Fly, 2, 6);
	}
	else if(wlr.side[0].fly_flag == 0 && wlr.side[1].fly_flag == 0)//在地面
	{
//        if(!wlr.shift_mode)//正常情况
//        {
//            aMartix_Cover(lqr[0].K, (float*)K_Array_Leg, 2, 6);
//            aMartix_Cover(lqr[1].K, (float*)K_Array_Leg, 2, 6);
//        }
//        else//加速
//        {
//            aMartix_Cover(lqr[0].K, (float*)K_Array_Leg, 2, 6);
//            aMartix_Cover(lqr[1].K, (float*)K_Array_Leg, 2, 6);
//        }
        if(wlr.control_mode == 1)		//力控
        {
            K_Array_Update(K_Array_test, vmc[0].L_fdb);
            aMartix_Cover(lqr[0].K, (float*)K_Array_test, 2, 6);
            K_Array_Update(K_Array_test, vmc[1].L_fdb);
			aMartix_Cover(lqr[1].K, (float*)K_Array_test, 2, 6);
            
//            aMartix_Cover(lqr[0].K, (float*)K_Array_Leg, 2, 6);
//            aMartix_Cover(lqr[1].K, (float*)K_Array_Leg, 2, 6);
		}
		else if(wlr.control_mode == 2)	//位控
		{
			aMartix_Cover(lqr[0].K, (float*)K_Array_Wheel, 2, 6);
			aMartix_Cover(lqr[1].K, (float*)K_Array_Wheel, 2, 6);
		}
	}
	//------------------------控制数据更新------------------------//
	//全身运动控制
	wlr.q0_offs   = PID_Calc(&pid_q0, vmc[0].q_fdb[0], vmc[1].q_fdb[0]);//双腿摆角同步控制
	wlr.roll_offs = PID_Calc(&pid_roll, wlr.roll_set, wlr.roll_fdb);
	if(wlr.wz_set > wlr.wz_fdb + wlr.max_wz_error)//z轴速度窗口 很重要
		wlr.wz_set = wlr.wz_fdb + wlr.max_wz_error;
	else if(wlr.wz_set < wlr.wz_fdb - wlr.max_wz_error)
		wlr.wz_set = wlr.wz_fdb - wlr.max_wz_error;
	wlr.wz_offs   = PID_Calc(&pid_wz, wlr.wz_fdb, wlr.wz_set);//Yaw控制
	//两侧轮腿分别独立控制
	for(int i = 0; i < 2; i++)
	{
		//LQR输入控制值 计算得出轮子与腿的力矩
		lqr[i].X_ref[1] = twm.v_ref[i];
//		if(lqr[i].X_ref[1] > lqr[i].X_fdb[1] + wlr.max_speed_error)//最大速度误差设定 相当于给目标速度一个窗口
//			lqr[i].X_ref[1] = lqr[i].X_fdb[1] + wlr.max_speed_error;
//		else if(lqr[i].X_ref[1] < lqr[i].X_fdb[1] - wlr.max_speed_error)
//			lqr[i].X_ref[1] = lqr[i].X_fdb[1] - wlr.max_speed_error;
//		if(wlr.stop_mode)
//		{
//			//刹车模式 摆角目标值变为前进方向的-10度
//			if(lqr[i].X_fdb[1] > 0.5f)
//				lqr[i].X_ref[2] = - wlr.max_stop_angle * PI / 180;
//			else if(lqr[i].X_fdb[1] < -0.5f)
//				lqr[i].X_ref[2] = wlr.max_stop_angle * PI / 180;
//			else
//				lqr[i].X_ref[2] = 0;
//		}
//		else
			lqr[i].X_ref[2] = 0;
		aMartix_Add(1, lqr[i].X_ref, -1, lqr[i].X_fdb, lqr[i].X_diff, 6, 1);
		aMartix_Mul(lqr[i].K, lqr[i].X_diff, lqr[i].U_ref, 2, 6, 1);
//		pid_leg_length[i].kp = test_p;//测试用
//		pid_leg_length[i].ki = test_i;
//		pid_leg_length[i].kd = test_d;
//		pid_leg_length[i].i_max = test_iout;
//		pid_leg_length[i].out_max = test_oout;
		//腿部虚拟力控制
		if(wlr.jump_flag == 2 || wlr.jump_flag == 4)						//跳跃蹬腿阶段 响应要大
			wlr.side[i].Fy = PID_Calc(&pid_leg_length_fast[i], tlm.l_ref[i], vmc[i].L_fdb)\
								+ mb * GRAVITY / 2 + WLR_SIGN(i) * wlr.roll_offs;
		else if(wlr.jump_flag == 3)											//跳跃收腿阶段 响应要大
			wlr.side[i].Fy = PID_Calc(&pid_leg_length_fast[i], tlm.l_ref[i], vmc[i].L_fdb);
		else if(wlr.side[0].fly_flag && wlr.side[1].fly_flag)				//浮空收腿 响应不用那么大
		{
			wlr.side[i].Fy = PID_Calc(&pid_leg_length_fast[i], tlm.l_ref[i], vmc[i].L_fdb) - ml * GRAVITY;
			wlr.wz_offs = 0;
		}
		else																//常态 跳跃压腿阶段 跳跃落地阶段
			wlr.side[i].Fy = PID_Calc(&pid_leg_length[i], tlm.l_ref[i], vmc[i].L_fdb)\
								+ mb * GRAVITY / 2 + WLR_SIGN(i) * wlr.roll_offs;
		wlr.side[i].T0 = -lqr[i].U_ref[0] / 2 + WLR_SIGN(i) * wlr.q0_offs;	//两条腿时，LQR输出力矩需要除以2
		VMC_Inverse_Solution(&vmc[i], wlr.high_set, wlr.q0_set, wlr.side[i].T0, wlr.side[i].Fy);
	}
	//------------------------控制数据输出------------------------//
	for(int i = 0; i < 2; i++)
	{
		wlr.side[i].T1 =  vmc[i].T_ref.e.T1_ref;
		wlr.side[i].T4 =  vmc[i].T_ref.e.T4_ref;
		wlr.side[i].Tw = -lqr[i].U_ref[1] + WLR_SIGN(i) * wlr.wz_offs;
		wlr.side[i].P1 =  vmc[i].q_ref[1];
		wlr.side[i].P4 =  vmc[i].q_ref[4];
		
//		OUTPUT_LIMIT(wlr.side[i].T1, 2, -2);
//		OUTPUT_LIMIT(wlr.side[i].T4, 2, -2);
//		OUTPUT_LIMIT(wlr.side[i].Tw, 4.5f, -4.5f);
	}
}
