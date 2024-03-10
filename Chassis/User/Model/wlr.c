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
float mb = 4.4f, ml = 2.09f, mw = 0.715f;//机体质量 腿部质量 轮子质量 14.5
const float BodyWidth = 0.42f;//两轮间距
const float WheelRadius = 0.06f;//轮子半径
const float LegLengthMax = 0.30f, LegLengthMin = 0.15f;

const float LegLengthJump1 = 0.15f;//压腿
const float LegLengthJump2 = 0.35f;//蹬腿
const float LegLengthJump3 = 0.24f;//收腿
const float LegLengthJump4 = 0.22f;//落地
const float LegLengthFly = 0.20f;//腾空
const float LegLengthHigh = 0.30f;//长腿
const float LegLengthNormal = 0.15f;//正常
float LegCanChange = 0.15f;

float x3_balance_zero = 0.03, x5_balance_zero = -0.035f;//腿摆角角度偏置 机体俯仰角度偏置

//								位移  速度	角度	角速度  角度	角速度
float K_Array_Wheel[2][6] =		{{60, 30, 80, 8, 300, 10}, 
                                { -0, -0.7, -8, -1, 3, 2}};
float K_Array_Leg[2][6] =		{{6.08535, 12.6636, 49.1418, 7.57203, 54.8073, 12.7387}, {-0.793527, -1.75976, -13.1544, -1.78789, 4.8796, 1.66868}};
float K_Array_Fly[2][6] =		{{0, 0, 80, 10, 0, 0}, { 0, 0, 0, 0, 0, 0}};

float K_Array_test[2][6];
float K_Array_test2[2][6];
float K_Array_List[12][4] = 
{{9.93454,-4.78349,-49.5749,73.6911},{18.011,-17.0877,-55.0937,94.3588},{34.2832,183.616,-790.238,854.89},{7.46697,5.18589,-32.1947,34.5385},{55.9469,106.131,-54.0832,-87.9729},{10.7002,47.3682,-56.1761,11.2716},{-0.976265,-1.07592,-0.586662,2.68085},{-2.02614,0.0521471,-7.15447,10.5053},{-7.1408,-46.8898,66.44,-45.4585},{-1.61957,0.353914,-13.8965,13.0106},{7.72793,-9.97756,-13.7811,29.7827},{2.92789,-6.42717,5.36498,-0.454433}};
    
float K_Array_List_lq[12][4][4] = 
{{{5.22798,0.120698,-0.000840624,1.30043e-6},{-27.8965,0.476016,-0.00280186,0},{-63.4158,0.0760932,0,0},{116.423,0,0,0}},
{{8.71734,0.29992,-0.00203995,2.86743e-6},{-71.0269,0.76054,-0.00457909,0},{-15.2343,0.168384,0,0},{71.7744,0,0,0}},
{{-22.99,1.69686,-0.0119447,0.0000182525},{-99.7898,4.6908,-0.0268931,0},{-761.51,0.472008,0,0},{1096.43,0,0,0}},
{{5.83449,0.0891018,-0.000731637,1.77972e-6},{-48.1618,0.816036,-0.00471951,0},{31.5375,0.0972437,0,0},{-53.7863,0,0,0}},
{{75.0024,-1.07961,0.00679659,-6.20881e-6},{144.783,1.05686,-0.00505925,0},{-328.529,-0.3575,0,0},{217.495,0,0,0}},
{{15.5352,-0.311456,0.00198484,-2.0315e-6},{75.1284,0.202374,-0.000786395,0},{-187.928,-0.148803,0,0},{181.46,0,0,0}},
{{-1.33258,0.0159886,-0.000100859,9.22646e-8},{-1.41807,-0.0127862,0.0000605034,0},{1.05816,0.00470869,0,0},{2.29189,0,0,0}},
{{-2.72025,0.0238647,-0.000148877,1.18866e-7},{-0.375646,0.0154076,-0.0000903077,0},{-14.2395,0.00189886,0,0},{26.0355,0,0,0}},
{{-4.91795,-0.0760693,0.00086145,-3.06738e-6},{54.8722,-2.12031,0.0117915,0},{77.9843,-0.0406164,0,0},{-84.3636,0,0,0}},
{{-2.00836,0.00600775,-0.0000267199,-5.19685e-8},{-0.986647,0.118254,-0.000643844,0},{-33.8391,-0.00886079,0,0},{46.0011,0,0,0}},
{{4.86264,0.0699955,-0.00050198,8.93186e-7},{-33.212,0.353139,-0.00210746,0},{12.5199,0.0682708,0,0},{0.422155,0,0,0}},
{{2.93993,0.00177889,-0.0000390133,2.43053e-7},{-14.3874,0.104903,-0.000639501,0},{20.4728,0.026013,0,0},{-23.4521,0,0,0}}};
//float K_Array_List_lq[12][4][4] = 
//{{{4.7087,0.141274,-0.000962769,1.36206e-6},{-28.0204,0.40143,-0.0023947,0},{-56.6185,0.0784622,0,0},{112.515,0,0,0}},
//{{7.17705,0.307745,-0.00206142,2.70471e-6},{-66.2286,0.553269,-0.00341108,0},{5.8847,0.158318,0,0},{47.0187,0,0,0}},
//{{-21.8196,1.58107,-0.0110047,0.0000161347},{-80.3436,3.69808,-0.0213159,0},{-670.733,0.426098,0,0},{995.026,0,0,0}},
//{{5.00812,0.0995982,-0.000772778,1.65838e-6},{-42.9151,0.660686,-0.00385128,0},{28.6617,0.0922979,0,0},{-42.935,0,0,0}},
//{{94.0735,-1.02142,0.006405,-5.68319e-6},{133.635,1.17394,-0.00577817,0},{-331.72,-0.324255,0,0},{230.401,0,0,0}},
//{{21.1366,-0.298077,0.00189146,-1.87857e-6},{68.9534,0.252046,-0.00109136,0},{-181.404,-0.13502,0,0},{175.295,0,0,0}},
//{{-1.51513,0.0139221,-0.0000874833,7.77953e-8},{-1.21593,-0.0135415,0.0000663689,0},{1.35252,0.00392049,0,0},{1.44274,0,0,0}},
//{{-2.83479,0.0164206,-0.000101533,7.185e-8},{0.604203,0.0256778,-0.000140393,0},{-17.2328,-0.00125662,0,0},{28.7394,0,0,0}},
//{{-4.47738,-0.106048,0.00105777,-3.2855e-6},{54.0809,-2.10507,0.0117189,0},{85.2606,-0.0466696,0,0},{-97.6142,0,0,0}},
//{{-2.07197,0.00318767,-7.80634e-6,-7.71736e-8},{-0.841234,0.123583,-0.00066989,0},{-34.7815,-0.0105598,0,0},{46.9729,0,0,0}},
//{{4.88035,0.0919061,-0.000639694,1.01811e-6},{-35.9925,0.328582,-0.00198781,0},{17.9988,0.0754478,0,0},{-1.76811,0,0,0}},
//{{3.00566,0.00844127,-0.0000814119,2.85729e-7},{-15.6899,0.0973122,-0.000603877,0},{23.7715,0.0288295,0,0},{-26.2856,0,0,0}}};

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

void K_Array_Update(float K[2][6], float high_fdb)
{
    for(int i = 0; i < 2; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            K[i][j] = 0;
            for(int h = 0; h < 4; h++)
            {
                K[i][j] += K_Array_List[i * 6 + j][h] * powf(high_fdb, h);
            }
        }
    }
}

void K_Array_Updata_lq(float K[2][6], float high_fdb, float q0_fdb)
{
    float temp;
    for(int i = 0; i < 2; i++)
        for(int j = 0; j < 6; j++)
        {
            temp = 0;
            for(int x = 0; x <= 3; x++)
                for(int y = 0; y <= 3; y++)
                    temp += (K_Array_List_lq[i * 6 + j][x][y] * powf(high_fdb, x) * powf(q0_fdb, y));
            K[i][j] = temp;
        }
}

float lowpass_filter(float target0, float target1, float cofficient)
{
    return target0 * (1.0f - cofficient) + target1 * cofficient;
}

void WLR_Init(void)
{
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
		PID_Init(&pid_leg_length[i], 500, 0, 20000, 10, 20);//10 20
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
        if(ABS(wlr.v_set) > 1e-3f || ABS(wlr.wz_set) > 0.1f || ABS(vmc[0].q_fdb[0] - vmc[1].q_fdb[0]) > 0.01f)//有输入速度 或 两腿有差角时 将位移反馈置0  不发挥作用
			lqr[i].X_fdb[0] = 0;
		//支持力解算
		float L0_array[3] = {vmc[i].L_fdb, vmc[i].V_fdb.e.vy0_fdb, vmc[i].Acc_fdb.L0_ddot};
		float theta_array[3] = {lqr[i].X_fdb[2], lqr[i].X_fdb[3], lqr[i].dot_x4};
		wlr.side[i].Fn_fdb = WLR_Fn_Calc(wlr.az_fdb, vmc[i].F_fdb.e.Fy_fdb, vmc[i].F_fdb.e.T0_fdb, L0_array, theta_array);
		wlr.side[i].Fn_kal = Kalman1_Filter_Calc(&kal_fn[i], wlr.side[i].Fn_fdb);
		//离地检测
		if(wlr.side[i].Fn_kal < 17.0f)
			wlr.side[i].fly_cnt++;
		else if(wlr.side[i].fly_cnt != 0)
			wlr.side[i].fly_cnt--;
        
        if(wlr.side[i].fly_cnt > 30)
        {
            wlr.side[i].fly_cnt = 30;
            wlr.side[i].fly_flag = 1;
        }
        else if(wlr.side[i].fly_cnt == 0)
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
    {
		wlr.high_set = LegLengthFly;
    }
	else
    {
		if(wlr.high_mode)//大高度
            wlr.high_set = LegCanChange;
        else
            wlr.high_set = LegLengthNormal;
    }
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
        if(wlr.control_mode == 1)		//力控
        {
//            K_Array_Update(K_Array_test, vmc[0].L_fdb);
//            aMartix_Cover(lqr[0].K, (float*)K_Array_test, 2, 6);
//            K_Array_Update(K_Array_test, vmc[1].L_fdb);
//            aMartix_Cover(lqr[1].K, (float*)K_Array_test, 2, 6);

            K_Array_Updata_lq(K_Array_test, vmc[0].L_fdb, vmc[0].q_fdb[0]/PI*180);
            aMartix_Cover(lqr[0].K, (float*)K_Array_test, 2, 6);
            K_Array_Updata_lq(K_Array_test, vmc[1].L_fdb, vmc[1].q_fdb[0]/PI*180);
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
        //速度融合 在快要撞限位时减小速度控制
        if(wlr.side[i].q1 > 3.6f)
            lqr[i].X_ref[1] = lowpass_filter(twm.v_ref[i], lqr[i].X_fdb[1], (wlr.side[i].q1 - 3.6f)/0.3f);
        else if(wlr.side[i].q4 < -0.3f)
            lqr[i].X_ref[1] = lowpass_filter(twm.v_ref[i], lqr[i].X_fdb[1], (-0.3f - wlr.side[i].q4)/0.3f);
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
                                + 35 * vmc[i].L_fdb + 14 + WLR_SIGN(i) * wlr.roll_offs;
//								+ mb * GRAVITY / 2 + WLR_SIGN(i) * wlr.roll_offs;
//                                + WLR_SIGN(i) * wlr.roll_offs;
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
