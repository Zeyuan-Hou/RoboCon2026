#include "algorithm.h"

/*-------------------------------------------------------------------------------------------------
函数功能：斜坡输入信号，用于摇杆和键盘分配速度
-------------------------------------------------------------------------------------------------*/
void ramp_signal(float* p_Output, float DesValue, float Step)
{
    u8 type = 0;

    if(*p_Output < DesValue)
            type = 0;
    else if(*p_Output > DesValue)
            type = 1;

    if(!type)
    {
            if(*p_Output >= DesValue)
            {
                    *p_Output = DesValue;
            }
            else
            {
                    *p_Output += Step;
                    if(*p_Output >= DesValue) 
                        *p_Output = DesValue;
            }
    }
    else
    {
            if(*p_Output <= DesValue)
            {
                    *p_Output = DesValue;
            }
            else
            {
                    *p_Output -= Step;
                    if(*p_Output <= DesValue) *p_Output = DesValue;
            }
    }
    *p_Output = (float)*p_Output;
}

/*-------------------------------------------------------------------------------------------------
函数功能：将一个字节中的第n位置为1
-------------------------------------------------------------------------------------------------*/
void set_bit1(u8 *data ,u8 n)
{
	*data |= (1 << n);
}

/*-------------------------------------------------------------------------------------------------
函数功能：将一个字节中的第n位置为0
-------------------------------------------------------------------------------------------------*/
void set_bit0(u8 *data ,u8 n)
{
	*data &= ~(1 << n);
}

/*-------------------------------------------------------------------------------------------------
函数功能：获取一个字节每位的01值
-------------------------------------------------------------------------------------------------*/
u8 get_bit(u8 data ,u8 n)
{
	return (data>>n)&1;
}

/*-------------------------------------------------------------------------------------------------
函数功能：一二阶LESO算法，通过状态观测器来观测扰动，在输出电流量中补偿掉
-------------------------------------------------------------------------------------------------*/
void LESO_Order1(ST_LESO_1order * leso_1order, float y,float U0)
{

  leso_1order->U0 = U0;

  leso_1order->E = y - leso_1order->Z1;
	
	leso_1order->Z1 += leso_1order->h*(leso_1order->b0 * leso_1order->U0+ leso_1order->Z2+leso_1order->Beta01 * leso_1order->E);

	leso_1order->Z2 += leso_1order->h*leso_1order->Beta02 * leso_1order->E;
	
	leso_1order->U = leso_1order->U0 - leso_1order->Z2/leso_1order->b0;

	if(leso_1order->U>=leso_1order->fpUMax) leso_1order->U = leso_1order->fpUMax;
	if(leso_1order->U<=-leso_1order->fpUMax) leso_1order->U = -leso_1order->fpUMax;
}

void LESO_Order2(ST_LESO_2order * leso_2order, float y,float U0)
{

  leso_2order->U0 = U0;

  leso_2order->E1hat = y - leso_2order->Z1;


  leso_2order->Z3 += leso_2order->h*(leso_2order->Beta03 * (leso_2order->E1hat));
	
	leso_2order->U = leso_2order->U0 - 1/leso_2order->b0 * leso_2order->Z3;

	leso_2order->Z2 += leso_2order->h*(leso_2order->Z3+leso_2order->Beta02 * (leso_2order->E1hat) + leso_2order->b0*leso_2order->U);
	leso_2order->Z1 += leso_2order->h*(leso_2order->Z2+leso_2order->Beta01 * (leso_2order->E1hat));

	if(leso_2order->U>=leso_2order->fpUMax) leso_2order->U = leso_2order->fpUMax;
	if(leso_2order->U<=-leso_2order->fpUMax) leso_2order->U = -leso_2order->fpUMax;
	
}
/*-------------------------------------------------------------------------------------------------
 * @brief: 以归一化时间为自变量的五次函数式位置插值函数
 * @param {float} start_pos
 * @param {float} end_pos
 * @param {float} time
 * @note: 不需要分段，直接算值就完了
 * @author: HITCRT
-------------------------------------------------------------------------------------------------*/
float plan_of_time(float start_pos, float end_pos, float time)
{
	float current_pos;
	float k =0;
	if(time >= 0.0f && time <= 1.0f) k = 6*time*time*time*time*time -15*time*time*time*time + 10*time*time*time;
	else if( time > 1.0f ) k = 1;
	current_pos = start_pos + k*(end_pos - start_pos);
	return current_pos;
}

///*-------------------------------------------------------------------------------------------------
//函数功能：S型多项式插值斜坡信号（五次多项式平滑过渡）
//参数说明：
//  p_Output  : 当前输出值指针
//  DesValue  : 目标值
//  Step      : 用于计算过渡时间的步长参数（影响过渡速度）
//  TimeStep  : 时间步长（每次调用的时间增量，单位秒）
//-------------------------------------------------------------------------------------------------*/
//float start_value = 0.0f;    // 过渡起始值
//float target_value = 0.0f;   // 当前目标值
//float elapsed_time = 0.0f;   // 已过渡时间
//float total_time = 0.0f;     // 总过渡时间
//void s_curve_ramp_signal(float* p_Output, float DesValue, float Step, float TimeStep)
//{
//    // 状态变量（静态存储）
////    static float start_value = 0.0f;    // 过渡起始值
////    static float target_value = 0.0f;   // 当前目标值
////    static float elapsed_time = 0.0f;   // 已过渡时间
////    static float total_time = 0.0f;     // 总过渡时间
//    
//    // 目标值变化时重置状态
//    if (fabs(target_value-DesValue)>10) {
//        start_value = *p_Output;
//        target_value = DesValue;
//        elapsed_time = 0.0f;
//        
//        // 计算总过渡时间：基于步长参数和差值
//        float value_diff = fabsf(target_value - start_value);
//        total_time = value_diff / Step;  // 时间 = 距离 / 速度
//        if (total_time < 0.001f) total_time = 0.001f; // 避免除零
//    }
//    
//    // 已到达目标值
//    if (elapsed_time >= total_time) {
//        *p_Output = target_value;
//        return;
//    }
//    
//    // 更新时间进度
//    elapsed_time += TimeStep;
//    if (elapsed_time > total_time) elapsed_time = total_time;
//    
//    // 计算归一化时间 [0, 1]
//    float t = elapsed_time / total_time;
//    
//    // 计算五次多项式系数 (6t^5 - 15t^4 + 10t^3)
//    float t3 = t * t * t;
//    float t4 = t3 * t;
//    float t5 = t4 * t;
//    float k = 6.0f * t5 - 15.0f * t4 + 10.0f * t3;
//    
//    // 应用插值
//    *p_Output = start_value + k * (target_value - start_value);
//}
/*-------------------------------------------------------------------------------------------------
函数功能：S型多项式斜坡信号（基于五次多项式）
参数说明：
  p_Output  : 当前输出值指针
  DesValue  : 目标值
  Step      : 基础步长（决定变化速率）
  Curve     : 曲线强度 (0.0-1.0)，0=线性，1=完全S曲线)
-------------------------------------------------------------------------------------------------*/
void s_curve_ramp_signal(float* p_Output, float DesValue, float Step, float Curve)
{
    static float last_output = 0.0f;
    static float last_target = 0.0f;
    static float progress = 0.0f;
    
    // 当目标值变化时重置状态
    if (last_target != DesValue) {
        last_output = *p_Output;
        last_target = DesValue;
        progress = 0.0f;
    }
    
    // 计算到目标的距离
    float distance = DesValue - last_output;
    float abs_distance = fabsf(distance);
    
    // 已经到达目标值
    if (abs_distance < 0.0001f) {
        *p_Output = DesValue;
        return;
    }
    
    // 计算当前进度比例 (0-1)
    float direction = (distance > 0) ? 1.0f : -1.0f;
    
    // 更新进度 - 基于步长和距离
    progress += Step / fmaxf(abs_distance, Step);
    progress = fminf(progress, 1.0f);
    
    // 应用S曲线插值
    float t = progress;
    float t3 = t * t * t;
    float t4 = t3 * t;
    float t5 = t4 * t;
    
    // 混合线性插值和S曲线
    float linear = t;
    float s_curve = 6.0f * t5 - 15.0f * t4 + 10.0f * t3;
    float k = linear * (1.0f - Curve) + s_curve * Curve;
    
    // 计算新输出值
    *p_Output = last_output + k * distance;
    
    // 确保精确到达目标值
    if (progress >= 1.0f) {
        *p_Output = DesValue;
    }
}

/*-------------------------------------------------------------------------------------------------
函数功能：龙伯格观测器
-------------------------------------------------------------------------------------------------*/
float L11 = 0.006f;
float L22 = 0.01f;
void Luenberger_observer_uniform_velocity_model( ST_Luenberger_observer* observer )
{
	//求先验状态向量
	observer->x_Matrix_head_prior[0] = observer->x_Matrix_head[0]+0.001f*observer->x_Matrix_head[1];
	observer->x_Matrix_head_prior[1] = observer->x_Matrix_head[1];
	//增益矩阵赋值
	L11=0.004f;
	
	observer->L_Matrix[0][0] = L11;
	observer->L_Matrix[1][1] = L22;
	//求状态估计
	observer->x_Matrix_head[0] = (1-observer->L_Matrix[0][0])*observer->x_Matrix_head_prior[0]+observer->L_Matrix[0][0]*observer->y_Matrix[0];
	observer->x_Matrix_head[1] = (1-observer->L_Matrix[1][1])*observer->x_Matrix_head_prior[1]+observer->L_Matrix[1][1]*observer->y_Matrix[1];
}


/*-------------------------------------------------------------------------------------------------
函数功能：卡尔曼更新
-------------------------------------------------------------------------------------------------*/
float KalmanUpdate(KalmanFilter* kf, float v_ins, float v_whl) 
{
    // 步骤1: 预测
    kf->P += kf->Q;  // 状态协方差更新

    // 步骤2: 计算打滑情况并调整轮速噪声
    float R_whl = kf->R_whl_base;
    if (fabsf(v_ins - v_whl) > kf->slip_thres) {
        R_whl *= kf->slip_scale;  // 打滑时增大轮速噪声
    }

    // 步骤3: 惯导速度更新 (优先)
    float K_ins = kf->P / (kf->P + kf->R_ins);
    kf->v += K_ins * (v_ins - kf->v);
    kf->P *= (1 - K_ins);

    // 步骤4: 轮速更新 (根据打滑情况动态加权)
    float K_whl = kf->P / (kf->P + R_whl);
    kf->v += K_whl * (v_whl - kf->v);
    kf->P *= (1 - K_whl);

    return kf->v;
}
/*-------------------------------------------------------------------------------------------------
函数功能：低通滤波器，滤除高频噪声
-------------------------------------------------------------------------------------------------*/
void LpFilter(ST_LPF *lpf)
{
    float fir_a = 1 / (1 + lpf->off_freq * lpf->samp_tim);
    lpf->out = fir_a * lpf->preout + (1 - fir_a) * lpf->in;
    lpf->preout = lpf->out;
}


/*-------------------------------------------------------------------------------------------------
函数功能：将一系列变量显示在VOFA里面
-------------------------------------------------------------------------------------------------*/
//void G_vofa_watch(void)
//{
//	float time;
//	time = (float)Current_Time;
//	memcpy(&vofa.fdata[0],&time,4);
//	//观察轮子电机是否能跟上以及打滑的情况
//	memcpy(&vofa.fdata[1],&chassis_run.leftup.fpDes,4);
//	memcpy(&vofa.fdata[2],&chassis_run.rightup.fpDes,4);
//	memcpy(&vofa.fdata[3],&chassis_run.rightdown.fpDes,4);
//	memcpy(&vofa.fdata[4],&chassis_run.leftdown.fpDes,4);
//	memcpy(&vofa.fdata[5],&chassis_run.leftup.fpFB,4);
//	memcpy(&vofa.fdata[6],&chassis_run.rightup.fpFB,4);
//	memcpy(&vofa.fdata[7],&chassis_run.rightdown.fpFB,4);
//	memcpy(&vofa.fdata[8],&chassis_run.leftdown.fpFB,4);
	//观察路径是否可以跟上
//		memcpy(&vofa.fdata[1],&nav.auto_path.pos_pid.x.fpDes,4);
//		memcpy(&vofa.fdata[2],&nav.auto_path.pos_pid.y.fpDes,4);
//		memcpy(&vofa.fdata[3],&nav.auto_path.pos_pid.w.fpDes,4);
//		memcpy(&vofa.fdata[4],&stRobot.stPos.fpPosX,4);
//		memcpy(&vofa.fdata[5],&stRobot.stPos.fpPosY,4);
//		memcpy(&vofa.fdata[6],&nav.auto_path.pos_pid.w.fpFB,4);
//		memcpy(&vofa.fdata[7],&Vision_Data.nav_pos_x,4);
//		memcpy(&vofa.fdata[8],&Vision_Data.nav_pos_y,4);
//		memcpy(&vofa.fdata[9],&Vision_Data.nav_v_x,4);
//		memcpy(&vofa.fdata[10],&Vision_Data.nav_v_y,4);
//}

/*********************************************************************************************
函数功能：接球机构J60电机的控制中心：
					0：NO_TORQUE_MODE 
						无力矩模式实际会产生部分阻尼效果，机械臂下落不会瞬间降下
						同时将进入控制模式初始化的标志位清0
					1：CONTROL_MODE
						位控模式。加入了TD的平滑效果
				无力矩模式切换到位控模式会主动进行期望位置复位，防止电机乱动
**********************************************************************************************/
//float J60_Kp = 300.f;
//float J60_Kd = 7.f;
//void J60_motor_control(void)
//{
//	static u8 flag_exit_ctrl_mode;
//	if(J60_Ctrl_Mode == NO_TORQUE_MODE)
//	{
//		J60_Motor_Ctrl.position_ = 0.f;
//		J60_Motor_Ctrl.velocity_ = 0.f;
//		J60_Motor_Ctrl.torque_ = 0.f;
//		J60_Motor_Ctrl.kp_ = 0.f;
//		J60_Motor_Ctrl.kd_ = 2.f;
//		
//		flag_exit_ctrl_mode = 0;
//	}
//	else 
//	{
//		if(flag_exit_ctrl_mode == 0)
//		{
//			J60_target_pos = J60_angle;//J60目标位置记录为当前位置
//			J60_td.x1 = J60_angle;
//			J60_td.x2 = 0;
//			flag_exit_ctrl_mode = 1;
//		}
//		J60_td.aim = ClipFloat(J60_target_pos,0,76);
//		CalTD(&J60_td);
//		J60_Motor_Ctrl.position_ = J60_td.x1*RADIAN;
//		J60_Motor_Ctrl.kp_ = J60_Kp;
//		J60_Motor_Ctrl.kd_ = J60_Kd;
//		J60_Motor_Ctrl.torque_= ClipFloat(5e-6*J60_angle*J60_angle*J60_angle*J60_angle
//																		-0.0005*J60_angle*J60_angle*J60_angle
//																		+0.013*J60_angle*J60_angle
//																		+0.0471*J60_angle-0.0079,0,4.9);
//	}
//}
//云台电机反作用力矩前馈
/**
 * @brief 根据两个关节电机力矩，计算云台yaw轴的前馈补偿电流
 * @param motor1 电机1状态
 * @param motor2 电机2状态
 * @param yaw_ctrl 云台控制器指针（会更新其 iq_ff 和 iq_cmd）
 */
void Yaw_Feedforward_Update(float torque_Nm1, float torque_Nm2, YawController_t *yaw_ctrl,GRAVITYPARAM *Gra_ff)
{
    // 1. 计算对yaw轴的总反作用力矩（Nm）
    float tau_disturb = yaw_ctrl->ff_gain_1 * (torque_Nm1-Gra_ff->Output_Tor[0]) +yaw_ctrl->ff_gain_2 * (torque_Nm2-Gra_ff->Output_Tor[1]);

    // 2. 转换为前馈补偿电流（A）
    //    云台电机需要输出相反的力矩来抵消扰动，因此取负号
	if(fabs(tau_disturb)>=1.5)
    yaw_ctrl->t_ff = -tau_disturb ;
	else
	yaw_ctrl->t_ff=0;	
}
//摩擦力前馈
float FrictionFeedforward(const FRICTIONPARAM* params, float velocity) 
{
    // 若速度绝对值小于阈值，可认为接近静止，摩擦力前馈设为0
    // （静摩擦通常由反馈控制器或单独的静摩擦补偿处理）
    if (fabsf(velocity) < params->velocity_threshold) {
        return 0.0f;
    }
    //当速度为正时，我们给出正的力矩来对抗负的摩擦力。   
    // 标准前馈（抵消摩擦力）：
    return params->Fc * (velocity > 0 ? 1.0f : -1.0f) + params->B * velocity;
}
float k=0.1f;

//取块机械臂重力前馈
void BlockArm_GravityFeedforward(GRAVITYPARAM *params)
{
    float Angle_t[3];//当前角度与水平方向的夹角，逆时针为正
    Angle_t[0]=params->angle0[0]+params->q[0];
    Angle_t[1]=params->angle0[1]+params->q[1];
    Angle_t[2]=params->q[2];//解算出各级机械臂当前角度与水平方向的夹角

    params->Output_Tor[2]=params->k_Joint3*cosf(Angle_t[2]*RADIAN);
    params->Output_Tor[1]=-(params->k_Joint2*cosf(Angle_t[1]*RADIAN)+params->Output_Tor[2]/16384*20*0.375f);
    params->Output_Tor[0]=params->k_Joint1*cosf(Angle_t[0]*RADIAN)+0.1f*params->Output_Tor[1];//解算出各关节电机的重力前馈扭矩
}


//取杆机械臂重力前馈
void PoleArm_GravityFeedforward(GRAVITYPARAM *params)
{
    float Angle_t[3];//当前角度与水平方向的夹角，逆时针为正
    Angle_t[0]=params->angle0[0]+params->q[0];
    Angle_t[1]=params->angle0[1]+params->q[0]+params->q[1];
    Angle_t[2]=params->angle0[2]+params->q[0]+params->q[1]+params->q[2];//解算出各级机械臂当前角度与水平方向的夹角

    params->Output_Tor[2]=params->k_Joint3*cosf(Angle_t[2]*RADIAN);
    params->Output_Tor[1]=-params->k_Joint2*cosf(Angle_t[1]*RADIAN)+params->Output_Tor[2];
    params->Output_Tor[0]=-params->k_Joint1*cosf(Angle_t[0]*RADIAN)+params->Output_Tor[1];//解算出各关节电机的重力前馈扭矩
}
