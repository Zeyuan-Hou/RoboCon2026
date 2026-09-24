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
void G_vofa_watch(void)
{
	float time;
	time = (float)Current_Time;
	memcpy(&vofa.fdata[0],&time,4);
	//观察轮子电机是否能跟上以及打滑的情况
	memcpy(&vofa.fdata[1],&chassis_run.leftup.fpDes,4);
	memcpy(&vofa.fdata[2],&chassis_run.rightup.fpDes,4);
	memcpy(&vofa.fdata[3],&chassis_run.rightdown.fpDes,4);
	memcpy(&vofa.fdata[4],&chassis_run.leftdown.fpDes,4);
	memcpy(&vofa.fdata[5],&chassis_run.leftup.fpFB,4);
	memcpy(&vofa.fdata[6],&chassis_run.rightup.fpFB,4);
	memcpy(&vofa.fdata[7],&chassis_run.rightdown.fpFB,4);
	memcpy(&vofa.fdata[8],&chassis_run.leftdown.fpFB,4);
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
}

