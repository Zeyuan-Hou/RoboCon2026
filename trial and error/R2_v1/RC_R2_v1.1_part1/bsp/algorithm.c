#include "algorithm.h"

void LpFilter(ST_LPF *lpf)
{
    float fir_a = 1 / (1 + lpf->off_freq * lpf->samp_tim);
    lpf->out = fir_a * lpf->preout + (1 - fir_a) * lpf->in;
    lpf->preout = lpf->out;
}

void PID_Calc(ST_PID *pid, float reference, float feedback)
{
    pid->fpPreE = pid->fpE;
    pid->fpDes = reference;
    pid->fpFB = feedback;
    pid->fpE = pid->fpDes - pid->fpFB;
    if (fabs(pid->fpE) < pid->fpEMin)
    {
        pid->fpE = 0;
    }
    pid->fpE = ClipFloat(pid->fpE, -pid->fpEMax, pid->fpEMax);

    pid->fpSumE += pid->fpE;
    pid->fpSumE = ClipFloat(pid->fpSumE, -pid->fpSumEMax, pid->fpSumEMax);

    pid->fpUKp = pid->fpE * pid->fpKp;
    pid->fpUKp = ClipFloat(pid->fpUKp, -pid->fpUpMax, pid->fpUpMax);

    pid->fpUKd = (pid->fpPreE - pid->fpE) * pid->fpKd;
    pid->fpUKd = ClipFloat(pid->fpUKd, -pid->fpUdMax, pid->fpUdMax);

    pid->fpUKi = pid->fpSumE * pid->fpKi;

    pid->fpU = pid->fpUKp + pid->fpUKd + pid->fpUKi;
    pid->fpU = ClipFloat(pid->fpU, -pid->fpUMax, pid->fpUMax);
}

//位置环
void PID_Calc_Pos(ST_PID *pStPID)
{
	
	pStPID->fpPreE = pStPID->fpE;
	
    pStPID->fpE = pStPID->fpDes - pStPID->fpFB; // 计算当前误差
	// 误差限幅
    pStPID->fpE = ClipFloat(pStPID->fpE, -pStPID->fpEMax, pStPID->fpEMax);

    // 误差限幅
	
    // 计算P项输出，并且限幅在[-fpUpMax,fpUpMax]里
    pStPID->fpUKp = ClipFloat(pStPID->fpKp * pStPID->fpE, -pStPID->fpUpMax, pStPID->fpUpMax);

	//计算i项

		pStPID->fpUKi += pStPID->fpKi * pStPID->fpE;
		pStPID->fpUKi = ClipFloat(pStPID->fpUKi, -pStPID->fpUiMax, pStPID->fpUiMax);
  	if(fabs(pStPID->fpE)> pStPID->fpElimit) pStPID->fpUKi =0;//积分分离
	
    // 计算D项输出
    pStPID->fpUKd = ClipFloat(pStPID->fpKd * (pStPID->fpE - pStPID->fpPreE), -pStPID->fpUdMax, pStPID->fpUdMax);

	//死区，消除微小抖动
    if (fabs(pStPID->fpE) <= pStPID->fpEMin)
    {       
        pStPID->fpUKp = 0;
    }
	if(fabs(pStPID->fpE)< pStPID->fpEforID)
	{
		pStPID->fpUKi = 0;
		pStPID->fpUKd = 0;
		
	}
    // 计算总输出
    pStPID->fpU = pStPID->fpUKp + pStPID->fpUKi + pStPID->fpUKd; 

    // PID运算总限幅
    pStPID->fpU = ClipFloat(pStPID->fpU, -pStPID->fpUMax, pStPID->fpUMax);
}
//处理+ -180跃变
float Cal_Angle_Error(float current, float target) 
{
    float error = target - current;
    // 处理角度跃变，确保误差在[-180, 180)范围内
    if (error >= 180.0f) 
	{
        error -= 360.0f;
    } 
	else if (error < -180.0f) 
	{
        error += 360.0f;
    }
    return error;
}
//位置环中的角度环
void PID_Calc_Angle(ST_PID *pStPID)
{
	
	pStPID->fpPreE = pStPID->fpE;
	
    pStPID->fpE = Cal_Angle_Error(pStPID->fpFB,pStPID->fpDes);
	// 误差限幅
    pStPID->fpE = ClipFloat(pStPID->fpE, -pStPID->fpEMax, pStPID->fpEMax);

    // 误差限幅
	
    // 计算P项输出，并且限幅在[-fpUpMax,fpUpMax]里
    pStPID->fpUKp = ClipFloat(pStPID->fpKp * pStPID->fpE, -pStPID->fpUpMax, pStPID->fpUpMax);

	//计算i项

		pStPID->fpUKi += pStPID->fpKi * pStPID->fpE;
		pStPID->fpUKi = ClipFloat(pStPID->fpUKi, -pStPID->fpUiMax, pStPID->fpUiMax);
	if(fabs(pStPID->fpE)> pStPID->fpElimit) pStPID->fpUKi =0;//积分分离
	
    // 计算D项输出
    pStPID->fpUKd = ClipFloat(pStPID->fpKd * (pStPID->fpE - pStPID->fpPreE), -pStPID->fpUdMax, pStPID->fpUdMax);

	//死区，消除微小抖动
    if (fabs(pStPID->fpE) <= pStPID->fpEMin)
    {       
        pStPID->fpUKp = 0;
    }
	if(fabs(pStPID->fpE)< pStPID->fpEforID)
	{
		pStPID->fpUKi = 0;
		pStPID->fpUKd = 0;
		
	}
    // 计算总输出
    pStPID->fpU = pStPID->fpUKp + pStPID->fpUKi + pStPID->fpUKd; 

    // PID运算总限幅
    pStPID->fpU = ClipFloat(pStPID->fpU, -pStPID->fpUMax, pStPID->fpUMax);
}
void Cascade_PID_Calc(ST_CASCADE_PID *cascade_pid, float outer_reference, float outer_feedback)
{
    PID_Calc(&cascade_pid->outer, outer_reference, outer_feedback);
    PID_Calc(&cascade_pid->inner, cascade_pid->outer.fpU, cascade_pid->inner.fpFB);
    cascade_pid->final_fpU = cascade_pid->inner.fpU;
}

void Cascade_PID_TD_Calc(ST_CASCADE_PID *cascade_pid, float outer_reference, float outer_feedback)
{
    cascade_pid->td.aim = outer_reference;
    CalTD(&(cascade_pid->td));
    PID_Calc(&cascade_pid->outer, cascade_pid->td.x1, outer_feedback);
    PID_Calc(&cascade_pid->inner, cascade_pid->outer.fpU * 0.7f + cascade_pid->td.x2 * 0.3f, cascade_pid->inner.fpFB);
    cascade_pid->final_fpU = cascade_pid->inner.fpU;
}

float Sgn(float x)
{
    return (x > 0) - (x < 0);
}

void CalTD(ST_TD *pStTD)
{
    float d, d0, y, a0, a = 0, fhan;
    pStTD->x = pStTD->x1 - pStTD->aim;
    d = pStTD->r * pStTD->h;
    d0 = pStTD->h * d;
    y = pStTD->x + pStTD->h * pStTD->x2;
    a0 = sqrt(d * d + 8 * pStTD->r * fabs(y));

    if (fabs(y) > d0)
        a = pStTD->x2 + (a0 - d) * Sgn(y) / 2;
    else
        a = pStTD->x2 + y / pStTD->h;

    if (fabs(a) > d)
        fhan = -1 * pStTD->r * Sgn(a);
    else
        fhan = -1 * pStTD->r * a / d;

    pStTD->x1 += pStTD->T * pStTD->x2;
    pStTD->x2 += pStTD->T * fhan;
}

/*******************************************************************
函数名称：ConvertAngle()
函数功能：将角度转换为全局坐标系的航向角范围[-PI,PI)
输入：    ang：目标角度(RADIAN)
输出：    转换后的角度(RADIAN)
备注：    逆时针为正，顺时针为负，不适合对角度值较大的值做转换
********************************************************************/
float ConvertAngle(float fpAngA)
{
    do
    {
        if (fpAngA >= PI)
            fpAngA -= PI2;
        else if (fpAngA < -PI)
            fpAngA += PI2;
    } while (fpAngA >= PI || fpAngA < -PI);
    return fpAngA;
}

float ConvertDegree(float deg){
	while(deg > 180) deg-=360.f;
	while(deg <= 180) deg +=360.f;
	return deg;
}

void Covert_coordinate(ST_VECTOR *a)
{
    if (a->type == POLAR)
    {
        a->fpX = a->fpLength * cosf(a->fpThetha * RADIAN);
        a->fpY = a->fpLength * sinf(a->fpThetha * RADIAN);
    }
    else if (a->type == CARTESIAN)
    {
        a->fpLength = Geometric_mean(a->fpX, a->fpY);
        if (fabs(a->fpY) < 1e-5 && fabs(a->fpX) < 1e-5)
        {
            a->fpThetha = 0;
        }
        else
            a->fpThetha = atan2(a->fpY, a->fpX) / RADIAN;
    }
}

// 将全局坐标下的速度转化为局部坐标下的速度
void Convert_velt(ST_VECTOR *global, ST_VECTOR *local, float fpQ)
{
    Covert_coordinate(global);
    Covert_coordinate(local);
    local->fpX = global->fpX * cosf(fpQ+PI/2) + global->fpY * sinf(fpQ+PI/2);
    local->fpY = -global->fpX * sinf(fpQ+PI/2) + global->fpY * cosf(fpQ+PI/2);
    local->fpLength = sqrt(pow(local->fpX, 2) + pow(local->fpY, 2));
    local->fpW = global->fpW;
}

float Geometric_mean(float a, float b)
{
    return sqrt(pow(a, 2) + pow(b, 2));
}

// float normalize_angle(float angle_deg)
// {
//     angle_deg = fmodf(angle_deg, 360.0f);
//     if (angle_deg < 0)
//     {
//         angle_deg += 360.0f;
//     }
//     // 处理接近360度的情况，将其转换为0度
//     if (fabsf(angle_deg - 360.0f) < 2.0f)
//     {
//         angle_deg = 0.0f;
//     }
//     // 映射到[-180, 180)
//     if (angle_deg >= 180.0f)
//     {
//         angle_deg -= 360.0f;
//     }
//     // 处理接近-180度的情况，将其转换为180度
//     if (fabsf(angle_deg + 180.0f) < 2.0f)
//     {
//         angle_deg = 180.0f;
//     }
//     return angle_deg;
// }

void ramp_signal(float *p_Output, float DesValue, float Step)
{
    uint8_t type = 0;

    if (*p_Output < DesValue) type = 0;
    else if (*p_Output > DesValue) type = 1;

    if (!type){
        if (*p_Output >= DesValue) *p_Output = DesValue;
        else
        {
            *p_Output += Step;
            if (*p_Output >= DesValue)
                *p_Output = DesValue;
        }
    }else{
        if (*p_Output <= DesValue) *p_Output = DesValue;
        else
        {
            *p_Output -= Step;
            if (*p_Output <= DesValue)
                *p_Output = DesValue;
        }
    }
    *p_Output = (float)*p_Output;
}

float ClipFloat(float fpValue, float fpMin, float fpMax)
{
    if (fpValue < fpMin)
        return fpMin;
    else if (fpValue > fpMax)
        return fpMax;
    else
        return fpValue;
}

float uint_to_float(const int x_int, const float x_min, const float x_max, const int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

int float_to_uint(const float x, const float x_min, const float x_max, const int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float feedforward_curve_sin(float goal_pos, float current_pos, int interval_ms, int time_ms)
{
    // 构造aim_pos关于时间的函数，将[current_pos, goal_pos]区间依照[-π/2, π/2]区间的sin函数曲线映射到[t0, t0+interval_ms]区间
    return current_pos + (goal_pos - current_pos) / 2.f * sinf((float)time_ms / interval_ms * PI - PI / 2.f) + (goal_pos - current_pos) / 2.f;
}

float feedforward_linear(float goal_pos, float current_pos, int interval_ms, int time_ms)
{
    return current_pos + (goal_pos - current_pos) * ((float)time_ms / interval_ms);
}

float feedforward_G_tor(float tor_k, float angle)
{
    return tor_k * cosf(angle * PI / 180.0f);
}
