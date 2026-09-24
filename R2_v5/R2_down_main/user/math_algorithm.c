#include "math_algorithm.h"
/*******************************************************************
函数名称：ConvertAngle()
函数功能：将角度转换为全局坐标系的航向角范围[-PI,PI)
输入：    ang：目标角度(RADIAN)
输出：    转换后的角度(RADIAN)
备注：    逆时针为正，顺时针为负，不适合对角度值较大的值做转换
********************************************************************/
fp32 ConvertAngle(fp32 fpAngA)
{
    do
    {
        if (fpAngA >= PI)
        {
            fpAngA -= PI2;
        }
        else if (fpAngA < -PI)
        {
            fpAngA += PI2;
        }
    } while (fpAngA >= PI || fpAngA < -PI);
    return fpAngA;
}


/*******************************************************************
函数名称：ClipFloat()
函数功能：削波函数，去除超出最大值与最小值之间的值，代之以最大或最小值
输入：    fpValue:实际值
		  fpMin:下限值
		  fpMax:上限值
输出：    fpValue：削波后的值
备注：	  适用于浮点数变量的消波
********************************************************************/
fp32 ClipFloat(fp32 fpValue, fp32 fpMin, fp32 fpMax)
{
    if(fpValue < fpMin)
    {
        return fpMin;
    }
    else if(fpValue > fpMax)
    {
        return fpMax;
    }
    else
    {
        return fpValue;
    }
}

fp32 Geometric_mean(fp32 a,fp32 b)
{
	return sqrt(pow(a, 2) + pow(b, 2));
}

/*******************************************************************************************
函数名称：LpFilter()
函数功能：一阶低通滤波器，用于平滑输入信号，去除高频噪声
输入：   1. lpf 指向低通滤波器结构体的指针，结构体成员包括：
             - off_freq：截止频率
             - samp_tim：采样时间
             - in：当前输入信号
             - preout：上一次输出信号
*******************************************************************************************/
void LpFilter(ST_LPF *lpf)
{
    // 计算滤波器系数 fir_a，基于截止频率和采样时间的典型公式：
    // fir_a = 1 / (1 + 截止频率 * 采样时间)
    float fir_a = 1 / (1 + lpf->off_freq * lpf->samp_tim);

    // 根据当前输入信号和上一次的输出信号计算新的输出值：
    // 输出是当前输入信号和历史输出信号的加权平均，权重由 fir_a 决定
    lpf->out = fir_a * lpf->preout + (1 - fir_a) * lpf->in;

    // 更新历史输出值，为下次调用提供参考
    lpf->preout = lpf->out;
}


// 自适应LPF更新函数  
float AdaptiveLPF_Update(AdaptiveLPF *f, float feedback, float target)
{
    float error = fabs(feedback - target);
    
    // 根据误差大小动态调整滤波强度
    if(error > f->error_threshold) {
        f->alpha_dynamic = 0.2f;  // 大误差时弱滤波，快速响应
    } else {
        f->alpha_dynamic = 0.7f;  // 小误差时强滤波，平滑运行
    }
    
    // 一阶滤波
    float y = f->alpha_dynamic * f->y_prev + (1 - f->alpha_dynamic) * feedback;
    f->y_prev = y;
    return y;
}


/*-------------------------------------------------------------------------------------------------
函数功能：斜坡输入信号
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

