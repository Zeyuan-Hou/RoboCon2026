#include "algorithm.h"

void LpFilter(ST_LPF *lpf)
{
    float fir_a = 1 / (1 + lpf->off_freq * lpf->samp_tim);
    lpf->out = fir_a * lpf->preout + (1 - fir_a) * lpf->in;
    lpf->preout = lpf->out;
}

void PID_Calc(ST_PID *pid, float reference, float feedback)
{
    pid->fpDes = reference;
    pid->fpFB = feedback;
    pid->fpE = pid->fpDes - pid->fpFB;
    if(fabs(pid->fpE) < pid->fpEMin){
        pid->fpE = 0;
    }
    pid->fpE = ClipFloat(pid->fpE, -pid->fpEMax, pid->fpEMax);

    pid->fpPreE = pid->fpE;
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

    if (fabs(y) > d0) a = pStTD->x2 + (a0 - d) * Sgn(y) / 2;
    else a = pStTD->x2 + y / pStTD->h;

    if (fabs(a) > d) fhan = -1 * pStTD->r * Sgn(a);
    else fhan = -1 * pStTD->r * a / d;

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
    do{
        if (fpAngA >= PI) fpAngA -= PI2;
        else if (fpAngA < -PI) fpAngA += PI2;
    } while (fpAngA >= PI || fpAngA < -PI);
    return fpAngA;
}

// 就相当于解出极坐标下的点在笛卡尔坐标系下的投影，解出笛卡尔坐标系下的点在极坐标系下的投影
// 这里只考虑出了平动下两个坐标系的转换，没有连上角速度，解不出旋转时的任何信息，因此在SpeedDistribute_Four_OmnidriectionalWhile函数中
// 平动可以直接用Handle_OmnidriectionalWhile函数处理，旋转时得先自行计算轮子所需速度再调用Handle_OmnidriectionalWhile函数
void Covert_coordinate(ST_VECTOR *a)
{
    if (a->type == POLAR){
        a->fpX = a->fpLength * cosf(a->fpThetha * RADIAN);
        a->fpY = a->fpLength * sinf(a->fpThetha * RADIAN);
    }else if (a->type == CARTESIAN){
        a->fpLength = Geometric_mean(a->fpX, a->fpY);
        if (fabs(a->fpY) < 1e-5 && fabs(a->fpX) < 1e-5){
            a->fpThetha = 0;
        }else a->fpThetha = atan2(a->fpY, a->fpX) / RADIAN;
    }
}

// 将全局坐标下的速度转化为局部坐标下的速度
void Concert_coorindnate(ST_VECTOR *global, ST_VECTOR *local, float fpQ)
{
    Covert_coordinate(global);
    Covert_coordinate(local);
    local->fpX = global->fpX * cosf(fpQ) + global->fpY * sinf(fpQ);
    local->fpY = -global->fpX * sinf(fpQ) + global->fpY * cosf(fpQ);
    local->fpLength = sqrt(pow(local->fpX, 2) + pow(local->fpX, 2));
    local->fpW = global->fpW;
}

float Geometric_mean(float a, float b)
{
    return sqrt(pow(a, 2) + pow(b, 2));
}

/*-------------------------------------------------------------------------------------------------
函数功能：卡尔曼更新
-------------------------------------------------------------------------------------------------*/
float KalmanUpdate(KalmanFilter *kf, float v_ins, float v_whl)
{
    // 步骤1: 预测
    kf->P += kf->Q; // 状态协方差更新

    // 步骤2: 计算打滑情况并调整轮速噪声
    float R_whl = kf->R_whl_base;
    if (fabsf(v_ins - v_whl) > kf->slip_thres)
    {
        R_whl *= kf->slip_scale; // 打滑时增大轮速噪声
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

/*******************************************************************
函数名称：ClipFloat()
函数功能：削波函数，去除超出最大值与最小值之间的值，代之以最大或最小值
输入：    fpValue:实际值
          fpMin:下限值
          fpMax:上限值
输出：    fpValue：削波后的值
备注：	  适用于浮点数变量的消波
********************************************************************/
float ClipFloat(float fpValue, float fpMin, float fpMax){
    if (fpValue < fpMin) return fpMin;
    else if (fpValue > fpMax) return fpMax;
    else return fpValue;
}

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
