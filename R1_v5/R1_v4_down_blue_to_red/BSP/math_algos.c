#include "math_algos.h"

float norm_angle(float q)
{
    if (!isfinite(q))
        return 0.0f;      // 1. 处理无穷大和NaN，防止后续逻辑失效
    q = fmodf(q, 360.0f); // 2. 一次性归一化到 (-360, 360)
    if (q > 180.0f)
        q -= 360.0f; // 3. 映射到 (-180, 180]
    if (q <= -180.0f)
        q += 360.0f;
    return q;
}

float wrap_to_2pi(float q)
{
    q = fmodf(q, PI2);
    if (q < 0)
        q += PI2;
    return q;
}

int8_t sgnf(float x)
{
    if (x > EPS)
        return 1;
    else if (x < -EPS)
        return -1;
    else
        return 0;
}

int u8_to_i32(unsigned char *data)//注意，所有stm32都是小端序
{
    int val;
    memcpy(&val, data, 4);
    return val;
}

// 正弦插值浮点查找表 (0 - 256), 对应权重 0.0f ~ 1.0f
const float SINE_INTERP_TABLE[257] = {
    0.000000f, 0.000061f, 0.000244f, 0.000549f, 0.000977f, 0.001526f, 0.002197f, 0.002991f, 0.003906f, 0.004944f,
    0.006104f, 0.007386f, 0.008789f, 0.010315f, 0.011963f, 0.013733f, 0.015625f, 0.017639f, 0.019776f, 0.022034f,
    0.024401f, 0.026901f, 0.029511f, 0.032243f, 0.035081f, 0.038041f, 0.041122f, 0.044312f, 0.047623f, 0.051053f,
    0.054581f, 0.058228f, 0.061982f, 0.065858f, 0.069825f, 0.073915f, 0.078099f, 0.082399f, 0.086791f, 0.091310f,
    0.095918f, 0.100633f, 0.105455f, 0.110373f, 0.115390f, 0.120499f, 0.125701f, 0.131000f, 0.136399f, 0.141890f,
    0.147470f, 0.153140f, 0.158892f, 0.164736f, 0.170666f, 0.176678f, 0.182767f, 0.188940f, 0.195193f, 0.201511f,
    0.207922f, 0.214402f, 0.220958f, 0.227591f, 0.234287f, 0.241047f, 0.247837f, 0.254719f, 0.261649f, 0.268652f,
    0.275700f, 0.282811f, 0.289967f, 0.297184f, 0.304448f, 0.311758f, 0.319114f, 0.326510f, 0.333965f, 0.341457f,
    0.348997f, 0.356578f, 0.364203f, 0.371862f, 0.379555f, 0.387291f, 0.395056f, 0.402854f, 0.410674f, 0.418524f,
    0.426391f, 0.434278f, 0.442183f, 0.450095f, 0.458024f, 0.465954f, 0.473899f, 0.481846f, 0.489799f, 0.497750f,
    0.505704f, 0.513651f, 0.521601f, 0.529545f, 0.537490f, 0.545417f, 0.553337f, 0.561245f, 0.569138f, 0.577014f,
    0.584873f, 0.592709f, 0.600531f, 0.608326f, 0.616098f, 0.623842f, 0.631558f, 0.639239f, 0.646890f, 0.654504f,
    0.662085f, 0.669624f, 0.677123f, 0.684581f, 0.691999f, 0.699368f, 0.706693f, 0.713968f, 0.721190f, 0.728360f,
    0.735474f, 0.742531f, 0.749529f, 0.756463f, 0.763334f, 0.771415f, 0.777085f, 0.783907f, 0.790664f, 0.797351f,
    0.803965f, 0.810505f, 0.816966f, 0.823348f, 0.829649f, 0.835866f, 0.841999f, 0.848043f, 0.854000f, 0.859864f,
    0.865636f, 0.871311f, 0.876891f, 0.882370f, 0.887750f, 0.893028f, 0.898205f, 0.903282f, 0.908254f, 0.913123f,
    0.918991f, 0.923539f, 0.928312f, 0.932984f, 0.937552f, 0.942017f, 0.946376f, 0.950626f, 0.954769f, 0.958797f,
    0.962711f, 0.966509f, 0.970188f, 0.973751f, 0.977196f, 0.980517f, 0.983713f, 0.986782f, 0.989719f, 0.992523f,
    0.995191f, 0.997721f, 0.999120f, 0.999451f, 0.999656f, 0.999939f, 1.000000f
};

float sin_interp_fast(float x_min, float x_max, float t) {
    float t_scaled = t * 256.0f;

    if(t_scaled <= 0) 
        return x_min;
    if(t_scaled >= 256) 
        return x_max;

    uint16_t t_index = (uint16_t)t_scaled;
    float t_frac = t_scaled - (float)t_index;

    float y0 = SINE_INTERP_TABLE[t_index];
    float y1 = SINE_INTERP_TABLE[t_index + 1];

    return (y0 + (y1 - y0) * t_frac) * (x_max - x_min) + x_min;
}

void CalTD(ST_TD *pStTD)
{
    float d, d0, y, a0, a = 0, fhan;   // 定义中间变量
    pStTD->x = pStTD->x1 - pStTD->aim; // 计算当前位置与目标位置的误差

    d = pStTD->r * pStTD->h;             // 计算调节因子 d
    d0 = pStTD->h * d;                   // 时间相关项 d0
    y = pStTD->x + pStTD->h * pStTD->x2; // 组合当前位置误差和速度误差

    a0 = sqrt(d * d + 8 * pStTD->r * fabs(y)); // 计算带有非线性特性的中间量 a0

    if (fabs(y) > d0)
        a = pStTD->x2 + (a0 - d) * sgnf(y) / 2; // 误差较大时，采用非线性调节
    else
        a = pStTD->x2 + y / pStTD->h; // 误差较小时，采用线性调节

    if (fabs(a) > d)
        fhan = -1 * pStTD->r * sgnf(a);
    else
        fhan = -1 * pStTD->r * a / d; // 控制速度更新的方向和幅度

    pStTD->x1 += pStTD->T * pStTD->x2; // 更新位置
    pStTD->x2 += pStTD->T * fhan;      // 更新速度
}

uint32_t iabs(int a)
{
    if (a < 0)
        return -a;
    return a;
}

float clipfloat(float x, float min, float max)
{
    if (x < min)
        return min;
    else if (x > max)
        return max;
    else
        return x;
}

void PID_Calc(ST_PID *pid, float fpDes, float fpFB)
{
    pid->fpDes = fpDes;
    pid->fpFB = fpFB;
    pid->fpE = pid->fpDes - pid->fpFB;
    if (fabsf(pid->fpE) <= pid->fpEMin)
        pid->fpE = 0;

    if (fabsf(pid->fpE) > pid->fpElimit)
        pid->fpSumE = 0; // 防止积分饱和
    else
        pid->fpSumE = clipfloat(pid->fpSumE + pid->fpE, -pid->fpSumEMax, pid->fpSumEMax);

    pid->fpUp = clipfloat(pid->fpKp * pid->fpE, -pid->fpUpMax, pid->fpUpMax);
    pid->fpUi = pid->fpKi * pid->fpSumE;
    pid->fpUd = clipfloat(pid->fpKd * (pid->fpE - pid->fpPreE), -pid->fpUdMax, pid->fpUdMax);
    pid->fpU = clipfloat(pid->fpUp + pid->fpUi + pid->fpUd, -pid->fpUMax, pid->fpUMax);

    pid->fpPreE = pid->fpE;
}

void PID_Calc_withoutDiff(ST_PID *pid, float fpDes, float fpFB, float v)
{
    pid->fpDes = fpDes;
    pid->fpFB = fpFB;
    pid->fpE = pid->fpDes - pid->fpFB;
    if (fabsf(pid->fpE) <= pid->fpEMin)
        pid->fpE = 0;

    if (fabsf(pid->fpE) > pid->fpElimit)
        pid->fpSumE = 0; // 防止积分饱和
    else
        pid->fpSumE = clipfloat(pid->fpSumE + pid->fpE, -pid->fpSumEMax, pid->fpSumEMax);

    pid->fpUp = clipfloat(pid->fpKp * pid->fpE, -pid->fpUpMax, pid->fpUpMax);
    pid->fpUi = pid->fpKi * pid->fpSumE;
    pid->fpUd = clipfloat(pid->fpKd * v, -pid->fpUdMax, pid->fpUdMax); 

    pid->fpU = clipfloat(pid->fpUp + pid->fpUi + pid->fpUd, -pid->fpUMax, pid->fpUMax);
}

void PID_Calc_withTD(ST_PID_withTD *pid, float fpDes, float fpFB){
    pid->fpDes = fpDes;
    pid->fpFB = fpFB;
    pid->fpE = pid->fpDes - pid->fpFB;
    if (fabsf(pid->fpE) <= pid->fpEMin)
        pid->fpE = 0;
    pid->td.aim = pid->fpE;
    CalTD(&pid->td);

    if (fabsf(pid->fpE) > pid->fpElimit)
        pid->fpSumE = 0; // 防止积分饱和
    else
        pid->fpSumE = clipfloat(pid->fpSumE + pid->fpE, -pid->fpSumEMax, pid->fpSumEMax);

    pid->fpUp = clipfloat(pid->fpKp * pid->fpE, -pid->fpUpMax, pid->fpUpMax);
    pid->fpUi = pid->fpKi * pid->fpSumE;
    pid->fpUd = clipfloat(pid->fpKd * pid->td.x2, -pid->fpUdMax, pid->fpUdMax);
    pid->fpU = clipfloat(pid->fpUp + pid->fpUi + pid->fpUd, -pid->fpUMax, pid->fpUMax);
}

void PID_Calc_Angle(ST_PID *pid, float fpDes, float fpFB)
{ // 算角度用的，角度限制在[-180,180]
    pid->fpDes = fpDes;
    pid->fpFB = fpFB;
    pid->fpE = norm_angle(pid->fpDes - pid->fpFB);
    if (fabsf(pid->fpE) <= pid->fpEMin)
        pid->fpE = 0;

    if (fabsf(pid->fpE) > pid->fpElimit)
        pid->fpSumE = 0; // 防止积分饱和
    else
        pid->fpSumE = clipfloat(pid->fpSumE + pid->fpE, -pid->fpSumEMax, pid->fpSumEMax);

    pid->fpUp = clipfloat(pid->fpKp * pid->fpE, -pid->fpUpMax, pid->fpUpMax);
    pid->fpUi = pid->fpKi * pid->fpSumE;
    pid->fpUd = clipfloat(pid->fpKd * (pid->fpE - pid->fpPreE), -pid->fpUdMax, pid->fpUdMax);
    pid->fpU = clipfloat(pid->fpUp + pid->fpUi + pid->fpUd, -pid->fpUMax, pid->fpUMax);

    pid->fpPreE = pid->fpE;
}

void PID_Calc_Angle_withoutDiff(ST_PID *pid, float fpDes, float fpFB, float v)
{
    pid->fpDes = fpDes;
    pid->fpFB = fpFB;
    pid->fpE = norm_angle(pid->fpDes - pid->fpFB);
    if (fabsf(pid->fpE) <= pid->fpEMin)
        pid->fpE = 0;

    if (fabsf(pid->fpE) > pid->fpElimit)
        pid->fpSumE = 0; // 防止积分饱和
    else
        pid->fpSumE = clipfloat(pid->fpSumE + pid->fpE, -pid->fpSumEMax, pid->fpSumEMax);

    pid->fpUp = clipfloat(pid->fpKp * pid->fpE, -pid->fpUpMax, pid->fpUpMax);
    pid->fpUi = pid->fpKi * pid->fpSumE;
    pid->fpUd = clipfloat(pid->fpKd * v, -pid->fpUdMax, pid->fpUdMax);

    pid->fpU = clipfloat(pid->fpUp + pid->fpUi + pid->fpUd, -pid->fpUMax, pid->fpUMax);

    pid->fpPreE = pid->fpE;
}

void PID_Calc_Angle_withTD(ST_PID_withTD *pid, float fpDes, float fpFB)
{    
    pid->fpDes = fpDes;
    pid->fpFB = fpFB;
    pid->fpE = fabsf(pid->fpDes - pid->fpFB);
    if (fabsf(pid->fpE) <= pid->fpEMin)
        pid->fpE = 0;
    pid->td.aim = pid->fpE;
    CalTD(&pid->td);

    if (fabsf(pid->fpE) > pid->fpElimit)
        pid->fpSumE = 0; // 防止积分饱和
    else
        pid->fpSumE = clipfloat(pid->fpSumE + pid->fpE, -pid->fpSumEMax, pid->fpSumEMax);

    pid->fpUp = clipfloat(pid->fpKp * pid->fpE, -pid->fpUpMax, pid->fpUpMax);
    pid->fpUi = pid->fpKi * pid->fpSumE;
    pid->fpUd = clipfloat(pid->fpKd * pid->td.x2, -pid->fpUdMax, pid->fpUdMax);
    pid->fpU = clipfloat(pid->fpUp + pid->fpUi + pid->fpUd, -pid->fpUMax, pid->fpUMax);
}

void PID_Cascade_Calc(ST_CASCADE_PID *pid, float outer_des, float outer_feedback, float inner_feedback)
{
    pid->outer_des = outer_des;
    pid->outer_fb = outer_feedback;
    pid->inner_fb = inner_feedback;
    PID_Calc(&pid->outer, pid->outer_des, pid->outer_fb);
    PID_Calc(&pid->inner, pid->outer.fpU, pid->inner_fb);
    pid->output = pid->inner.fpU;
}

void PID_Cascade_Calc_Angle(ST_CASCADE_PID *pid, float outer_des, float outer_feedback, float inner_feedback)
{ // 算角度用的，角度限制在[-180,180]
    pid->outer_des = outer_des;
    pid->outer_fb = outer_feedback;
    pid->inner_fb = inner_feedback;
    PID_Calc_Angle(&pid->outer, pid->outer_des, pid->outer_fb);
    PID_Calc(&pid->inner, pid->outer.fpU, pid->inner_fb);
    pid->output = pid->inner.fpU;
    pid->output = pid->inner.fpU;
}


void FeedForward_Calc(ST_FF *ff, float vel)
{
    ff->td.aim = vel;
    CalTD(&ff->td);
    ff->vel = vel;
    float u0 = 0;
    if (vel > ff->minvel)
        u0 = ff->k0;
    else if (vel < -ff->minvel)
        u0 = -ff->k0;
    ff->output = u0 + ff->k1 * ff->vel + ff->k2 * ff->td.x2;
}
void LpFilter(ST_LPF *lpf)
{
    float fir_a = 1 / (1 + lpf->off_freq * 2 * PI * lpf->samp_tim);
    lpf->out = fir_a * lpf->preout + (1 - fir_a) * lpf->in;
    lpf->preout = lpf->out;
}

// 初始化函数
// fs: 采样频率 (Hz)
// f0: 目标陷波频率 (Hz)
// Q: 品质因数 (Q值越大，陷波带宽越窄，对周围频率影响越小，但瞬态响应变慢)
void NotchFilter_Init(NotchFilter* filter, float fs, float f0, float Q) {
    // 计算角频率
    float omega = 2.0f * PI * f0 / fs;
    float sn = sin(omega);
    float cs = cos(omega);
    float alpha = sn / (2.0f * Q);

    // 计算标准双二阶系数
    float a0 = 1.0f + alpha;
    filter->b0 = 1.0f / a0;
    filter->b1 = (-2.0f * cs) / a0;
    filter->b2 = 1.0f / a0;
    filter->a1 = (-2.0f * cs) / a0;
    filter->a2 = (1.0f - alpha) / a0;

    // 清空历史状态
    filter->x1 = 0.0f;
    filter->x2 = 0.0f;
    filter->y1 = 0.0f;
    filter->y2 = 0.0f;
}

float NotchFilter_Update(NotchFilter* filter, float input) {
    // 差分方程: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    float output = filter->b0 * input 
                  + filter->b1 * filter->x1 
                  + filter->b2 * filter->x2 
                  - filter->a1 * filter->y1 
                  - filter->a2 * filter->y2;

    // 更新状态缓冲区
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;

    return output;
}



