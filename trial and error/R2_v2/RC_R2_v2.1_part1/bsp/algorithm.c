#include "algorithm.h"

void LpFilter(ST_LPF *lpf)
{
    float fir_a = 1 / (1 + lpf->off_freq * 2 * PI * lpf->samp_tim);
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

    pid->fpUKd = (pid->fpE - pid->fpPreE) * pid->fpKd;
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
	if(fabs(pStPID->fpE)< pStPID->fpElimit)//积分分离
	{
		pStPID->fpUKi += pStPID->fpKi * pStPID->fpE;
		pStPID->fpUKi = ClipFloat(pStPID->fpUKi, -pStPID->fpUiMax, pStPID->fpUiMax);
	}
	
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
void PID_Calc_Angle(ST_PID *pid, float reference, float feedback)
{
	pid->fpPreE = pid->fpE;
    pid->fpDes = reference;
    pid->fpFB = feedback;
    pid->fpE = Cal_Angle_Error(pid->fpFB, pid->fpDes);

    if (fabs(pid->fpE) < pid->fpEMin)
    {
        pid->fpE = 0;
    }
    pid->fpE = ClipFloat(pid->fpE, -pid->fpEMax, pid->fpEMax);

    pid->fpSumE += pid->fpE;
    pid->fpSumE = ClipFloat(pid->fpSumE, -pid->fpSumEMax, pid->fpSumEMax);

    pid->fpUKp = pid->fpE * pid->fpKp;
    pid->fpUKp = ClipFloat(pid->fpUKp, -pid->fpUpMax, pid->fpUpMax);

    pid->fpUKd = (pid->fpE - pid->fpPreE) * pid->fpKd;
    pid->fpUKd = ClipFloat(pid->fpUKd, -pid->fpUdMax, pid->fpUdMax);

    pid->fpUKi = pid->fpSumE * pid->fpKi;

    pid->fpU = pid->fpUKp + pid->fpUKd + pid->fpUKi;
    pid->fpU = ClipFloat(pid->fpU, -pid->fpUMax, pid->fpUMax);
}

// PI前馈控制，适用于速度控制
void PI_Feedforward_Calc(ST_PI_Feedforward *pi_ff, float reference, float feedback)
{
    pi_ff->last_target = pi_ff->fpDes;
    pi_ff->fpDes = reference;
    pi_ff->fpFB = feedback;
    pi_ff->fpE = pi_ff->fpDes - pi_ff->fpFB;
    pi_ff->fpE = ClipFloat(pi_ff->fpE, -pi_ff->fpEMax, pi_ff->fpEMax);

    // 前馈项计算
    // 针对速度及加速度的前馈控制，分别使用当前目标值和目标值变化率进行计算
    pi_ff->fpUff = pi_ff->fpKffv * pi_ff->fpDes + pi_ff->fpKffa * (pi_ff->fpDes - pi_ff->last_target);
    pi_ff->fpUff = ClipFloat(pi_ff->fpUff, -pi_ff->fpUffMax, pi_ff->fpUffMax);

    // P项计算
    pi_ff->fpUKp = ClipFloat(pi_ff->fpKp * pi_ff->fpE, -pi_ff->fpUpMax, pi_ff->fpUpMax);

    // I项计算
    pi_ff->fpUKi += pi_ff->fpKi * pi_ff->fpE;
    pi_ff->fpUKi = ClipFloat(pi_ff->fpUKi, -pi_ff->fpUiMax, pi_ff->fpUiMax);

    // 计算总输出
    pi_ff->fpU = pi_ff->fpUff + pi_ff->fpUKp + pi_ff->fpUKi;

    // PID运算总限幅
    pi_ff->fpU = ClipFloat(pi_ff->fpU, -pi_ff->fpUMax, pi_ff->fpUMax);
}

void PID_Fuzzy_Calc(ST_PID_Fuzzy *pStPID, float reference, float feedback)
{
    pStPID->fpPreE = pStPID->fpE;
    pStPID->fpDes = reference;
    pStPID->fpFB = feedback;
    pStPID->fpE = pStPID->fpDes - pStPID->fpFB;

    // 根据当前误差计算模糊隶属度，并动态调整PID参数
    // 模糊隶属度membership_degree根据误差绝对值在const_near和const_far之间线性变化，范围为[0,1]
    // 例如，fpKp的范围是[1*kp_init, (1+kp_boost)*kp_init]
    pStPID->membership_degree = ClipFloat((fabsf(pStPID->fpE) - pStPID->const_near) / (pStPID->const_far - pStPID->const_near), 0.0f, 1.0f);
    pStPID->fpKp = pStPID->kp_init * (1.0f + pStPID->kp_boost * pStPID->membership_degree);
    pStPID->fpUpMax = pStPID->upmax_init * (1.0f + pStPID->upmax_boost * pStPID->membership_degree);

    // 利用原始误差更新pid参数后再处理误差
    pStPID->fpE = ClipFloat(pStPID->fpE, -pStPID->fpEMax, pStPID->fpEMax);

    // 计算P、I、D项输出，并且分别限幅
    pStPID->fpUKp = pStPID->fpE * pStPID->fpKp;
    pStPID->fpUKp = ClipFloat(pStPID->fpUKp, -pStPID->fpUpMax, pStPID->fpUpMax);

    if (fabs(pStPID->fpE) > pStPID->fpEMin)
    {
        pStPID->fpSumE = 0;
    }
    else
    {
        pStPID->fpSumE += pStPID->fpE;
    }
    pStPID->fpUKi = pStPID->fpSumE * pStPID->fpKi;
    pStPID->fpUKi = ClipFloat(pStPID->fpUKi, -pStPID->fpUiMax, pStPID->fpUiMax);

    pStPID->fpUKd = (pStPID->fpE - pStPID->fpPreE) * pStPID->fpKd;
    pStPID->fpUKd = ClipFloat(pStPID->fpUKd, -pStPID->fpUdMax, pStPID->fpUdMax);

    pStPID->fpU = pStPID->fpUKp + pStPID->fpUKd + pStPID->fpUKi;
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
void Convert_Velt_Global2Local(ST_VECTOR global, ST_VECTOR *local, float fpQ)
{
    local->fpX = -global.fpX * cosf(fpQ+PI/2) + global.fpY * sinf(fpQ+PI/2);
    local->fpY = -global.fpX * sinf(fpQ+PI/2) - global.fpY * cosf(fpQ+PI/2);
    local->fpLength = sqrt(pow(local->fpX, 2) + pow(local->fpY, 2));
    local->fpW = global.fpW;
}

void Convert_Velt_Local2Global(ST_VECTOR *global, ST_VECTOR local, float fpQ)
{
    global->fpX = local.fpX * cosf(fpQ+PI/2) - local.fpY * sinf(fpQ+PI/2);
    global->fpY = local.fpX * sinf(fpQ+PI/2) + local.fpY * cosf(fpQ+PI/2);
    global->fpLength = sqrt(pow(global->fpX, 2) + pow(global->fpY, 2));
    global->fpW = local.fpW;
}

float Geometric_mean(float a, float b)
{
    return sqrt(pow(a, 2) + pow(b, 2));
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

void Cubic_Curve_Set(Cube_Line* cube, float p0, float v0, float p1, float v1, int32_t tim_total)
{
    const float v0_ms = v0 / 1000.f;
    const float v1_ms = v1 / 1000.f;
    const float t = (float)tim_total;
    cube->k0 = p0;
    cube->k1 = v0_ms;
    cube->k2 = (3.f * (p1 - p0) - (2.f * v0_ms + v1_ms) * t) / (t * t);
    cube->k3 = (-2.f * (p1 - p0) + (v0_ms + v1_ms) * t) / (t * t * t);
}

/**
 * @brief 五次多项式轨迹参数设置
 *
 * 轨迹形式：
 * p(t) = k5*t^5 + k4*t^4 + k3*t^3 + k2*t^2 + k1*t + k0
 *
 * 其中 t 的单位为 ms。
 *
 * @param quintic    五次多项式结构体指针
 * @param p0         起点位置
 * @param v0         起点速度，单位：位置单位/s
 * @param a0         起点加速度，单位：位置单位/s^2
 * @param p1         终点位置
 * @param v1         终点速度，单位：位置单位/s
 * @param a1         终点加速度，单位：位置单位/s^2
 * @param tim_total  总时间，单位：ms
 */
void Quintic_Curve_Set(Quintic_Line *quintic,
                       float p0, float v0, float a0,
                       float p1, float v1, float a1,
                       int32_t tim_total)
{
    if (quintic == 0)
        return;

    if (tim_total <= 0)
    {
        quintic->k0 = p1;
        quintic->k1 = 0.0f;
        quintic->k2 = 0.0f;
        quintic->k3 = 0.0f;
        quintic->k4 = 0.0f;
        quintic->k5 = 0.0f;
        return;
    }

    const float T = (float)tim_total;

    // 速度从 位置单位/s 转为 位置单位/ms
    const float v0_ms = v0 / 1000.0f;
    const float v1_ms = v1 / 1000.0f;

    // 加速度从 位置单位/s^2 转为 位置单位/ms^2
    const float a0_ms2 = a0 / 1000000.0f;
    const float a1_ms2 = a1 / 1000000.0f;

    const float T2 = T * T;
    const float T3 = T2 * T;
    const float T4 = T3 * T;
    const float T5 = T4 * T;

    /*
     * 起点边界：
     * p(0)  = k0 = p0
     * p'(0) = k1 = v0_ms
     * p''(0)= 2*k2 = a0_ms2
     */
    quintic->k0 = p0;
    quintic->k1 = v0_ms;
    quintic->k2 = 0.5f * a0_ms2;

    /*
     * 为了简化计算，先扣除 k0, k1, k2 对终点的影响
     */
    const float dp = p1 - (quintic->k0 + quintic->k1 * T + quintic->k2 * T2);
    const float dv = v1_ms - (quintic->k1 + 2.0f * quintic->k2 * T);
    const float da = a1_ms2 - (2.0f * quintic->k2);

    /*
     * 求解 k3, k4, k5：
     *
     * k3*T^3 + k4*T^4 + k5*T^5 = dp
     * 3*k3*T^2 + 4*k4*T^3 + 5*k5*T^4 = dv
     * 6*k3*T + 12*k4*T^2 + 20*k5*T^3 = da
     */
    quintic->k3 = (20.0f * dp - 8.0f * T * dv + T2 * da) / (2.0f * T3);
    quintic->k4 = (-15.0f * dp + 7.0f * T * dv - T2 * da) / T4;
    quintic->k5 = (12.0f * dp - 6.0f * T * dv + T2 * da) / (2.0f * T5);
}

/**
 * @brief 五次多项式轨迹计算
 *
 * @param aim_p      输出目标位置
 * @param aim_v      输出目标速度，单位：位置单位/s
 * @param aim_a      输出目标加速度，单位：位置单位/s^2
 * @param quintic    五次多项式结构体
 * @param tim        当前时间，单位：ms
 * @param tim_total  总时间，单位：ms
 */
void Quintic_Curve_Calc(float *aim_p,
                        float *aim_v,
                        float *aim_a,
                        Quintic_Line quintic,
                        int32_t tim,
                        int32_t tim_total)
{
    if (tim > tim_total)
        tim = tim_total;
    if (tim < 0)
        tim = 0;

    const float t = (float)tim;

    const float t2 = t * t;
    const float t3 = t2 * t;
    const float t4 = t3 * t;
    const float t5 = t4 * t;

    /*
     * p(t) = k5*t^5 + k4*t^4 + k3*t^3 + k2*t^2 + k1*t + k0
     */
    if (aim_p != 0)
    {
        *aim_p =
            quintic.k5 * t5 +
            quintic.k4 * t4 +
            quintic.k3 * t3 +
            quintic.k2 * t2 +
            quintic.k1 * t +
            quintic.k0;
    }

    /*
     * v(t) = 5*k5*t^4 + 4*k4*t^3 + 3*k3*t^2 + 2*k2*t + k1
     *
     * 由于内部单位是 位置单位/ms，
     * 所以输出时乘 1000，转回 位置单位/s。
     */
    if (aim_v != 0)
    {
        *aim_v =
            (5.0f * quintic.k5 * t4 +
             4.0f * quintic.k4 * t3 +
             3.0f * quintic.k3 * t2 +
             2.0f * quintic.k2 * t +
             quintic.k1) *
            1000.0f;
    }

    /*
     * a(t) = 20*k5*t^3 + 12*k4*t^2 + 6*k3*t + 2*k2
     *
     * 由于内部单位是 位置单位/ms^2，
     * 所以输出时乘 1000^2，转回 位置单位/s^2。
     */
    if (aim_a != 0)
    {
        *aim_a =
            (20.0f * quintic.k5 * t3 +
             12.0f * quintic.k4 * t2 +
             6.0f * quintic.k3 * t +
             2.0f * quintic.k2) *
            1000000.0f;
    }
}

void Cubic_Curve_Calc(float* aim_p, float* aim_v, Cube_Line cube, int32_t tim, int32_t tim_total)
{
    if (tim > tim_total) tim = tim_total;
    if (tim < 0) tim = 0;
    const float t = (float)tim;
    *aim_p = cube.k3 * t * t * t + cube.k2 * t * t + cube.k1 * t + cube.k0;
    *aim_v = (3.f * cube.k3 * t * t + 2.f * cube.k2 * t + cube.k1) * 1000.f;
}

int32_t Nav_Time_Calc(uint8_t inx, int32_t v_s)
{
    float dis = Geometric_mean(Path_Point.point[inx].fpX - Path_Point.point[inx + 1].fpX, Path_Point.point[inx].fpY - Path_Point.point[inx + 1].fpY);
    return (int32_t)(dis / v_s);
}

void Trapezoid_Curve_Calc(float* aim_p, float* aim_v, float p0, float p1, float v_target, int32_t tim, int32_t tim_total)
{
    float v_target_ms = v_target / 1000.f;
    if (fabs(p1 - p0) / fabs(v_target_ms) >=  tim_total)
    {
        *aim_p = p0 + (p1 - p0) * ((float)tim / tim_total);
        *aim_v = (p1 - p0) / tim_total * 1000.f;
    }
    else
    {
        float acc = v_target_ms * v_target_ms / ((v_target_ms * tim_total) - (p1 - p0));
        if (tim < (int32_t)(v_target_ms / acc))
        {
            *aim_p = p0 + 0.5f * acc * ((float)tim * tim);
            *aim_v = acc * tim * 1000.f;
        }
        else if (tim < (int32_t)(tim_total - v_target_ms / acc))
        {
            *aim_p = p0 + 0.5f * v_target_ms * v_target_ms / acc + v_target_ms * ((float)tim - (v_target_ms / acc));
            *aim_v = v_target_ms * 1000.f;
        }
        else if (tim <= tim_total)
        {
            int32_t t_dec = tim_total - tim;
            *aim_p = p1 - 0.5f * acc * ((float)t_dec * t_dec);
            *aim_v = acc * t_dec * 1000.f;
        }
        else if (tim > tim_total)
        {
            *aim_p = p1;
            *aim_v = 0;
        }
    }
}

void Bezier_3rd_Set(Bezier_3rd *bezier, ST_VECTOR *p0, ST_VECTOR *v0, ST_VECTOR *p1, ST_VECTOR *v1, int32_t tim_total)
{
    bezier->pos1 = *p0;
    bezier->pos2.fpX = p0->fpX + v0->fpX * tim_total / 3000.f;
    bezier->pos2.fpY = p0->fpY + v0->fpY * tim_total / 3000.f;
    bezier->pos3.fpX = p1->fpX - v1->fpX * tim_total / 3000.f;
    bezier->pos3.fpY = p1->fpY - v1->fpY * tim_total / 3000.f;
    bezier->pos4 = *p1;

    Cubic_Curve_Set(&bezier->yaw, p0->fpW, v0->fpW, p1->fpW, v1->fpW, tim_total);
}

void Bezier_3rd_Calc(Bezier_3rd bezier, float *aim_px, float *aim_py, float *aim_yaw, float *aim_vx, float *aim_vy, float *aim_vyaw,  int32_t tim, int32_t tim_total)
{
    float t = (float)tim / tim_total;
    if (t > 1) t = 1;
    
    *aim_px = (1 - t) * (1 - t) * (1 - t) * bezier.pos1.fpX + 3 * (1 - t) * (1 - t) * t * bezier.pos2.fpX + 3 * (1 - t) * t * t * bezier.pos3.fpX + t * t * t * bezier.pos4.fpX;
    *aim_py = (1 - t) * (1 - t) * (1 - t) * bezier.pos1.fpY + 3 * (1 - t) * (1 - t) * t * bezier.pos2.fpY + 3 * (1 - t) * t * t * bezier.pos3.fpY + t * t * t * bezier.pos4.fpY;
    *aim_vx = (3 * (1 - t) * (1 - t) * (bezier.pos2.fpX - bezier.pos1.fpX) + 6 * (1 - t) * t * (bezier.pos3.fpX - bezier.pos2.fpX) + 3 * t * t * (bezier.pos4.fpX - bezier.pos3.fpX)) * 1000.f / tim_total;
    *aim_vy = (3 * (1 - t) * (1 - t) * (bezier.pos2.fpY - bezier.pos1.fpY) + 6 * (1 - t) * t * (bezier.pos3.fpY - bezier.pos2.fpY) + 3 * t * t * (bezier.pos4.fpY - bezier.pos3.fpY)) * 1000.f / tim_total;

    Cubic_Curve_Calc(aim_yaw, aim_vyaw, bezier.yaw, tim, tim_total);
}

float Angle_Limit(float delta_a){
    while (delta_a >= 180.f) delta_a -= 360.f;
    while (delta_a < -180.f) delta_a +=360.f;
    return delta_a;
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
