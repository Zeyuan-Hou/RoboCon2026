#ifndef ___MATH_ALGOS_H___
#define ___MATH_ALGOS_H___


#include <math.h>
#include "stm32h7xx_hal.h"

#define EPS 0.0001f
#define PI 3.14159265368f
#define PI2 6.2831853072f
#define PI_2 1.57079632679f
#define RADIAN 0.0174532922f
#define RADIAN_10 0.00174532922f
#define RPM_TO_RADS (PI2/60.f)//单位是rad/s
#define RAD_TO_DEG (180.f/PI)
#define DEG_TO_RAD (PI/180.f)
#define DEG10_TO_RAD (PI/1800.f)
#define RAD_TO_DEG10 (1800.f/PI)

typedef struct
{
  float x1;		//position
  float x2;	    //velocity
  float x;		//displacement
  float r;		//TD tracking factor
  float h;		//TD filtering factor
  float T;		//TD intergretion step size, which should be consistent with your sampling period
  float aim;	//target position
} ST_TD;

typedef struct
{
    float fpDes; // 控制变量目标值
    float fpFB;  // 控制变量反馈值

    float fpKp; // 比例系数Kp
    float fpKi; // 积分系数Ki
    float fpKd; // 微分系数Kd

    float fpE;    // 本次偏差
    float fpEMin; // 偏差死区
    float fpElimit;//积分分离

    float fpPreE;    // 上次偏差
    float fpSumE;    // 总偏差
    float fpSumEMax; // 积分偏差最大值

    float fpU;     // 总输出
    float fpUMax;  // 总输出最大值
    float fpUpMax; // 比例项输出限幅
    float fpUdMax; // 微分项输出上限
    float fpUiMax; // 积分项输出限幅 ////

    float fpUp; // 比例项输出
    float fpUi; // 积分项输出
    float fpUd; // 微分项输出

} ST_PID;

typedef struct{
    float fpDes; // 控制变量目标值
    float fpFB;  // 控制变量反馈值

    float fpKp; // 比例系数Kp
    float fpKi; // 积分系数Ki
    float fpKd; // 微分系数Kd

    float fpE;    // 本次偏差
    float fpEMin; // 偏差死区
    float fpElimit;//积分分离

    ST_TD td;
    float fpSumE;    // 总偏差
    float fpSumEMax; // 积分偏差最大值

    float fpU;     // 总输出
    float fpUMax;  // 总输出最大值
    float fpUpMax; // 比例项输出限幅
    float fpUdMax; // 微分项输出上限
    float fpUiMax; // 积分项输出限幅 ////

    float fpUp; // 比例项输出
    float fpUi; // 积分项输出
    float fpUd; // 微分项输出    
}ST_PID_withTD;

typedef struct{
    ST_PID outer;
    ST_PID inner;
    float outer_des;
    float outer_fb;
    float inner_fb;
    float output;
}ST_CASCADE_PID;



typedef struct{
    float vel;
	float minvel;
    ST_TD td;
    float k0;
	float k1;
    float k2;
    float output;
}ST_FF;//二阶前馈

// 一阶低通滤波
typedef struct
{
    float preout;   // 上一个输出值，用于保持滤波器状态，以便在连续调用之间维持滤波效果
    float out;      // 当前输出值，即经过低通滤波处理后的信号
    float in;       // 输入值，这是将要被滤波的原始信号
    float off_freq; // 截止频率，近似为Hz
    float samp_tim; // 采样步长，单位为s
} ST_LPF;           // 定义了一阶低通滤波器的结构体，一阶意味着它的传递函数有一阶多项式


// 陷波滤波器结构体
typedef struct {
    // 滤波器系数
    float b0, b1, b2;
    float a1, a2;
    // 状态缓冲区（延迟线）
    float x1, x2; // 输入历史
    float y1, y2; // 输出历史
} NotchFilter;


void PID_Calc(ST_PID *pid, float fpDes, float fpFB);
void PID_Calc_withoutDiff(ST_PID *pid, float fpDes, float fpFB, float v);
void PID_Calc_Angle(ST_PID *pid, float fpDes, float fpFB);
void PID_Calc_Angle_withoutDiff(ST_PID *pid, float fpDes, float fpFB, float v);
void PID_Cascade_Calc(ST_CASCADE_PID *pid, float outer_des, float outer_feedback, float inner_feedback);
void PID_Cascade_Calc_Angle(ST_CASCADE_PID *pid, float outer_des, float outer_feedback, float inner_feedback);

void FeedForward_Calc(ST_FF *ff, float vel);
void CalTD(ST_TD *pStTD);
float clipfloat(float x, float min, float max);
float wrap_to_2pi(float q);

float norm_angle(float q);
int8_t sgnf(float x);
uint32_t iabs(int a);
int u8_to_i32(unsigned char *data);
float sin_interp_fast(float x_min, float x_max, float t);
void LpFilter(ST_LPF *lpf);
void NotchFilter_Init(NotchFilter* filter, float fs, float f0, float Q);
float NotchFilter_Update(NotchFilter* filter, float input);


















#endif // ___MATH_ALGOS_H___





