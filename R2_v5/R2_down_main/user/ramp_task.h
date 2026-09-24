#ifndef RAMP_TASK_H
#define RAMP_TASK_H


#include "Type.h"
#include "chassis.h"
#include "path.h"
#define FPU_SATURATION  14000.f // fpu饱和阈值（留1000余量提前检测）
#define TARGET_POWER  85000000.0f  // 目标等效功率(需先测试)
#define POWER_I_MAX         0.3f         // 积分限幅：防止积分饱和
#define SPEED_ADJUST_MIN    0.7f         // 速度调整下限：最低降速30%
#define SPEED_ADJUST_MAX    1.3f         // 速度调整上限：最高增速30%
#define PRELOAD_MAX 1.26f         // 15度斜坡最大预加载系数
#define RAMP_CRUISE_SPEED 1550.f//巡航速度

#define RAMP_EXIT_DECEL_DISTANCE 200.0f // 距离终点200mm开始减速
#define RAMP_EXIT_SPEED_FACTOR 0.8f     // 减速到原速度的40%
typedef struct
{
    float x;
    float y;
}BezierPoint;

typedef struct
{
    float vx;
    float vy;
    float magnitude;
    float angle;
}BezierVel;

typedef struct
{
    BezierPoint p0;//起点
    BezierPoint p1;//控制点1
    BezierPoint p2;//终点
    uint32_t total_time;//时间
    float length;//路程长度
    float velt;//巡航速度
}Bezier;

void Bezier_init(Bezier *bezier, BezierPoint p0, BezierPoint p1, BezierPoint p2, float cruise_speed);
void Bezier_calc_by_time(const Bezier *bezier, uint32_t current_time, BezierPoint *position, BezierVel *velocity);
static float calc_single_wheel_power(float fpu, float wheel_speed, float target_wheel_speed);
void constant_power_control(void);
void ramp_task(void);

#endif // RAMP_TASK_H
