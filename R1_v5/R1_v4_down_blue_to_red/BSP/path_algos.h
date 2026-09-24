#ifndef ___PATH_ALGOS_H___
#define ___PATH_ALGOS_H___


#include "math.h"
#include "string.h"
#include "stm32h7xx_hal.h"
#include "math_algos.h"

#define WAYPOINT_SIZE 50
#define ARC_TIME(R,vel) (1.656f*(R)/(vel))//近似匀速转弯(90度)时的理想时间，对于其他角度只需按比例放缩即可

typedef struct
{
    float a;
    float b;
    float p_start;
    float p_end;
    float v_start;
    float v_end;
    float T;
    float t_start;
    float p;
    float v;
    uint8_t init_flag;
    
}CUBIC_SPLINE;//t单位为秒，其余单位前后一致即可

typedef struct
{
    float a;
    float b;
    float c;
    float p_start;
    float p_end;
    float v_start;
    float v_end;
    float acc_start;
    float acc_end;
    float T;
    float t_start;
    float p;
    float v;
    float acc;
    uint8_t init_flag;
    
}QUINTIC_SPLINE;//t单位为秒，其余单位前后一致即可

typedef struct{
    float fpPosX;//mm
    float fpPosY;
    float fpPosQ;//deg,逆时针为正
}ST_POS;

typedef struct{
    float fpVx;//mm/s
    float fpVy;
    float fpW;//deg/s
}ST_VEL;

typedef struct{
    float fpAx;
    float fpAy;
    float fpAq;
}ST_ACC;
//轨迹点插值法, 使用三次路径（因为三次相比于五次更可控）
//注，导航和路径系统中均为全局坐标系，没有局部坐标系，单位均为mm,s,mm/s,degree
// 轨迹点 (包含时间戳)

typedef struct {
    ST_POS pos;
    ST_VEL vel;
    float time;    // 该状态点的时间 (s)(相对于上一个点)
}PATH_WAYPOINT;

typedef struct{
    PATH_WAYPOINT point[WAYPOINT_SIZE];//注意，这其中的第一个点是空的，用于记录起始位置
    CUBIC_SPLINE spline_Vx;
    CUBIC_SPLINE spline_Vy;
    CUBIC_SPLINE spline_W;
    uint8_t point_pos;//当前目标点的位置（从1开始）
	uint8_t point_num;//点的总数
    uint8_t init_flag;//0为未初始化，1为已初始化
    float t_start;//起始时间
    float point_t_end;//当前点的结束时间，目标点的起始时间
    float t_sum;//总时间
    float t_remain;//剩余时间，单位是s
    float progress;//当前总时间进度，0-1之间
}PATH_POINTS;

typedef struct 
{
    ST_POS pos;//输出的角速度已经归一化过了
    ST_VEL vel;
}PATH_OBJECT;//单位也是mm,s,mm/s,deg





extern PATH_POINTS Path_Points;

uint8_t Path_isEnd(void);
void Path_Spline(PATH_OBJECT *object);
void Path_Init(ST_POS *start_pos, ST_POS *final_pos);
void Dubins_Path(PATH_WAYPOINT* start, PATH_WAYPOINT* arc , PATH_WAYPOINT* end, float R, float a_max);
void Path_time_fill(PATH_WAYPOINT *start, PATH_WAYPOINT *end, float k);
void Path_vel_fill(PATH_WAYPOINT *start, PATH_WAYPOINT *pass, PATH_WAYPOINT *end, float speed);
void Path_Reset(void);
void The_Fastest_Path(PATH_WAYPOINT points[4], float a_max, float v_max);
void Points_pop(PATH_WAYPOINT *points, uint8_t index);
void Points_time_extension(PATH_WAYPOINT *point, float k_time);
void yaw_plan(PATH_WAYPOINT *start_point, PATH_WAYPOINT *pass_point, PATH_WAYPOINT *end_point);
void yaw_plan_4points(PATH_WAYPOINT points[4]);
void yaw_choose(PATH_WAYPOINT *start_point, PATH_WAYPOINT *pass_point, PATH_WAYPOINT *end_point, uint8_t flag);










#endif // ___PATH_ALGOS_H___




