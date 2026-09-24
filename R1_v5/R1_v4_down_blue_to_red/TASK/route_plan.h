#ifndef ___ROUTE_PLAN_H___
#define ___ROUTE_PLAN_H___

#include "robot.h"
#include "point_library.h"

typedef enum{
    SAME=0,
    NEAR=1,
    OPPO_1=2,//opposite,前后
    OPPO_2=3,//opposite,左右
}REGION_RELATION;

extern uint8_t mac_step;
extern ST_VEL MAC_vel[3]; // 速度大小（或许会固定为1个值）
extern int8_t mac_dir[3]; 
extern uint8_t MAC_spotnum;
extern uint16_t MAC_spot[3];

void route_choose(uint16_t spot);
void Area1_Route(PATH_WAYPOINT *points, uint16_t des_spot);
void Area2_Route(PATH_WAYPOINT *points, uint16_t des_spot, ST_POS *cur_pos);
void Area3_Route(PATH_WAYPOINT *points, uint16_t des_spot, ST_POS *cur_pos);
uint8_t area2_divide(ST_POS* pos);
REGION_RELATION area2_judge(uint8_t region1, uint8_t region2);
REGION_RELATION area2_judge_new(uint8_t region1, uint8_t region2);
void route_into_middle(uint8_t region, PATH_WAYPOINT* point);
void area2_oppo_route(PATH_WAYPOINT *points, uint8_t region_start, uint8_t region_pass, uint8_t region_end, ST_POS *des_pos, ST_POS *cur_pos);
uint8_t corner_slowdown_judge(uint16_t des_spot, ST_VEL *mac_vel);
void mac_Area2_Route(PATH_WAYPOINT *points, uint16_t des_spot, ST_VEL *mac_vel, ST_VEL *start_vel);
void mac_dist_lengthen(ST_POS *pos, ST_VEL *mac_vel);
int8_t dist_judge(uint16_t start_spot, uint16_t end_spot);
void mac_area2_oppo_route(PATH_WAYPOINT *points, uint8_t region_start, uint8_t region_pass, uint8_t region_end, ST_POS *des_pos, ST_VEL *mac_vel, float corner_end_vel, float line_end_vel);
void area3_angle_optimize(PATH_WAYPOINT *start_point, PATH_WAYPOINT *pass_point, PATH_WAYPOINT *end_point);
void area2_speed_optimized(PATH_WAYPOINT points[4]);
void MAC_Route_Init(uint16_t spot1, uint16_t spot2, uint16_t spot3);
void MAC_Route(uint16_t spot_into_area3);
void mac_Area3_Route(PATH_WAYPOINT *points,uint16_t des_spot , ST_VEL *start_vel);
void mac_Area2to3_Route(PATH_WAYPOINT *points, ST_VEL *start_vel);
#endif // ___ROUTE_PLAN_H___



