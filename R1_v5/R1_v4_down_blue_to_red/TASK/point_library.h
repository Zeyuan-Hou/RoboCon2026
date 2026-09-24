#ifndef ___POINT_LIBRARY_H___
#define ___POINT_LIBRARY_H___

#include "robot.h"

#define FRONT_REGION 0x01
#define RIGHT_REGION 0x02
#define BACK_REGION 0x04
#define LEFT_REGION 0x08

extern const PATH_WAYPOINT uphill_point[5];
extern const PATH_WAYPOINT backregion_to_hill;
extern const PATH_WAYPOINT leftregion_to_hill;

PATH_WAYPOINT arcspot_choose(uint8_t curregion, uint8_t desregion, float vel);
uint8_t area2_spot_choose(ST_POS *pos, uint16_t spot);
void area3_spot_choose(ST_POS *pos, uint16_t spot);
uint8_t area2_spot_choose(ST_POS *pos, uint16_t spot);;
ST_POS area1_spot_choose(uint16_t spot);
void area2_macvel_choose(ST_VEL *mac_vel, uint16_t spot, float vel);
















#endif // ___POINT_LIBRARY_H___










