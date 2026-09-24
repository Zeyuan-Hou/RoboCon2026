#include "point_library.h"
#include "vision.h"

#define MF_TOP 8000
#define MF_BTM 3200
#define MF_LEFT 1200
#define MF_RIGHT 4800
#define CORNER_DELTA 0 // 用来补偿曲率半径的值
#define CORNER_R (520 + CORNER_DELTA)

#define TO_AREA3_SPEED 2100

#define MIR_WIDTH 6000 // 用来从蓝场转移到红场

#define CATCH_DIST 500

//2号红场代码

#define PUT_KFS_IN_THIRD_COLUMN 0x3001	//LEFT
#define PUT_KFS_IN_SECOND_COLUMN 0x3002	//MIDDLE
#define PUT_KFS_IN_FITST_COLUMN 0x3003	//RIGHT

#define MIRRORED_PUT_KFS_IN_THIRD_COLUMN 0x3004		//LEFT
#define MIRRORED_PUT_KFS_IN_SECOND_COLUMN 0x3005	//MIDDLE
#define MIRRORED_PUT_KFS_IN_FITST_COLUMN 0x3006		//RIGHT

#define COMBINE_WITH_SECOND_COLUMN_R2 0x3011	//R1 AT LEFT
#define COMBINE_WITH_FIRST_COLUMN_R2 0x3012 	//R1 AT MIDDLE

#define MIRRORED_COMBINE_WITH_THIRD_COLUMN_R2 0x3015		//R1 AT MIDDLE
#define MIRRORED_COMBINE_WITH_SECOND_COLUMN_R2 0x3016		//R1 AT RIGHT

#define PUSH_PLATFORM_FOR_SECOND_COLUMN_R2 0x3017	//R1 AT LEFT
#define PUSH_PLATFORM_FOR_FIRST_COLUMN_R2 0x3018	//R1 AT MIDDLE

#define MIRRORED_PUSH_PLATFORM_FOR_THIRD_COLUMN_R2 0x3021		//R1 AT MIDDLE
#define MIRRORED_PUSH_PLATFORM_FOR_SECOND_COLUMN_R2 0x3022	//R1 AT RIGHT


const ST_VEL to_area3_vel = {.fpVx = -TO_AREA3_SPEED};

const PATH_WAYPOINT uphill_point[5] =                                        // 第一到四是冲坡，第五是竞技赛点，角度是按照右区写的
    {{{MIR_WIDTH - 650, 9050, 0}, {.fpVy = 1300}, .time = 0.4f},             // 入坡
     {{MIR_WIDTH - 650, 10450, 0}, {.fpVy = 1200}, .time = 0.85f}, // 出坡
     {{MIR_WIDTH - 1600, 11450, 0}, {.fpVx = -TO_AREA3_SPEED}, .time = 0.93f},
     {{MIR_WIDTH - 2750, 11000, 0}, {.fpVx = -3000}, .time = 0.5f}, // 单项赛
     {{MIR_WIDTH - 3860, 10550, 90}, {.fpVx = -3000}, .time = 0.44f}};

ST_POS area1_spot_choose(uint16_t spot)
{
    ST_POS des_pos = {0};
    switch (spot)
    {
    case 0x1001:
				des_pos = (ST_POS){3712.8, 793.01, -90.2f};
        break;
    case 0x1002://802
				des_pos = (ST_POS){3525.00, 794.895, -90.71f};
        break;
    case 0x1003://800.13
				des_pos = (ST_POS){3323.15, 797.285, -90.5f};
        break;
    case 0x1004:
        des_pos = (ST_POS){3131.635, 786.45, -91.f};
        break;
    case 0x1005://radar
				Vision_transfer_of_axes(&H_matrix, -1082, -928, &des_pos.fpPosX, &des_pos.fpPosY);
				des_pos.fpPosQ = 0.f;
				//des_pos = (ST_POS){3907.35f, 1316.63f, 0.42f};
        break;
		case 0x1006:
				des_pos = (ST_POS){5450.f, 430.f, -90.f};
				break;
    default:
        break;
    }
		
    return des_pos;
}

// 二区角点,总共8个，均为出弯道的点，这个函数不提供预期时间，vel是绝对值
PATH_WAYPOINT arcspot_choose(uint8_t curregion, uint8_t desregion, float vel)
{
    PATH_WAYPOINT point = {0};
    uint8_t regions = curregion << 4 | desregion;
    switch (regions)
    {
    case FRONT_REGION << 4 | LEFT_REGION:
        point.pos = (ST_POS){MF_LEFT - CORNER_R, MF_BTM + CORNER_DELTA, -90};
        point.vel.fpVy = vel;
        break;
    case FRONT_REGION << 4 | RIGHT_REGION:
        point.pos = (ST_POS){MF_RIGHT + CORNER_R, MF_BTM + CORNER_DELTA, 90};
        point.vel.fpVy = vel;
        break;
    case BACK_REGION << 4 | LEFT_REGION:
        point.pos = (ST_POS){MF_LEFT - CORNER_R, MF_TOP - CORNER_DELTA, -90};
        point.vel.fpVy = -vel;
        break;
    case BACK_REGION << 4 | RIGHT_REGION:
        point.pos = (ST_POS){MF_RIGHT + CORNER_R, MF_TOP - CORNER_DELTA, 90};
        point.vel.fpVy = -vel;
        break;
    case LEFT_REGION << 4 | FRONT_REGION:
        point.pos = (ST_POS){MF_LEFT + CORNER_DELTA, MF_BTM - CORNER_R, 0};
        point.vel.fpVx = vel;
        break;
    case LEFT_REGION << 4 | BACK_REGION:
        point.pos = (ST_POS){MF_LEFT + CORNER_DELTA, MF_TOP + CORNER_R, 180};
        point.vel.fpVx = vel;
        break;
    case RIGHT_REGION << 4 | FRONT_REGION:
        point.pos = (ST_POS){MF_RIGHT - CORNER_DELTA, MF_BTM - CORNER_R, 0};
        point.vel.fpVx = -vel;
        break;
    case RIGHT_REGION << 4 | BACK_REGION:
        point.pos = (ST_POS){MF_RIGHT - CORNER_DELTA, MF_TOP + CORNER_R, 180};
        point.vel.fpVx = -vel;
        break;
    default:
        point.pos = location;
        break;
    }
    return point;
}

void area2_macvel_choose(ST_VEL *mac_vel, uint16_t spot, float vel)
{ // vel为环形方向
    switch (spot)
    {
    case 0x2001:
			
        mac_vel->fpVx = -vel;
        break;
    case 0x2002:
        mac_vel->fpVx = -vel;
        break;
    case 0x2003:
        mac_vel->fpVx = -vel;
        break;
    case 0x2010:
        mac_vel->fpVy = vel;
        break;
    case 0x2020:
        mac_vel->fpVy = vel;
        break;
    case 0x2030:
        mac_vel->fpVy = vel;
        break;
    case 0x2040:
        mac_vel->fpVy = vel;
        break;
    case 0x2014:
        mac_vel->fpVy = -vel;
        break;
    case 0x2024:
        mac_vel->fpVy = -vel;
        break;
    case 0x2034:
        mac_vel->fpVy = -vel;
        break;
    case 0x2044:
        mac_vel->fpVy = -vel;
        break;
    case 0x2051:
        mac_vel->fpVx = vel;
        break;
    case 0x2052:
        mac_vel->fpVx = vel;
        break;
    case 0x2053:
        mac_vel->fpVx = vel;
        break;
    default:
        mac_vel->fpVx = 0;
        mac_vel->fpVy = 0;
        break;
    }
}

uint8_t area2_spot_choose(ST_POS *pos, uint16_t spot)
{
    switch (spot)
    {
    case 0x2003:
        *pos = (ST_POS){1800, MF_BTM - CATCH_DIST, 0};
        break;
    case 0x2002:
        *pos = (ST_POS){3000, MF_BTM - CATCH_DIST, 0};
        break;
    case 0x2001:
        *pos = (ST_POS){4200, MF_BTM - CATCH_DIST, 0};
        break;

    case 0x2010:
        *pos = (ST_POS){MF_RIGHT + CATCH_DIST, MF_BTM + 600, 90};
        break;
    case 0x2020:
        *pos = (ST_POS){MF_RIGHT + CATCH_DIST, MF_BTM + 1800, 90};
        break;
    case 0x2030:
        *pos = (ST_POS){MF_RIGHT + CATCH_DIST, MF_BTM + 3000, 90};
        break;
    case 0x2040:
        *pos = (ST_POS){MF_RIGHT + CATCH_DIST, MF_BTM + 4200, 90};
        break;

    case 0x2014:
        *pos = (ST_POS){MF_LEFT - CATCH_DIST, MF_BTM + 600, -90};
        break;
    case 0x2024:
        *pos = (ST_POS){MF_LEFT - CATCH_DIST, MF_BTM + 1800, -90};
        break;
    case 0x2034:
        *pos = (ST_POS){MF_LEFT - CATCH_DIST, MF_BTM + 3000, -90};
        break;
    case 0x2044:
        *pos = (ST_POS){MF_LEFT - CATCH_DIST, MF_BTM + 4200, -90};
        break;

    case 0x2051:
        *pos = (ST_POS){4200, MF_TOP + CATCH_DIST, 180};
        break;
    case 0x2052:
        *pos = (ST_POS){3000, MF_TOP + CATCH_DIST, 180};
        break;
    case 0x2053:
        *pos = (ST_POS){1800, MF_TOP + CATCH_DIST, 180};
        break;
    default:
        *pos = location;
        break;
    }
    if ((spot & 0xf0) == 0x50)
        return BACK_REGION;
    else if ((spot & 0xf0) == 0)
        return FRONT_REGION;
    else if ((spot & 0x0f) == 0)
        return RIGHT_REGION;
    else if ((spot & 0x0f) == 0x04)
        return LEFT_REGION;
    return 0;
}

void area3_spot_choose(ST_POS *pos, uint16_t spot)
{
    ST_POS ret = {0};
		float dt35_y = 0;

    switch (spot)
    {
    case 0x3003:
        ret = (ST_POS){250, -9486.f, -0.8f};//radar
				dt35_y = 10006.19f;
        break;
    case 0x3002:
        ret = (ST_POS){874.92f, 10546.19f, -0.8f};
        break;
    case 0x3001:
        ret = (ST_POS){874.92f, 11086.19f, -0.8f};
        break;
    case 0x3004:
        ret = (ST_POS){268.f, -10970.f, -178.7f};//radar
				dt35_y = 11484.49f;
        break;
    case 0x3005:
        ret = (ST_POS){879.81f, 10944.49f, -178.7f};
        break;
    case 0x3006:
        ret = (ST_POS){879.81f, 10404.49f, -178.7f};
        break;
		case 0x3011:
				ret = (ST_POS){599.f, 11355.f, 0.f};//
				break;
		case 0x3012://dt35
				ret = (ST_POS){599.f, 10815.f, 0.f};//
				break;
		case 0x3015://dt35back
				ret = (ST_POS){591.91, 10675.94, -179.f};//
				break;
		case 0x3016://dt35back
				ret = (ST_POS){591.91, 10135.94, -179.f};//
				break;
		case 0x3017:
				ret = (ST_POS){549, -11113, 0.f};//radar
				dt35_y = 11625.f;
				break;
		case 0x3018:
				ret = (ST_POS){599.f, 11085.f, 0.f};//
				break;
		case 0x3021://dt35back
				ret = (ST_POS){591.91f, 10405.94f, -179.f};//
				break;
		case 0x3022://dt35back
				ret = (ST_POS){548, -9348, -179.f};//radar
				dt35_y = 9865.94f;
				break;
		case 0x3041:
				ret = (ST_POS){384, -11059, 3.4};//radar
				dt35_y = 11571;
				break;//放杆点：91.95, -11165.94, 90.63//radar  //重试点：-4432.89， -10965.1074, -1.54
    default:
        ret = location;
        break;
    }
    float raw_x = ret.fpPosX;
    float raw_y = ret.fpPosY;
    float pcd_x, pcd_y;
    Vision_transfer_of_axes(&H_matrix, raw_x, raw_y, &pcd_x, &pcd_y);
    *pos = ret;
    switch (spot)
    {
    case 0x3017:
    case 0x3022:
    case 0x3041:
		case 0x3003:
		case 0x3004:
				pos->fpPosX = pcd_x;
				pos->fpPosY = dt35_y;
				break;
    default:
        break;
    }
}


//这里面是1号红场
