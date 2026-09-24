#include "route_plan.h"

#define MF_BTM 3200            // 梅林起始处的Y坐标
#define MF_TOP (MF_BTM + 4800) // 梅林终点处的Y坐标
#define MF_LEFT 1200           // 梅林左边界
#define MF_RIGHT 4800          // 梅林右边界

#define ROBOT_R 300 // 车半长
// #define STRAIGHT_VEL 1400 // 直道速度
// #define A_MAX 1500.f      // 最大加速度
#define CORNER_R 550 // 转弯半径
// #define CORNER_VEL 700    // 转弯速度
#define MAC_TIME 0.1f
#define UPHILL_SPEED 2000
#define UPHILL_ACC 300
#define AREA3_ACC 3000
#define AREA3_VEL 3500
#define AREA3_VEL_1 1800
#define AREA3_VEL_2 1800
#define BRAKING_TIME 1.5f                                                                         // 刹车时间
#define OPTIMIZE_TIME(v_start, v_end) (3 * ((v_start) + (v_end)) / (2 * (v_start) + 4 * (v_end))) // 优化时间

#define MAC_DIST 300

#define EXTRACT_NEAR_REGION(cur, des) ((cur) & ~(((des) << 2 | (des) >> 2) & 0x0f)) // 提取出复合区域中靠近目标区域的部分
#define EXTRACT_FAR_REGION(cur, des) ((cur) & (((des) << 2 | (des) >> 2) & 0x0f))   // 提取出复合区域中远离目标区域的部分

// 目前的理想末端加速度 约850

float STRAIGHT_VEL = 2700;
float A_MAX = 2700;
float CORNER_VEL = 1300;

float test_time = 1.3f;

// 常规路径导航
void route_choose(uint16_t spot)
{
    if (!Path_isEnd())
        return;

    PATH_WAYPOINT points[WAYPOINT_SIZE] = {0};
    ST_POS cur_pos = {0};
    cur_pos = location;

    points[0].pos = cur_pos;

    if (spot >> 12 == 0x02 && spot != 0x2100)
    {
        Area2_Route(points, spot, &cur_pos);
    }
    else if (spot >> 12 == 0x03)
    {
        Area3_Route(points, spot, &cur_pos);
    }
    else if (spot >> 12 == 0x01)
    {
        Area1_Route(points, spot);
    }
    else
    {
        switch (spot)
        {
        case 0x0001: // 测试路径
            points[1].pos = (ST_POS){4000, 10500, 180};
            points[1].time = test_time;
            break;
        case 0x0002:
            points[1].pos = (ST_POS){2500, 10500, 180};
            points[1].time = test_time;
            break;
        case 0x0003:
            points[1].pos = location;
            points[1].pos.fpPosQ = 120;
            points[1].vel.fpW = 120;
            points[1].time = 1;
            points[2].pos = location;
            points[2].pos.fpPosQ = -120;
            points[2].vel.fpW = 120;
            points[2].time = 1;
            points[3].pos = location;
            points[3].pos.fpPosQ = 0;
            points[3].vel.fpW = 120;
            points[3].time = 1;

            break;
        case 0x0004:
            // 启动位置应该在{850, 1000, 180}
            points[2].pos = (ST_POS){2000, 1550, -90};
            points[2].vel.fpVy = CORNER_VEL;
            Dubins_Path(points, &points[1], &points[2], CORNER_R, A_MAX);
            points[3].pos = (ST_POS){2000, 2150, -90};
            Path_time_fill(&points[2], &points[3], 1);
            yaw_plan_4points(points);

            break;
        case 0x2100:
            points[1].pos = (ST_POS){5380, 7438, 0};
            points[1].vel.fpVy = 2000;
            points[1].time = 0.77f;
            points[2].pos = (ST_POS){5380, 9440, 0};
            points[2].vel.fpVy = 2500;
            points[2].time = 0.95f;
            points[3].pos = (ST_POS){4980, 10150, 0};
            points[3].vel.fpVx = -1000;
            points[3].time = 0.54f;
            points[4].pos = (ST_POS){4100, 9700, 0};
            points[4].vel = (ST_VEL){-700, -700, 0};
            points[4].time = 0.9f;
            points[5].pos = (ST_POS){3582, 9532, 0};
            points[5].time = 1.f;
            break;
        default:
            break;
        }
    }
    memcpy(Path_Points.point, points, sizeof(PATH_WAYPOINT) * WAYPOINT_SIZE);
}

void Area1_Route(PATH_WAYPOINT *points, uint16_t des_spot) // 默认points[0]为当前位置
{
    ST_POS cur_pos = points[0].pos;
    ST_POS des_pos = area1_spot_choose(des_spot);

    if ((des_spot & 0x0f) < 5)
    {
        if (cur_pos.fpPosX < 4500)
        {
            points[1].pos.fpPosX = (cur_pos.fpPosX + des_pos.fpPosX) / 2;
            points[1].pos.fpPosY = des_pos.fpPosY + 250;
            points[1].pos.fpPosQ = -90;
            points[1].vel.fpVx = sgnf(des_pos.fpPosX - cur_pos.fpPosX) * 225;
            points[1].time = 0.8f;
            points[2].pos = des_pos;
            points[2].time = 0.8f;
        }
        else
        {
            points[1].pos.fpPosY = des_pos.fpPosY + 150;
            points[1].pos.fpPosX = (2 * location.fpPosX + 3 * des_pos.fpPosX) / 5.f;
            points[1].pos.fpPosQ = -90;
            points[1].vel.fpVx = -2000;
            Path_time_fill(&points[0], &points[1], 1.1f);
            points[2].pos = des_pos;
            Path_time_fill(&points[1], &points[2], 1.35f);
        }
    }
    else if (des_spot == 0x1005)
    { // 0x1005对接点
        points[0].vel.fpVy = 250;
        points[1].pos.fpPosY = des_pos.fpPosY;
        points[1].pos.fpPosX = location.fpPosX - 400;
        points[1].vel.fpVx = -1500;
        points[1].time = 0.8f;
        points[2].pos = des_pos;
        points[2].pos.fpPosX = des_pos.fpPosX + 400;
        if (points[2].pos.fpPosX > points[1].pos.fpPosX)
            points[2].pos.fpPosX = points[1].pos.fpPosX;
        points[2].vel.fpVx = -1500;
        Path_time_fill(&points[1], &points[2], 1);
        yaw_plan(&points[0], &points[1], &points[2]);
        points[3].pos = des_pos;
        points[3].vel.fpVx = -200;
        Path_time_fill(&points[2], &points[3], 1.f);
    }
    else if (des_spot == 0x1006)
    {
        points[1].pos.fpPosX = (location.fpPosX + des_pos.fpPosX) / 2.f;
        points[1].pos.fpPosY = location.fpPosY;
        points[1].vel.fpVx = 2700;
        points[2].pos = des_pos;
        Path_time_fill(&points[0], &points[1], 1.f);
        Path_time_fill(&points[1], &points[2], 1.05f);
        yaw_plan(&points[0], &points[1], &points[2]);
    }
}

/**
 * 二区点的命名规则：例如0x2010表示2区，左区，第1行第0列
 * 第9~12位：0b0001为前区1，0b0010为右区2，0b0100为后区4，0b1000为左区8
 */

void Area2_Route(PATH_WAYPOINT *points, uint16_t des_spot, ST_POS *cur_pos)
{
    uint8_t cur_region = area2_divide(cur_pos);
    ST_POS des_pos;
    uint8_t des_region = area2_spot_choose(&des_pos, des_spot);
    switch (area2_judge(cur_region, des_region))
    {
    case SAME:
        points[3].pos = des_pos;
        The_Fastest_Path(&points[0], A_MAX, STRAIGHT_VEL); // area2_speed_optimized(&points[0]);
        break;
    case NEAR:
        cur_region = EXTRACT_NEAR_REGION(cur_region, des_region);
        points[4] = arcspot_choose(cur_region, des_region, CORNER_VEL);
        if (location.fpPosY < 2000 && cur_region == FRONT_REGION)
            points[4].pos.fpPosY -= 600;
        Dubins_Path(&points[0], &points[3], &points[4], CORNER_R, A_MAX);
        points[7].pos = des_pos;
        The_Fastest_Path(&points[0], A_MAX, STRAIGHT_VEL);
        The_Fastest_Path(&points[4], A_MAX, STRAIGHT_VEL);
        yaw_plan_4points(&points[2]);
        break;
    case OPPO_1:
        cur_region = EXTRACT_FAR_REGION(cur_region, des_region);
        if (cur_pos->fpPosX + des_pos.fpPosX < MF_LEFT + MF_RIGHT)
            area2_oppo_route(points, cur_region, LEFT_REGION, des_region, &des_pos, cur_pos);
        else
            area2_oppo_route(points, cur_region, RIGHT_REGION, des_region, &des_pos, cur_pos);
        break;
    case OPPO_2:
        cur_region = EXTRACT_FAR_REGION(cur_region, des_region);
        if (cur_pos->fpPosY + des_pos.fpPosY > MF_TOP + MF_BTM)
            area2_oppo_route(points, cur_region, BACK_REGION, des_region, &des_pos, cur_pos);
        else
            area2_oppo_route(points, cur_region, FRONT_REGION, des_region, &des_pos, cur_pos);
        break;
    default:
        break;
    }
}

float test_speed = 1000;

void Area3_Route(PATH_WAYPOINT *points, uint16_t des_spot, ST_POS *cur_pos)
{
    ST_POS des_pos = {0};
    area3_spot_choose(&des_pos, des_spot);
    if (cur_pos->fpPosY < 9450)
    { // 冲坡前
        uint8_t cur_region = area2_divide(cur_pos);
        uint8_t point_curnum = 1;
        if (cur_region & RIGHT_REGION)
        {
            if (location.fpPosY < 8000 || fabsf(norm_angle(location.fpPosQ - 90)) < 20)
            {
                points[3] = uphill_point[0];
                points[3].pos.fpPosQ = 90;
                The_Fastest_Path(&points[0], A_MAX, STRAIGHT_VEL);
                memcpy(&points[6], &uphill_point[1], 2 * sizeof(PATH_WAYPOINT));
                points[6].pos.fpPosQ = 90;
                The_Fastest_Path(&points[3], UPHILL_ACC, UPHILL_SPEED);
                points[7].pos.fpPosQ = 90;
                point_curnum = 8;
            }
            else
            { // 给单项赛准备
                if (des_spot == 0x3001 || des_spot == 0x3004)
                {
                    memcpy(&points[3], &uphill_point[1], 3 * sizeof(PATH_WAYPOINT));
                    points[3].vel.fpVy = 1300;
                    points[3].pos.fpPosQ = 0;
                    The_Fastest_Path(&points[0], 1800, UPHILL_SPEED);
                    points[4].pos.fpPosQ = 0;
                    points[5].pos.fpPosQ = 0;
                    point_curnum = 6;
                }
                else
                {
                    memcpy(&points[3], &uphill_point[1], 2 * sizeof(PATH_WAYPOINT));
                    points[3].vel.fpVy = 1300;
                    points[3].pos.fpPosQ = 0;
                    The_Fastest_Path(&points[0], 1800, UPHILL_SPEED);
                    points[4].pos.fpPosQ = 0;
                    point_curnum = 5;
                }
            }
        }
        else if (cur_region & BACK_REGION)
        {
            points[4] = uphill_point[0];
            points[4].pos.fpPosQ = -90;
            Dubins_Path(&points[0], &points[3], &points[4], 450, 1300);
            The_Fastest_Path(&points[0], A_MAX, STRAIGHT_VEL);
            memcpy(&points[7], &uphill_point[1], 2 * sizeof(PATH_WAYPOINT));
            points[7].pos.fpPosQ = -90;
            The_Fastest_Path(&points[4], UPHILL_ACC, UPHILL_SPEED);
            points[8].pos.fpPosQ = -90;
            yaw_plan_4points(&points[1]);
            point_curnum = 9;
        }
        else if (cur_region & FRONT_REGION)
        {
            points[4] = arcspot_choose(FRONT_REGION, RIGHT_REGION, CORNER_VEL);
            Dubins_Path(&points[0], &points[3], &points[4], CORNER_R, A_MAX);
            points[3].pos.fpPosQ = 0;
            points[4].pos.fpPosQ = 0;
            points[7] = uphill_point[0];
            points[7].pos.fpPosQ = 0;
            memcpy(&points[10], &uphill_point[1], 2 * sizeof(PATH_WAYPOINT));
            points[10].pos.fpPosQ = 0;
						points[11].pos.fpPosQ = 0;
            The_Fastest_Path(&points[0], A_MAX, STRAIGHT_VEL);
            The_Fastest_Path(&points[4], A_MAX, STRAIGHT_VEL);
            The_Fastest_Path(&points[7], UPHILL_ACC, UPHILL_SPEED);
            point_curnum = 12;
        }
        else if (cur_region & LEFT_REGION)
        {
            points[4] = arcspot_choose(LEFT_REGION, BACK_REGION, CORNER_VEL);
            Dubins_Path(&points[0], &points[3], &points[4], CORNER_R, A_MAX);
            points[4].pos.fpPosQ = -90;
            points[8] = uphill_point[0];
            points[8].pos.fpPosQ = -90;
            Dubins_Path(&points[4], &points[7], &points[8], 450, 1300);
            points[7].pos.fpPosQ = -90;
            memcpy(&points[11], &uphill_point[1], 2 * sizeof(PATH_WAYPOINT));
            points[11].pos.fpPosQ = -90;
            points[12].pos.fpPosQ = -90;
            The_Fastest_Path(&points[0], A_MAX, STRAIGHT_VEL);
            The_Fastest_Path(&points[4], A_MAX, STRAIGHT_VEL);
            The_Fastest_Path(&points[8], UPHILL_ACC, UPHILL_SPEED);
            point_curnum = 13;
        }

        mac_Area3_Route(&points[point_curnum - 1], des_spot, &points[point_curnum - 1].vel);
    }
    else
    { // 在三区
        if (((des_spot & 0x0ff0) == 0 && (des_spot & 0x0f) <= 0x06) && fabsf(norm_angle(location.fpPosQ - des_pos.fpPosQ)) <= 120)
        {
            float des_yaw = des_pos.fpPosQ;
            points[2].pos = des_pos;
            points[2].pos.fpPosX = des_pos.fpPosX + 320;
            points[2].vel.fpVx = - test_speed;
            Dubins_Path(&points[0], &points[1], &points[2], 240, 2700);
            points[1].pos.fpPosQ = des_yaw;
            points[3].pos = des_pos;
            Path_time_fill(&points[2], &points[3], 1);
        }
        else
        {
            if (fabsf(norm_angle(location.fpPosQ - des_pos.fpPosQ)) > 120)
            {
                if (location.fpPosQ > -90 && location.fpPosQ < 90)
                {
                    points[2].pos = des_pos;
                    points[2].time = hypotf(des_pos.fpPosX - location.fpPosX, des_pos.fpPosY - location.fpPosY) * 2.5f / AREA3_VEL_2;
                    if (points[2].time < 3.f)
                        points[2].time = 3.f;
                    yaw_choose(&points[0], &points[1], &points[2], 0);
                }
                else
                {
                    points[2].pos = des_pos;
                    points[2].time = hypotf(des_pos.fpPosX - location.fpPosX, des_pos.fpPosY - location.fpPosY) * 2.5f / AREA3_VEL_2;
                    if (points[2].time < 3.f)
                        points[2].time = 3.f;
                    yaw_choose(&points[0], &points[1], &points[2], 1);
                }
                points[1].pos.fpPosX = location.fpPosX + 700;
            }
            else
            {
                points[3].pos = des_pos;
                The_Fastest_Path(&points[0], 1500, AREA3_VEL_2);
            }
        }
    }
}

uint8_t area2_divide(ST_POS *pos)
{
    uint8_t region = 0;
    if (pos->fpPosY < MF_BTM - 200)
        region |= FRONT_REGION; // 留了10cm的误差
    if (pos->fpPosY > MF_TOP + 200)
        region |= BACK_REGION;
    if (pos->fpPosX < MF_LEFT - 200)
        region |= LEFT_REGION;
    if (pos->fpPosX > MF_RIGHT + 200)
        region |= RIGHT_REGION;
    return region;
}

REGION_RELATION area2_judge(uint8_t region1, uint8_t region2)
{
    if (region1 & region2)
        return SAME;
    else if ((region1 | region2) == (FRONT_REGION | BACK_REGION))
        return OPPO_1;
    else if ((region1 | region2) == (LEFT_REGION | RIGHT_REGION))
        return OPPO_2;
    else
        return NEAR;
}

REGION_RELATION area2_judge_new(uint8_t region1, uint8_t region2)
{ // 相比于上个函数，这个函数在复合区域的NEAR情况下会返回OPPO
    if (region1 & region2)
        return SAME;
    else
    {
        uint8_t region1_farside = EXTRACT_FAR_REGION(region1, region2);
        if ((region1_farside | region2) == (FRONT_REGION | BACK_REGION))
            return OPPO_1;
        else if ((region1_farside | region2) == (LEFT_REGION | RIGHT_REGION))
            return OPPO_2;
        else
            return NEAR;
    }
}

void area2_oppo_route(PATH_WAYPOINT *points, uint8_t region_start, uint8_t region_pass, uint8_t region_end, ST_POS *des_pos, ST_POS *cur_pos)
{
    points[4] = arcspot_choose(region_start, region_pass, CORNER_VEL);
    Dubins_Path(&points[0], &points[3], &points[4], CORNER_R, A_MAX);
    points[8] = arcspot_choose(region_pass, region_end, CORNER_VEL);
    Dubins_Path(&points[4], &points[7], &points[8], CORNER_R, A_MAX);
    points[11].pos = *des_pos;
    The_Fastest_Path(&points[0], A_MAX, STRAIGHT_VEL);
    The_Fastest_Path(&points[4], A_MAX, STRAIGHT_VEL);
    The_Fastest_Path(&points[8], A_MAX, STRAIGHT_VEL);
    yaw_plan_4points(&points[2]);
    yaw_plan_4points(&points[6]);
}

void route_into_middle(uint8_t region, PATH_WAYPOINT *point)
{
    switch (region)
    {
    case FRONT_REGION:
        // point->pos.fpPosY = 1375;
        point->pos.fpPosY = Vision.front_region_mid;
        break;
    case BACK_REGION:
        // point->pos.fpPosY = 7360;
        point->pos.fpPosY = Vision.back_region_mid;
        break;
    case LEFT_REGION:
        // point->pos.fpPosX = 610;
        point->pos.fpPosX = Vision.left_region_mid;
        break;
    case RIGHT_REGION:
        // point->pos.fpPosX = 5385;
        point->pos.fpPosX = Vision.right_region_mid;
        break;
    default:
        break;
    }
}

void area2_speed_optimized(PATH_WAYPOINT points[4])
{
    The_Fastest_Path(&points[0], A_MAX, STRAIGHT_VEL);
    points[1].time *= OPTIMIZE_TIME(hypotf(points[0].vel.fpVx, points[0].vel.fpVy), hypotf(points[1].vel.fpVx, points[1].vel.fpVy));
    points[3].time *= OPTIMIZE_TIME(hypotf(points[2].vel.fpVx, points[2].vel.fpVy), hypotf(points[3].vel.fpVx, points[3].vel.fpVy));
}

// 以下为边走边吸
//这一部分目前是蓝场代码
/**
 * 大致策略：分为顺时针和逆时针两种情况，优先贪心（因为这样可以保证最后一个块之后是与三区相接的）
 * 满足以下任意情况则不边走边吸：
 * 1.上一个方块离自己的折线距离小于1500(就是下一个格子)
 * 2.下一段路径和当前路径的朝向不一致
 * 否则就选择边走边吸
 */

/**
 * 新的边走边吸函数：
 * 每次调用返回一个值以及导航的下一个状态，并生成所需路径
 */
// 在将二区视为环形通道的地方，记1,2,3块的方向为正方向
uint16_t MAC_spot[3] = {0};
ST_VEL MAC_vel[3] = {0}; // 速度大小（或许会固定为1个值）
int8_t mac_dir[3] = {0};
uint8_t MAC_spotnum = 0; // 取块点个数
uint8_t mac_step = 0;    // 已取块个数
const float MAC_SPEED = 300;

void MAC_Route_Init(uint16_t spot1, uint16_t spot2, uint16_t spot3)
{
    MAC_spot[0] = spot1;
    MAC_spot[1] = spot2;
    MAC_spot[2] = spot3;
    int8_t dist[4] = {0}; // 1表示逆时针，-1表示顺时针，0表示不存在
    if (spot2 == 0)
    {
        MAC_spotnum = 1;
        dist[0] = dist_judge(0x2003, spot1);
        dist[1] = dist_judge(spot1, 0x2050);
    }
    else if (spot3 == 0)
    {
        MAC_spotnum = 2;
        dist[0] = dist_judge(0x2003, spot1);
        dist[1] = dist_judge(spot1, spot2);
        dist[2] = dist_judge(spot2, 0x2050);
    }
    else
    {
        MAC_spotnum = 3;
        dist[0] = dist_judge(0x2003, spot1);
        dist[1] = dist_judge(spot1, spot2);
        dist[2] = dist_judge(spot2, spot3);
        dist[3] = dist_judge(spot3, 0x2050);
    }
    for (int i = 0; i < MAC_spotnum; i++)
    {
        if ((dist[i] * dist[i + 1] <= 0 || (dist[i] > -2 && dist[i] < 2)) && !(i == 0 && MAC_spot[0] == 0x2002)) // 例外情况
        {
            memset(&MAC_vel[i], 0, sizeof(ST_VEL));
            mac_dir[i] = 0;
        }
        else
        {
            if (dist[i] < 0)
            {
                area2_macvel_choose(&MAC_vel[i], MAC_spot[i], -MAC_SPEED);
                mac_dir[i] = -1;
            }
            else
            {
                area2_macvel_choose(&MAC_vel[i], MAC_spot[i], MAC_SPEED);
                mac_dir[i] = 1;
            }
        }
    }
}

void MAC_Route(uint16_t spot_into_area3)
{
    if (!Path_isEnd())
        return;
    //    ST_POS cur_pos = location;
    Path_Points.point[0].pos = location;

    switch (mac_step)
    {
    case 0:
        if (MAC_spot[0] != 0x2002 || MAC_vel[0].fpVx == 0)
            mac_Area2_Route(Path_Points.point, MAC_spot[0], &MAC_vel[0], &Path_Points.point[0].vel); // 第一段路径的起始速度就是自己（速度为0）
        else
        {
            ST_POS des_pos = {0};
            area2_spot_choose(&des_pos, 0x2002);
            PATH_WAYPOINT *points = Path_Points.point;
            points[4].pos = des_pos;
            points[4].pos.fpPosX = des_pos.fpPosX + MAC_DIST;
            points[4].vel = MAC_vel[0];
            Dubins_Path(&points[0], &points[3], &points[4], CORNER_R, A_MAX);
            points[3].pos.fpPosQ = des_pos.fpPosQ;
            The_Fastest_Path(&points[0], A_MAX, STRAIGHT_VEL);
        }

        if (MAC_spotnum == 1)
            mac_step = 3;
        else
            mac_step = 1;
        break;
    case 1:
        mac_Area2_Route(Path_Points.point, MAC_spot[1], &MAC_vel[1], &MAC_vel[0]);

        if (MAC_spotnum == 2)
            mac_step = 3;
        else
            mac_step = 2;
        break;
    case 2:
        mac_Area2_Route(Path_Points.point, MAC_spot[2], &MAC_vel[2], &MAC_vel[1]);

        mac_step = 3;
        break;
    case 3:
        mac_Area2to3_Route(Path_Points.point, &MAC_vel[MAC_spotnum - 1]);
        Nav.cur_spot = 0x2050;

        mac_step = 4;
        break;
    case 4:
        mac_Area3_Route(Path_Points.point, 0x3003, (ST_VEL *)&uphill_point[2].vel);

        mac_step = 5;
        break;
    default:
        break;
    }
}

void mac_Area2_Route(PATH_WAYPOINT *points, uint16_t des_spot, ST_VEL *mac_vel, ST_VEL *start_vel)
{
    ST_POS cur_pos = points[0].pos;
    uint8_t cur_region = area2_divide(&cur_pos);
    ST_POS des_pos;
    uint8_t des_region = area2_spot_choose(&des_pos, des_spot);
    float corner_end_vel, straight_end_vel;

    if (corner_slowdown_judge(des_spot, mac_vel))
    {
        corner_end_vel = 1000;
        straight_end_vel = 1000;
    }
    else
    {
        corner_end_vel = CORNER_VEL;
        straight_end_vel = STRAIGHT_VEL;
    }

    points[0].vel = *start_vel;
    mac_dist_lengthen(&des_pos, mac_vel);
    switch (area2_judge(cur_region, des_region))
    {
    case SAME:
        points[3].pos = des_pos;
        points[3].vel = *mac_vel;
        The_Fastest_Path(&points[0], A_MAX, STRAIGHT_VEL); // area2_speed_optimized(&points[0]);
        break;
    case NEAR:
        cur_region = EXTRACT_NEAR_REGION(cur_region, des_region);
        points[4] = arcspot_choose(cur_region, des_region, corner_end_vel);
        if (location.fpPosY < 2000 && cur_region == FRONT_REGION)
            points[4].pos.fpPosY -= 600;
        Dubins_Path(&points[0], &points[3], &points[4], CORNER_R, A_MAX);
        points[7].pos = des_pos;
        points[7].vel = *mac_vel;
        The_Fastest_Path(&points[0], A_MAX, STRAIGHT_VEL);
        The_Fastest_Path(&points[4], A_MAX, straight_end_vel);
        yaw_plan_4points(&points[2]);
        break;
    case OPPO_1:
        cur_region = EXTRACT_FAR_REGION(cur_region, des_region);
        if (cur_pos.fpPosX + des_pos.fpPosX < MF_LEFT + MF_RIGHT)
            mac_area2_oppo_route(points, cur_region, LEFT_REGION, des_region, &des_pos, mac_vel, corner_end_vel, straight_end_vel);
        else
            mac_area2_oppo_route(points, cur_region, RIGHT_REGION, des_region, &des_pos, mac_vel, corner_end_vel, straight_end_vel);
        break;
    case OPPO_2:
        cur_region = EXTRACT_FAR_REGION(cur_region, des_region);
        if (cur_pos.fpPosY + des_pos.fpPosY > MF_TOP + MF_BTM)
            mac_area2_oppo_route(points, cur_region, BACK_REGION, des_region, &des_pos, mac_vel, corner_end_vel, straight_end_vel);
        else
            mac_area2_oppo_route(points, cur_region, FRONT_REGION, des_region, &des_pos, mac_vel, corner_end_vel, straight_end_vel);
        break;
    default:
        break;
    }
}

void mac_Area2to3_Route(PATH_WAYPOINT *points, ST_VEL *start_vel)
{
    uint8_t cur_region = area2_divide(&points[0].pos);
    points[0].vel = *start_vel;
    if (cur_region & LEFT_REGION)
    {
        points[3] = uphill_point[0];
        points[3].pos.fpPosQ = -90;
        The_Fastest_Path(&points[0], A_MAX, STRAIGHT_VEL);
        memcpy(&points[6], &uphill_point[1], 2 * sizeof(PATH_WAYPOINT));
        points[6].pos.fpPosQ = -90;
        The_Fastest_Path(&points[0], UPHILL_ACC, UPHILL_SPEED);
        points[4].pos.fpPosQ = -90;
    }
    else if (cur_region & BACK_REGION)
    {
        points[4] = uphill_point[0];
        points[4].pos.fpPosQ = 90;
        Dubins_Path(&points[0], &points[3], &points[4], 450, 1300);
        The_Fastest_Path(&points[0], A_MAX, STRAIGHT_VEL);
        memcpy(&points[7], &uphill_point[1], 2 * sizeof(PATH_WAYPOINT));
        points[7].pos.fpPosQ = 90;
        The_Fastest_Path(&points[4], UPHILL_ACC, UPHILL_SPEED);
        points[8].pos.fpPosQ = 90;
        yaw_plan_4points(&points[1]);
    }
    else if (cur_region & FRONT_REGION)
    {
        points[4] = arcspot_choose(FRONT_REGION, LEFT_REGION, CORNER_VEL);
        Dubins_Path(&points[0], &points[3], &points[4], CORNER_R, A_MAX);
        points[3].pos.fpPosQ = 0;
        points[4].pos.fpPosQ = 0;
        points[7] = uphill_point[0];
        memcpy(&points[10], &uphill_point[1], 2 * sizeof(PATH_WAYPOINT));
        points[7].pos.fpPosQ = 0;
        points[10].pos.fpPosQ = 0;
        The_Fastest_Path(&points[0], A_MAX, STRAIGHT_VEL);
        The_Fastest_Path(&points[4], A_MAX, STRAIGHT_VEL);
        The_Fastest_Path(&points[7], UPHILL_ACC, UPHILL_SPEED);
    }
    else if (cur_region & RIGHT_REGION)
    {
        points[4] = arcspot_choose(RIGHT_REGION, BACK_REGION, CORNER_VEL);
        Dubins_Path(&points[0], &points[3], &points[4], CORNER_R, A_MAX);
        points[4].pos.fpPosQ = 90;
        points[8] = uphill_point[0];
        Dubins_Path(&points[4], &points[7], &points[8], 450, 1300);
        points[7].pos.fpPosQ = 90;
        points[8].pos.fpPosQ = 90;
        memcpy(&points[11], &uphill_point[1], 2 * sizeof(PATH_WAYPOINT));
        points[11].pos.fpPosQ = 90;
        points[12].pos.fpPosQ = 90;
        The_Fastest_Path(&points[0], A_MAX, STRAIGHT_VEL);
        The_Fastest_Path(&points[4], A_MAX, STRAIGHT_VEL);
        The_Fastest_Path(&points[8], UPHILL_ACC, UPHILL_SPEED);
    }
}

void mac_Area3_Route(PATH_WAYPOINT *points, uint16_t des_spot, ST_VEL *start_vel)//已完成蓝区转红区
{
    if (des_spot >> 4 != 0x300 || (des_spot & 0x0f) > 6)
        des_spot = 0x3003;

    const float add_dist = 100.f;

    ST_POS des_pos = {0};
    area3_spot_choose(&des_pos, des_spot);
    points[0].vel = *start_vel;
    points[5].pos = des_pos;
    points[4].pos = des_pos;
    points[4].pos.fpPosX = des_pos.fpPosX + 400.f;
    points[4].vel.fpVx = -1400;
    points[0].pos.fpPosX -= add_dist;
    Dubins_Path(&points[0], &points[3], &points[4], 1000, AREA3_ACC);
    points[3].pos.fpPosQ = des_pos.fpPosQ;
    The_Fastest_Path(&points[0], AREA3_ACC, AREA3_VEL);
    points[1].time += add_dist / 3000.f;
    points[0].pos.fpPosX += add_dist;
    points[2].pos.fpPosQ = des_pos.fpPosQ;
    points[2].vel.fpW = 0;
    yaw_plan_4points(&points[0]);
    Path_time_fill(&points[4], &points[5], 1.f);
}

uint8_t ring_map(uint16_t spot)
{ // 环形映射，1,2,3即位于1,2,3格子的前方，0~17
    uint8_t col = spot & 0x0f;
    uint8_t row = (spot >> 4) & 0x0f;
    if (row == 0)
        return col;
    else if (col == 4)
        return row + 4;
    else if (row == 5)
        return 13 - col;
    else // col==0
        return 18 - row;
}

int8_t dist_judge(uint16_t start_spot, uint16_t end_spot)
{
    int8_t diff = ring_map(end_spot) - ring_map(start_spot);
    uint8_t diff_pstv = (diff + 18) % 18;
    if (diff_pstv <= 8)
        return diff_pstv;
    else
        return diff_pstv - 18;
}

void mac_dist_lengthen(ST_POS *pos, ST_VEL *mac_vel)
{
    if (mac_vel->fpVx > 1e-4f)
        pos->fpPosX -= MAC_DIST;
    else if (mac_vel->fpVx < -1e-4f)
        pos->fpPosX += MAC_DIST;
    else if (mac_vel->fpVy > 1e-4f)
        pos->fpPosY -= MAC_DIST;
    else if (mac_vel->fpVy < -1e-4f)
        pos->fpPosY += MAC_DIST;
}

uint8_t corner_slowdown_judge(uint16_t des_spot, ST_VEL *mac_vel)
{
    if (mac_vel->fpVx > 1e-4f)
        return des_spot == 0x2001 || des_spot == 0x2051;
    else if (mac_vel->fpVx < -1e-4f)
        return des_spot == 0x2003 || des_spot == 0x2053;
    else if (mac_vel->fpVy > 1e-4f)
        return des_spot == 0x2010 || des_spot == 0x2014;
    else if (mac_vel->fpVy < -1e-4f)
        return des_spot == 0x2040 || des_spot == 0x2044;
    else
        return 0;
}

void mac_area2_oppo_route(PATH_WAYPOINT *points, uint8_t region_start, uint8_t region_pass, uint8_t region_end, ST_POS *des_pos, ST_VEL *mac_vel, float corner_end_vel, float line_end_vel)
{ // corner_end_vel是绝对值
    points[4] = arcspot_choose(region_start, region_pass, CORNER_VEL);
    Dubins_Path(&points[0], &points[3], &points[4], CORNER_R, A_MAX);
    points[8] = arcspot_choose(region_pass, region_end, corner_end_vel);
    Dubins_Path(&points[4], &points[7], &points[8], CORNER_R, A_MAX);
    points[11].pos = *des_pos;
    points[11].vel = *mac_vel;
    The_Fastest_Path(&points[0], A_MAX, STRAIGHT_VEL);
    The_Fastest_Path(&points[4], A_MAX, STRAIGHT_VEL);
    The_Fastest_Path(&points[8], A_MAX, line_end_vel);
    yaw_plan_4points(&points[2]);
    yaw_plan_4points(&points[6]);
}
