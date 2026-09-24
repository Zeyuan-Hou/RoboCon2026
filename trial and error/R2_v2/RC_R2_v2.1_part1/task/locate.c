#include "locate.h"

void Vision_Data_Deal(VISION_DATA_AUTO *p_vision_data)
{
    memcpy(&p_vision_data->x1, &vision_rec[1], 4);
    memcpy(&p_vision_data->y1, &vision_rec[5], 4);
    memcpy(&p_vision_data->z1, &vision_rec[9], 4);
    memcpy(&p_vision_data->yaw1, &vision_rec[13], 4);

    memcpy(&p_vision_data->x2, &vision_rec[17], 4);
    memcpy(&p_vision_data->y2, &vision_rec[21], 4);
    memcpy(&p_vision_data->z2, &vision_rec[25], 4);
    memcpy(&p_vision_data->yaw2, &vision_rec[29], 4);
    Vision_location(&nav);
}

void Vision_location(ST_Nav *p_nav)
{
    if (location_filter.flag)
    {
        LocationFilter();
        location_filter.flag = 0;
    }
    p_nav->auto_path.pos_pid.x.fpFB = stRobot.stPos.fpPosX;
    p_nav->auto_path.pos_pid.y.fpFB = stRobot.stPos.fpPosY;
    p_nav->auto_path.pos_pid.w.fpFB = stRobot.stPos.fpPosQ / 10.f;
}

void LocationFilter()
{
    location_filter.x[0] = vision_data_recieve.x1;
    location_filter.y[0] = vision_data_recieve.y1;
    location_filter.yaw[0] = vision_data_recieve.yaw1;
    for (int i = 4; i > 0; i--)
    {
        location_filter.x[i] = location_filter.x[i - 1] + location_filter.lpf_k * (vision_data_recieve.x1 - location_filter.x[i - 1]);
        location_filter.y[i] = location_filter.y[i - 1] + location_filter.lpf_k * (vision_data_recieve.y1 - location_filter.y[i - 1]);
        location_filter.yaw[i] = location_filter.yaw[i - 1] + location_filter.lpf_k * (vision_data_recieve.yaw1 - location_filter.yaw[i - 1]);
    }

    float sum_x = 0.f, sum_y = 0.f, sum_yaw = 0.f;
    for (int i = 0; i < 5; i++)
    {
        sum_x += location_filter.x[i];
        sum_y += location_filter.y[i];
        sum_yaw += location_filter.yaw[i];
    }

    // 坐标赋值
    stRobot.stPos.fpPosX = sum_x / 5.f;
    stRobot.stPos.fpPosY = sum_y / 5.f;
    stRobot.stPos.fpPosQ = -sum_yaw / 5.f * 1800 / PI; // 转为0.1度
}
