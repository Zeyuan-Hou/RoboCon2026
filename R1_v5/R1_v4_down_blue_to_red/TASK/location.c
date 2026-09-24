#include "location.h"
#include <math.h>   // 确保引入了 math.h
#include <stdlib.h> // 确保引入了 stdlib.h (为了 abs 函数)

#define DT35_FRONT_VERTICAL 300
#define DT35_FRONT_HORIZONTAL 0
#define DT35_BACK_VERTICAL 0 // 300是因为dt35标定是从车身开始测
#define DT35_BACK_HORIZONTAL 0
#define DT35_LEFT_VERTICAL 300
#define DT35_LEFT_HORIZONTAL -12.5f
#define DT35_RIGHT_VERTICAL 300
#define DT35_RIGHT_HORIZONTAL 12.5f

#define FRONT_ANGLE 0
#define RIGHT_ANGLE -90
#define LEFT_ANGLE 90
#define BACK_ANGLE 180

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.01745329252f
#endif

float dt35_front_dist = 0;
float dt35_back_dist = 0;
float dt35_left_dist = 0;
float dt35_right_dist = 0;

uint8_t dt35_flag = 0;

float vis_x;
float vis_y;
float vis_yaw;

ST_POS pre_vis_data;
uint8_t vis_init = 0;

//2号红场
const float vis_arc_position[4][2] = { // 3,1,10,12
    {-79.922, -2674.101},
    {-3670.921, -2692.467},
    {-3648.735, -7488.210},
    {-58.845, -7464.918}
};

		
		
KF_history_t KF_history_quene[DELAYED_KF_SIZE]; // 保存历史数据
uint16_t KF_history_pos;

uint8_t communication_delay = 0; // 通信延迟，换了贵的USB转ttl之后来回在10ms左右，单向通信延迟可以认为是0

uint8_t vision_transfer_flag = 0; // 0是2区，1是3区

void Location_dt35_vision(void)
{
    // bodyvel
    ST_VEL local_wheeltobody_vel = {0};
    if (g_stuck_cnt <= 10)
    {
        SteerWheels_solve_withGyro(&Chassis.wheels, &local_wheeltobody_vel, &wheeltobody_vel_residualSSE, g_omega);
    }
    else
    {
        SteerWheels_solve(&Chassis.wheels, &local_wheeltobody_vel, &wheeltobody_vel_residualSSE);
    }
    GlobalVel_To_Local(&wheeltobody_vel, &local_wheeltobody_vel, -location.fpPosQ);
    if (Vision.pos.fpPosX > 4700 && Vision.pos.fpPosY > 9300 && Vision.pos.fpPosY < 10800)
        wheeltobody_vel.fpVy *= 0.9625f;
    if (vis_init == 1)
    {
        PosKF_Predict(&PosKF, &wheeltobody_vel, wheeltobody_vel_residualSSE);
        Delayed_PosKF_Input(&wheeltobody_vel, wheeltobody_vel_residualSSE);
    }

    // gyro
    g_update_flag = 0;
    if (g_uart_rx_cnt > 0)
    {
        if (g_uart_rx_cnt + g_decode_data_pos > 512)
            g_uart_rx_cnt = 512 - g_decode_data_pos;
        memcpy(g_decode_data + g_decode_data_pos, g_uart_rx_buf, g_uart_rx_cnt);
        g_decode_data_pos += g_uart_rx_cnt;
        g_uart_rx_cnt = 0;
    }

    if (g_decode_data_pos > 0)
    {
        analysis_data(g_decode_data, g_decode_data_pos);
    }
    if (vis_init == 1 && g_stuck_cnt <= 10 && g_update_flag == YAW_UPDATE)
    {
        KF_history_quene[KF_history_pos].pos.fpPosQ = KF_history_quene[(KF_history_pos + 5) % DELAYED_KF_SIZE ].pos.fpPosQ + norm_angle(g_yaw - g_pre_yaw);
        KF_history_quene[KF_history_pos].P[2] = 0.01f;
    }
    g_update_flag = 0;

    // vision
    if (Vision_Data.radar_x != pre_vis_data.fpPosX ||
        Vision_Data.radar_y != pre_vis_data.fpPosY ||
        Vision_Data.radar_yaw != pre_vis_data.fpPosQ)
    {
        sys_mnt.cnt.radar_rx++;
        if (sys_mnt.cnt.radar_rx > 25 && vis_init == 0)
        {
            Vision_init_homography(&H_matrix,
                                   vis_arc_position[0][0], vis_arc_position[0][1],
                                   vis_arc_position[1][0], vis_arc_position[1][1],
                                   vis_arc_position[2][0], vis_arc_position[2][1],
                                   vis_arc_position[3][0], vis_arc_position[3][1]);
            Vision_transfer_of_axes(&H_matrix, Vision_Data.radar_x, Vision_Data.radar_y, &Vision.pos.fpPosX, &Vision.pos.fpPosY);
            Vision.pos.fpPosQ = norm_angle(Vision_Data.radar_yaw * RAD_TO_DEG - 180);
            PosKF.pos[0] = Vision.pos.fpPosX;
            PosKF.pos[1] = Vision.pos.fpPosY;
            PosKF.pos[2] = Vision.pos.fpPosQ;

            Delayed_PosKF_Init(&Vision.pos);
            vis_init = 1;
        }
        if (vis_init == 1)
        {
            Vision_transfer_of_axes(&H_matrix, Vision_Data.radar_x, Vision_Data.radar_y, &Vision.pos.fpPosX, &Vision.pos.fpPosY);
            Vision.pos.fpPosQ = norm_angle(Vision_Data.radar_yaw * RAD_TO_DEG - 180);
            PosKF_Update(&PosKF, &Vision.pos);
            Delayed_PosKF_Update(&Vision.pos, Vision_Data.radar_delay + communication_delay);

            sys_mnt.cnt.posKF_update++;
        }
    }

    ST_POS dkf_pos = KF_history_quene[KF_history_pos].pos;

    vis_x = Vision.pos.fpPosX;
    vis_y = Vision.pos.fpPosY;
    vis_yaw = Vision.pos.fpPosQ;

    pre_vis_data.fpPosX = Vision_Data.radar_x;
    pre_vis_data.fpPosY = Vision_Data.radar_y;
    pre_vis_data.fpPosQ = Vision_Data.radar_yaw;

    // dt35
    ST_POS pos = KF_history_quene[KF_history_pos].pos;
    //ST_POS pos = {PosKF.pos[0], PosKF.pos[1], PosKF.pos[2]};
    Dt35_position(&pos);
    if (fabsf(pos.fpPosX - PosKF.pos[0]) > 200)
        pos.fpPosX = PosKF.pos[0];
    if (fabsf(pos.fpPosY - PosKF.pos[1]) > 200)
        pos.fpPosY = PosKF.pos[1];

    location = pos;

    ST_POS pos_to_upper = dkf_pos;
    onlydt35_position(&pos_to_upper);
//    if (fabsf(pos_to_upper.fpPosX - dkf_pos.fpPosX) > 200)
//        pos_to_upper.fpPosX = dkf_pos.fpPosX;
//    if (fabsf(pos_to_upper.fpPosY - dkf_pos.fpPosY) > 200)
//        pos_to_upper.fpPosY = dkf_pos.fpPosY;

    location_to_upper = pos_to_upper;
}

void Dt35_position(ST_POS *pos)
{
    if (range_judge(2700, 4200, 300, 2000, RIGHT_ANGLE) && (Nav.cur_spot >> 12 == 0x01 && (Nav.cur_spot & 0x0f) < 5))
    {
        Dt35_Calc(RIGHT_ANGLE);
        pos->fpPosX = 6000 - dt35_front_dist;
        pos->fpPosY = 3200 - dt35_left_dist;
    }
    else if (range_judge(400, 1300, 9900, 11510, FRONT_ANGLE) && (Nav.cur_spot == 0x3011 || Nav.cur_spot == 0x3012 || Nav.cur_spot == 0x3018))
    {
        Dt35_Calc(FRONT_ANGLE);
        pos->fpPosX = 135 + dt35_left_dist; // 12045
        pos->fpPosY = 12000 - dt35_front_dist;
    }
    else if (range_judge(400, 1300, 9900, 11510, BACK_ANGLE) && (Nav.cur_spot == 0x3015 || Nav.cur_spot == 0x3016 || Nav.cur_spot == 0x3021))
    {
        Dt35_Calc(BACK_ANGLE);
        pos->fpPosX = 135 + dt35_right_dist; // 9535
        pos->fpPosY = 9500 + dt35_front_dist;
    }
    else if (range_judge(400, 1000, 9900, 11510, FRONT_ANGLE) && (Nav.cur_spot == 0x3002 || Nav.cur_spot == 0x3001))
    {
        Dt35_Calc(FRONT_ANGLE);
				pos->fpPosX = 135 + dt35_left_dist;
        pos->fpPosY = 12000 - dt35_front_dist;
    }
    else if (range_judge(400, 1000, 9900, 11510, BACK_ANGLE) && (Nav.cur_spot == 0x3005 || Nav.cur_spot == 0x3006))
    {
        Dt35_Calc(BACK_ANGLE);
				pos->fpPosX = 135 + dt35_right_dist;
        pos->fpPosY = 9500 + dt35_front_dist;
    }
    else if (range_judge(400, 1000, 9500, 12000, FRONT_ANGLE) && (Nav.cur_spot == 0x3003 || Nav.cur_spot == 0x3041 || Nav.cur_spot == 0x3017))
    {
        Dt35_Calc(FRONT_ANGLE);
        pos->fpPosY = 12000 - dt35_front_dist;
    }
    else if (range_judge(400, 1000, 9500, 12000, BACK_ANGLE) && (Nav.cur_spot == 0x3004 || Nav.cur_spot == 0x3022))
    {
        Dt35_Calc(BACK_ANGLE);
        pos->fpPosY = 9500 + dt35_front_dist;
    }
}

void onlydt35_position(ST_POS *pos)
{
    if (range_judge(2700, 4200, 300, 2000, RIGHT_ANGLE))
    {
        Dt35_Calc(RIGHT_ANGLE);
        pos->fpPosX = 6000 - dt35_front_dist;
        pos->fpPosY = 3200 - dt35_left_dist;
    }
    else if (range_judge(400, 1300, 9990, 11510, FRONT_ANGLE))
    {
        Dt35_Calc(FRONT_ANGLE);
        pos->fpPosX = 135 + dt35_left_dist; // 12045
        pos->fpPosY = 12000 - dt35_front_dist;
    }
    else if (range_judge(400, 1300, 9990, 11510, BACK_ANGLE))
    {
        Dt35_Calc(BACK_ANGLE);
        pos->fpPosX = 135 + dt35_right_dist; // 9535
        pos->fpPosY = 9500 + dt35_front_dist;
    }
}

void Dt35_Calc(float des)
{
    // 修复：由于三角函数需要弧度，这里将角度转换为弧度
    float delta_rad = (des - vis_yaw) * DEG_TO_RAD;

    float cos_d = cosf(delta_rad);
    float sin_d = sinf(delta_rad);

    dt35_front_dist = (QD.dt35_front + DT35_FRONT_VERTICAL) * cos_d + DT35_FRONT_HORIZONTAL * sin_d;
    dt35_back_dist = (QD.dt35_back + DT35_BACK_VERTICAL) * cos_d + DT35_BACK_HORIZONTAL * sin_d;
    dt35_left_dist = (QD.dt35_left + DT35_LEFT_VERTICAL) * cos_d + DT35_LEFT_HORIZONTAL * sin_d;
    dt35_right_dist = (QD.dt35_right + DT35_RIGHT_VERTICAL) * cos_d + DT35_RIGHT_HORIZONTAL * sin_d;
}

uint8_t angle_judge(float des)
{
    return fabsf(norm_angle(vis_yaw - des)) < 5;
}

uint8_t range_judge(float x_min, float x_max, float y_min, float y_max, float yaw_des)
{
    uint8_t ret = 1;
    ret &= vis_x > x_min && vis_x < x_max;
    ret &= vis_y > y_min && vis_y < y_max;
    ret &= fabsf(norm_angle(yaw_des - vis_yaw)) < 5;
    return ret;
}

void PosKF_Predict(PosKF_t *kf, ST_VEL *cur_vel, float residual_sse)
{
    const float dt = 1e-3f;

    kf->vel[0] = cur_vel->fpVx;
    kf->vel[1] = cur_vel->fpVy;
    kf->vel[2] = cur_vel->fpW;

    float Q_adapt_x = kf->Q_base[0] + kf->Q_adapt * residual_sse;
    float Q_adapt_y = kf->Q_base[1] + kf->Q_adapt * residual_sse;
    float Q_adapt_yaw = kf->Q_base[2] + kf->Q_adapt * residual_sse;
    // 残差平方和的单位是m^2/s^2，因此这里本来应该再乘一个系数dt*dt = 1e-6f，但由于我调整精度的时候已经放缩过了，这里就没有
    kf->pos[0] += cur_vel->fpVx * dt;
    kf->pos[1] += cur_vel->fpVy * dt;
    kf->pos[2] = norm_angle(kf->pos[2] + cur_vel->fpW * dt);

    kf->P[0] += Q_adapt_x;
    kf->P[1] += Q_adapt_y;
    kf->P[2] += Q_adapt_yaw;
}

void PosKF_Update(PosKF_t *kf, ST_POS *obs_pos)
{
    kf->radar[0] = obs_pos->fpPosX;
    kf->radar[1] = obs_pos->fpPosY;
    kf->radar[2] = obs_pos->fpPosQ;

    for (int i = 0; i < 3; i++)
    {
        // 计算创新值 (Innovation)
        float innovation = 0;
        float K = kf->P[i] / (kf->P[i] + kf->R_obs[i]);
        if (i == 2)
        { // 角度需要特殊处理差值
            innovation = norm_angle(kf->radar[i] - kf->pos[i]);
            kf->pos[i] += norm_angle(K * innovation);
        }
        else
        {
            innovation = kf->radar[i] - kf->pos[i];
            kf->pos[i] += K * innovation;
        }
        // 更新协方差 P = (1 - K)P
        kf->P[i] *= (1.0f - K);
    }
}
// 历史回溯卡尔曼
void Delayed_PosKF_Init(ST_POS *pos)
{
    for (uint8_t i = 0; i < DELAYED_KF_SIZE; i++)
    {
        KF_history_quene[i].P[0] = 1;
        KF_history_quene[i].P[1] = 1;
        KF_history_quene[i].P[2] = 1;
        KF_history_quene[i].pos = *pos;
    }
}

void Delayed_PosKF_Input(ST_VEL *cur_vel, float residualSSE)
{
    if (KF_history_pos == 0)
        KF_history_pos = DELAYED_KF_SIZE - 1;
    else
        KF_history_pos--;

    KF_history_quene[KF_history_pos].vel = *cur_vel;
    KF_history_quene[KF_history_pos].residualSSE = residualSSE;
    Delayed_PosKF_Predict(0);
}

const float vel_Q_base[3] = {0.05f, 0.05f, 0.1f};
const float vel_Q_adapt = 0.1f;
const float radar_Q[3] = {25.f, 25.f, 0.01f};

void Delayed_PosKF_Predict(uint16_t cur_delay)
{ // 输入速度之后由上一ms的位置预测出当前的位置
    const float dt = 1e-3f;
    if (cur_delay >= DELAYED_KF_SIZE - 1)
        cur_delay = DELAYED_KF_SIZE - 2; // 最大98ms延迟，这是为了让出前一个数组位置，防止数组更新顺序错乱

    uint16_t cur_index = (cur_delay + KF_history_pos) % DELAYED_KF_SIZE;
    uint16_t last_index = (cur_index + 1) % DELAYED_KF_SIZE;

    ST_POS last_pos = KF_history_quene[last_index].pos;
    float *last_P = KF_history_quene[last_index].P;
    ST_VEL vel = KF_history_quene[cur_index].vel;
    float residualSSE = KF_history_quene[cur_index].residualSSE;

    KF_history_quene[cur_index].pos.fpPosX = last_pos.fpPosX + vel.fpVx * dt;
    KF_history_quene[cur_index].pos.fpPosY = last_pos.fpPosY + vel.fpVy * dt;
    KF_history_quene[cur_index].pos.fpPosQ = norm_angle(last_pos.fpPosQ + vel.fpW * dt);

    KF_history_quene[cur_index].P[0] = last_P[0] + vel_Q_base[0] + vel_Q_adapt * residualSSE;
    KF_history_quene[cur_index].P[1] = last_P[1] + vel_Q_base[1] + vel_Q_adapt * residualSSE;
    KF_history_quene[cur_index].P[2] = last_P[2] + vel_Q_base[2] + vel_Q_adapt * residualSSE;
}

void Delayed_PosKF_Update(ST_POS *radar, uint8_t delayed_ticks)
{
    if (delayed_ticks >= DELAYED_KF_SIZE - 1)
        delayed_ticks = DELAYED_KF_SIZE - 2;
    ST_POS innovation = {0};
    float K[3] = {0};
    uint16_t delay_index = (delayed_ticks + KF_history_pos) % DELAYED_KF_SIZE;
    KF_history_t *kf = &KF_history_quene[delay_index];
    innovation.fpPosX = radar->fpPosX - kf->pos.fpPosX;
    innovation.fpPosY = radar->fpPosY - kf->pos.fpPosY;
    innovation.fpPosQ = norm_angle(radar->fpPosQ - kf->pos.fpPosQ);
    for (uint8_t i = 0; i < 3; i++)
    {
        if (kf->P[i] < 1e-3f)
            kf->P[i] = 1e-3f;
        K[i] = kf->P[i] / (kf->P[i] + radar_Q[i]);
        kf->P[i] *= 1.f - K[i];
    }
    kf->pos.fpPosX += K[0] * innovation.fpPosX;
    kf->pos.fpPosY += K[1] * innovation.fpPosY;
    kf->pos.fpPosQ = norm_angle(kf->pos.fpPosQ + K[2] * innovation.fpPosQ);
    while (delayed_ticks > 0)
    {
        delayed_ticks--;
        Delayed_PosKF_Predict(delayed_ticks);
    }
}
