#include "path_algos.h"

// 路径插值法
PATH_POINTS Path_Points = {0}; // 注意这是一个公用的结构体，在主程序中需要初始化

void Cubic_Hermite_Spline(CUBIC_SPLINE *spline) // 三次埃尔米特插值
{
    if (spline->init_flag == 0)
    { // 1. 初始化系数 (只运行一次)
        if (spline->T < 1e-4f)
            return; // 防止除以0，增加安全性

        float T = spline->T;
        float T2 = T * T;
        float T3 = T2 * T;

        spline->a = (2.0f * (spline->p_start - spline->p_end) + T * (spline->v_end + spline->v_start)) / T3;
        spline->b = (3.0f * (spline->p_end - spline->p_start) - T * (spline->v_end + 2.0f * spline->v_start)) / T2;

        spline->init_flag = 1;
    }
    float dt = (float)HAL_GetTick() / 1000.f - spline->t_start; // 2. 计算相对时间 dt

    if (dt < 0.0f)
        dt = 0.0f; // 3. 边界处理 (Clamping) - 非常重要！
    else if (dt > spline->T)
        dt = spline->T;

    float dt2 = dt * dt; // 4. 曲线计算,即使是初始化后的第一帧，也应该根据 dt 进行计算，而不是直接赋值 p_start
    float dt3 = dt2 * dt;

    spline->p = spline->a * dt3 + spline->b * dt2 + spline->v_start * dt + spline->p_start;
    spline->v = 3.0f * spline->a * dt2 + 2.0f * spline->b * dt + spline->v_start;
}



void Quintic_Hermite_Spline(QUINTIC_SPLINE *spline) // 五次埃尔米特插值
{
    if (spline->init_flag == 0)
    { // 1. 初始化系数 (只运行一次)
        if (spline->T < 1e-4f)
            return; // 防止除以0，增加安全性

        float T = spline->T;
        float T2 = T * T;
        float T3 = T2 * T;
        float T4 = T3 * T;
        float T5 = T4 * T;

        // 五次多项式系数计算公式（已化简）
        spline->a = (6.0f * (spline->p_end - spline->p_start) - 3.0f * T * (spline->v_end + spline->v_start) + 0.5f * T2 * (spline->acc_end - spline->acc_start)) / T5;
        spline->b = (-15.0f * (spline->p_end - spline->p_start) + T * (7.0f * spline->v_end + 8.0f * spline->v_start) - T2 * (spline->acc_end - 1.5f * spline->acc_start)) / T4;
        spline->c = (10.0f * (spline->p_end - spline->p_start) - T * (4.0f * spline->v_end + 6.0f * spline->v_start) + T2 * (0.5f * spline->acc_end - 1.5f * spline->acc_start)) / T3;

        spline->init_flag = 1;
    }
    float dt = (float)HAL_GetTick() / 1000.f - spline->t_start; // 2. 计算相对时间 dt

    if (dt < 0.0f)
        dt = 0.0f; // 3. 边界处理 (Clamping) - 非常重要！
    else if (dt > spline->T)
        dt = spline->T;

    float dt2 = dt * dt; // 4. 曲线计算,即使是初始化后的第一帧，也应该根据 dt 进行计算
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
    float dt5 = dt4 * dt;

    // 计算位置 (p)、速度 (v) 和加速度 (acc)
    spline->p = spline->a * dt5 + spline->b * dt4 + spline->c * dt3 + 0.5f * spline->acc_start * dt2 + spline->v_start * dt + spline->p_start;
    spline->v = 5.0f * spline->a * dt4 + 4.0f * spline->b * dt3 + 3.0f * spline->c * dt2 + spline->acc_start * dt + spline->v_start;
    spline->acc = 20.0f * spline->a * dt3 + 12.0f * spline->b * dt2 + 6.0f * spline->c * dt + spline->acc_start;
}


void Spline_Init(PATH_WAYPOINT *start, PATH_WAYPOINT *end, CUBIC_SPLINE *spline_Vx, CUBIC_SPLINE *spline_Vy, CUBIC_SPLINE *spline_W, float t_start) // t_start单位是秒
{
    spline_Vx->p_start = start->pos.fpPosX;
    spline_Vx->p_end = end->pos.fpPosX;
    spline_Vx->v_start = start->vel.fpVx;
    spline_Vx->v_end = end->vel.fpVx;
    spline_Vx->t_start = t_start;
    spline_Vx->T = end->time;
    spline_Vx->init_flag = 0;

    spline_Vy->p_start = start->pos.fpPosY;
    spline_Vy->p_end = end->pos.fpPosY;
    spline_Vy->v_start = start->vel.fpVy;
    spline_Vy->v_end = end->vel.fpVy;
    spline_Vy->t_start = t_start;
    spline_Vy->T = end->time;
    spline_Vy->init_flag = 0;

    spline_W->p_start = start->pos.fpPosQ;
    spline_W->p_end = start->pos.fpPosQ + norm_angle(end->pos.fpPosQ - start->pos.fpPosQ); // 这里的角度是有可能超过[-1800,1800]的，但Path_Spline_Process会自动将其归一化到[-1800,1800]之间
    spline_W->v_start = start->vel.fpW;                                                    // 注意归一化后速度不要反
    spline_W->v_end = end->vel.fpW;
//    spline_W->acc_start = start->acc_yaw;
//    spline_W->acc_end = end->acc_yaw;
    spline_W->t_start = t_start;
    spline_W->T = end->time;
    spline_W->init_flag = 0;
}

void Spline_Process(CUBIC_SPLINE *spline_Vx, CUBIC_SPLINE *spline_Vy, CUBIC_SPLINE *spline_W, PATH_OBJECT *object)
{
    Cubic_Hermite_Spline(spline_Vx);
    Cubic_Hermite_Spline(spline_Vy);
    Cubic_Hermite_Spline(spline_W);

    object->pos.fpPosX = spline_Vx->p;
    object->pos.fpPosY = spline_Vy->p;
    object->pos.fpPosQ = norm_angle(spline_W->p);

    object->vel.fpVx = spline_Vx->v;
    object->vel.fpVy = spline_Vy->v;
    object->vel.fpW = spline_W->v;
}

uint8_t Path_isEnd(void)
{ // 查询当前是否结束
    if (Path_Points.point_pos >= Path_Points.point_num)
        return 1;
    else
        return 0;
}

void Path_Reset(void) // 重置路径
{
    memset(&Path_Points, 0, sizeof(PATH_POINTS));
}

void Path_Spline(PATH_OBJECT *object) // object应输出物体的期望状态，time单位是秒
{
    float time = (float)HAL_GetTick() / 1000.0f; // 获取当前时间
    if (Path_Points.init_flag == 0)
    {
        Path_Points.t_start = time;
        Path_Points.point_t_end = Path_Points.t_start + Path_Points.point[Path_Points.point_pos].time;
        Path_Points.init_flag = 1;
        Spline_Init(&Path_Points.point[Path_Points.point_pos - 1], &Path_Points.point[Path_Points.point_pos], &Path_Points.spline_Vx, &Path_Points.spline_Vy, &Path_Points.spline_W, Path_Points.t_start); // 给第一个点初始化
    } // 初始化：记录起始时间

    if (Path_isEnd())
        return; // 如果已经到达最后一个点，则停止

    while (time >= Path_Points.point_t_end)
    { // 检查当前路径片段是否已经完成，如果完成则切换到下一个片段
        object->pos = Path_Points.point[Path_Points.point_pos].pos;
        object->vel = Path_Points.point[Path_Points.point_pos].vel;
        Path_Points.point_pos++;
        if (Path_isEnd())
        {
            object->pos = Path_Points.point[Path_Points.point_pos - 1].pos;
            object->vel = Path_Points.point[Path_Points.point_pos - 1].vel;
            memset(Path_Points.point, 0, WAYPOINT_SIZE * sizeof(PATH_WAYPOINT));
            Path_Points.progress = 1;
            Path_Points.t_remain = 0;
            return; // 如果已经到达最后一个点，则停止
        }
        Spline_Init(&Path_Points.point[Path_Points.point_pos - 1], &Path_Points.point[Path_Points.point_pos], &Path_Points.spline_Vx, &Path_Points.spline_Vy, &Path_Points.spline_W, Path_Points.point_t_end); // 给下一个点初始化
        Path_Points.point_t_end += Path_Points.point[Path_Points.point_pos].time;
    }

    Path_Points.progress = (time - Path_Points.t_start) / Path_Points.t_sum;
    Path_Points.t_remain = Path_Points.t_start + Path_Points.t_sum - time;
    Spline_Process(&Path_Points.spline_Vx, &Path_Points.spline_Vy, &Path_Points.spline_W, object);
}

void Path_Init(ST_POS *start_pos, ST_POS *final_pos) // 输入整段路径的起始点（通常为当前位置），输出终点
{
    Path_Points.point[0].pos = *start_pos;
    Path_Points.init_flag = 0;
    Path_Points.point_pos = 1; // 准备初始化，并将目标点设为第一个点
    Path_Points.t_sum = 0;
    Path_Points.point_num = 1; // 至少一个点
    Path_Points.progress = 0;
    for (uint8_t i = 1; i < WAYPOINT_SIZE; i++)
    {
        if (Path_Points.point[i].time < 1e-4f &&
            Path_Points.point[i].vel.fpVx < 1e-4f &&
            Path_Points.point[i].vel.fpVy < 1e-4f &&
            Path_Points.point[i].vel.fpW < 1e-4f &&
            Path_Points.point[i].pos.fpPosX < 1e-4f &&
            Path_Points.point[i].pos.fpPosY < 1e-4f &&
            Path_Points.point[i].pos.fpPosQ < 1e-4f)
        {
            *final_pos = Path_Points.point[i - 1].pos;
            break; // 遇到时间为0的点，说明路径结束，跳出循环
        }
        Path_Points.t_sum += Path_Points.point[i].time; // 第1个点时间为0，但也统计了
        Path_Points.point_num++;
    }
    if (Path_Points.t_sum < 1e-4f)
        Path_Points.t_sum = 1; // 防止除0错误
} // 注意应从数组第二个开始赋值

void Path_time_fill(PATH_WAYPOINT *start, PATH_WAYPOINT *end, float k) // 自动补全终点时间,仅适用于起点速度方向和终点相差不大的情况
{
    float dist = hypotf(end->pos.fpPosX - start->pos.fpPosX, end->pos.fpPosY - start->pos.fpPosY);
    float avg_vel = (hypotf(start->vel.fpVx, start->vel.fpVy) + hypotf(end->vel.fpVx, end->vel.fpVy)) / 2.0f * k;
    if (avg_vel < 1)
        avg_vel = 1;
    end->time = dist / avg_vel;
}

void Path_vel_fill(PATH_WAYPOINT *start, PATH_WAYPOINT *pass, PATH_WAYPOINT *end, float speed){
    float dist = hypotf(end->pos.fpPosX - start->pos.fpPosX, end->pos.fpPosY - start->pos.fpPosY);
    pass->vel.fpVx = speed * (end->pos.fpPosX - start->pos.fpPosX)/dist;
    pass->vel.fpVy = speed * (end->pos.fpPosY - start->pos.fpPosY)/dist;
}
/**
 * Dubins路径，用于生成先直线再圆弧的路径
 * Dubins路径的三个关键点，分别是起点、拐点和中点
 * 输入：起点的位置，终点的位置，预期半径, 最大加速度(a_max)
 * 输出：拐点的位置，速度，预期时间，终点的预期时间
 * 注意：生成路径时认为起点速度为0，计算最大加速度时不假设
 * 同时此处对于车朝向的处理和速度相同（即视为起点，终点朝向已给出）
 */
void Dubins_Path(PATH_WAYPOINT *start, PATH_WAYPOINT *arc, PATH_WAYPOINT *end, float R, float a_max)
{
    // 1. 提取基础坐标
    float x1 = start->pos.fpPosX;
    float y1 = start->pos.fpPosY;
    float x3 = end->pos.fpPosX;
    float y3 = end->pos.fpPosY;

    // 2. 确定真实的路径运动方向 (基于终点速度向量)
    float target_vel = hypotf(end->vel.fpVx, end->vel.fpVy);
    float path_theta3;

    if (target_vel > 0.1f)
    {
        path_theta3 = atan2f(end->vel.fpVy, end->vel.fpVx);
    }
    else
    {
        path_theta3 = end->pos.fpPosQ * PI / 180.0f;
        target_vel = 1000.0f;
    }

    // 3. 判断起点在运动方向的哪一侧 (利用叉积)
    float cross_product = cosf(path_theta3) * (y1 - y3) - sinf(path_theta3) * (x1 - x3);
    uint8_t is_left_side = (cross_product >= 0.0f);

    float delta_phi = 0.0f;
    float gamma = 0.0f;
    float L_dist = 0.0f;

    if (is_left_side)
    {
        // --- 处理左侧圆 (逆时针) ---
        float x_cL = x3 + R * cosf(path_theta3 + PI_2);
        float y_cL = y3 + R * sinf(path_theta3 + PI_2);
        float D_L = hypotf(x_cL - x1, y_cL - y1);

        if (D_L >= R)
        {
            float alpha_L = atan2f(y_cL - y1, x_cL - x1);
            float beta_L = asinf(R / D_L);
            float L_L = sqrtf(D_L * D_L - R * R);
            gamma = alpha_L - beta_L;
            L_dist = L_L;

            float phi_arc = atan2f(y1 + L_L * sinf(gamma) - y_cL, x1 + L_L * cosf(gamma) - x_cL);
            float phi_end = path_theta3 - PI_2;
            delta_phi = wrap_to_2pi(phi_end - phi_arc);

            arc->pos.fpPosX = x1 + L_L * cosf(gamma);
            arc->pos.fpPosY = y1 + L_L * sinf(gamma);
        }
        else
        {
            gamma = atan2f(y1 - y_cL, x1 - x_cL) + PI_2;
            float phi_arc = atan2f(y1 - y_cL, x1 - x_cL);
            float phi_end = path_theta3 - PI_2;
            delta_phi = wrap_to_2pi(phi_end - phi_arc);

            arc->pos.fpPosX = x1;
            arc->pos.fpPosY = y1;
        }
    }
    else
    {
        // --- 处理右侧圆 (顺时针) ---
        float x_cR = x3 + R * cosf(path_theta3 - PI_2);
        float y_cR = y3 + R * sinf(path_theta3 - PI_2);
        float D_R = hypotf(x_cR - x1, y_cR - y1);

        if (D_R >= R)
        {
            float alpha_R = atan2f(y_cR - y1, x_cR - x1);
            float beta_R = asinf(R / D_R);
            float L_R = sqrtf(D_R * D_R - R * R);
            gamma = alpha_R + beta_R;
            L_dist = L_R;

            float phi_arc = atan2f(y1 + L_R * sinf(gamma) - y_cR, x1 + L_R * cosf(gamma) - x_cR);
            float phi_end = path_theta3 + PI_2;
            delta_phi = wrap_to_2pi(phi_arc - phi_end);

            arc->pos.fpPosX = x1 + L_R * cosf(gamma);
            arc->pos.fpPosY = y1 + L_R * sinf(gamma);
        }
        else
        {
            gamma = atan2f(y1 - y_cR, x1 - x_cR) - PI_2;
            float phi_arc = atan2f(y1 - y_cR, x1 - x_cR);
            float phi_end = path_theta3 + PI_2;
            delta_phi = wrap_to_2pi(phi_arc - phi_end);

            arc->pos.fpPosX = x1;
            arc->pos.fpPosY = y1;
        }
    }

    // 4. 计算加速度与直线段预期时间
    float v0 = hypotf(start->vel.fpVx, start->vel.fpVy);
    if (L_dist > 0.0001f)
    {
        float a_req = (target_vel * target_vel - v0 * v0) / (2.0f * L_dist);
        if (a_req > a_max)
        {
            target_vel = sqrtf(v0 * v0 + 2.0f * a_max * L_dist);
        }
        else if (a_req < -a_max)
        {
            float v_sq = v0 * v0 - 2.0f * a_max * L_dist;
            target_vel = (v_sq > 0.0f) ? sqrtf(v_sq) : 0.001f;
        }
        arc->time = (1.45f * L_dist) / (v0 + target_vel + 1e-6f); // 重点：这里可以改直线段的加速度
    }
    else
    {
        arc->time = 0.0f;
    }

    // 5. 填充拐点和终点的平动速度向量
    arc->vel.fpVx = target_vel * cosf(gamma);
    arc->vel.fpVy = target_vel * sinf(gamma);

    end->vel.fpVx = target_vel * cosf(path_theta3);
    end->vel.fpVy = target_vel * sinf(path_theta3);

    // 6. 按照求出的圆弧包角计算终点预期时间
    end->time = ARC_TIME(R, target_vel) * (delta_phi / PI_2);

    // ---------------------------------------------------------
    // 7. [重写] 底盘自转解算 (姿态角 fpPosQ 与 角速度 fpW)
    // ---------------------------------------------------------

    // 7.1 计算圆弧段的路径转向角 (转换为角度)
    // 左转（逆时针）转向角为正，右转（顺时针）转向角为负
    float turn_angle_deg = (is_left_side ? delta_phi : -delta_phi) * (180.0f / PI);

    // 7.2 强制推算拐点角度 (拐点 = 终点 - 转向角)
    arc->pos.fpPosQ = end->pos.fpPosQ - turn_angle_deg;

    // 7.3 将拐点角度规范化到 -180 到 180 度之间，防止溢出或多圈缠绕
    while (arc->pos.fpPosQ > 180.0f)
        arc->pos.fpPosQ -= 360.0f;
    while (arc->pos.fpPosQ < -180.0f)
        arc->pos.fpPosQ += 360.0f;

    // 7.5 终点角速度清零
    end->vel.fpW = 0.0f;
}

void The_Fastest_Path(PATH_WAYPOINT points[4], float a_max, float v_max)
{
    // 限制最小加速度避免除零
    if (a_max < 1e-4f)
        return;

    float dx = points[3].pos.fpPosX - points[0].pos.fpPosX;
    float dy = points[3].pos.fpPosY - points[0].pos.fpPosY;
    float dist = hypotf(dy, dx);
    float theta = atan2f(dy, dx);

    // 保护：如果距离过短，直接原地生成，避免除零崩溃
    if (dist < 1e-4f)
    {
        points[1] = points[0];
        points[2] = points[0];
        return;
    }

    // 使用速度在路径方向的投影，而不是简单的 hypot，防止横向速度突变导致乱飞
    float v0 = fmaxf(0.0f, points[0].vel.fpVx * cosf(theta) + points[0].vel.fpVy * sinf(theta));
    float v3 = fmaxf(0.0f, points[3].vel.fpVx * cosf(theta) + points[3].vel.fpVy * sinf(theta));

    // 确保理论计算成立
    v_max = fmaxf(v_max, 1e-4f); // 仅限制最小速度防止除零，允许 v_max 小于 v0 或 v3
    a_max = fmaxf(a_max, fabsf(v0 * v0 - v3 * v3) / (2 * dist)); // 物理极限保护

    // 计算如果能达到 v_max 时的两端过渡距离（取绝对值，因为可能是减速过程）
    float l1 = fabsf(v_max * v_max - v0 * v0) / (2 * a_max);
    float l3 = fabsf(v_max * v_max - v3 * v3) / (2 * a_max);
    float l2 = dist - l1 - l3;

    float t1 = 0.0f, t2 = 0.0f, t3 = 0.0f;
    float v_mid = v_max;

    if (l2 >= 0.0f) // 能够达到 v_max (梯形速度曲线 或 双向减速/加速后匀速)
    {
        t1 = fabsf(v_max - v0) / a_max;
        t3 = fabsf(v_max - v3) / a_max;
        t2 = l2 / v_max;
        v_mid = v_max;
    }
    else // 距离不够达到 v_max (三角形/尖峰 或 V形/低谷速度曲线)
    {
        l2 = 0.0f;
        if (v_max > fmaxf(v0, v3)) 
        {
            // 目标速度高于两端速度，但距离不够：形成 A 字形尖峰 (加速后立即减速)
            v_mid = sqrtf((v0 * v0 + 2 * a_max * dist + v3 * v3) / 2.0f);
        }
        else if (v_max < fminf(v0, v3)) 
        {
            // 目标速度低于两端速度，但距离不够：形成 V 字形低谷 (减速后立即加速)
            // fmaxf 保护防止浮点精度误差导致根号内出现负数
            v_mid = sqrtf(fmaxf(0.0f, (v0 * v0 + v3 * v3 - 2 * a_max * dist) / 2.0f));
        }
        else 
        {
            // 如果 v_max 介于 v0 和 v3 之间，理论上 l2 >= 0 恒成立。
            // 此处仅作为浮点数精度防线，退化为无匀速段的单向过渡
            v_mid = v_max;
        }
        
        // 重新计算受限后的实际过渡距离和时间
        l1 = fabsf(v_mid * v_mid - v0 * v0) / (2 * a_max);
        t1 = fabsf(v_mid - v0) / a_max;
        t3 = fabsf(v_mid - v3) / a_max;
    }

    // 赋值坐标与速度 (使用过渡点的实际速度 v_mid)
    points[1].pos.fpPosX = points[0].pos.fpPosX + l1 * cosf(theta);
    points[1].pos.fpPosY = points[0].pos.fpPosY + l1 * sinf(theta);
    points[1].vel.fpVx = v_mid * cosf(theta);
    points[1].vel.fpVy = v_mid * sinf(theta);
    points[1].time = t1;

    points[2].pos.fpPosX = points[1].pos.fpPosX + l2 * cosf(theta);
    points[2].pos.fpPosY = points[1].pos.fpPosY + l2 * sinf(theta);
    points[2].vel.fpVx = v_mid * cosf(theta);
    points[2].vel.fpVy = v_mid * sinf(theta);
    points[2].time = t2;

    points[3].time = t3;

    // 基于真实物理路程进行航向角 (Q) 与 角速度 (W) 的插值计算
    float delta = norm_angle(points[3].pos.fpPosQ - points[0].pos.fpPosQ);
    float dq_ds = delta / dist; // 单位距离的偏航角变化率 (弧度/米)

    points[1].pos.fpPosQ = points[0].pos.fpPosQ + delta * (l1 / dist);
    points[2].pos.fpPosQ = points[0].pos.fpPosQ + delta * ((l1 + l2) / dist);
    points[3].pos.fpPosQ = points[0].pos.fpPosQ + delta;

    // 角速度 W = (dQ/ds) * (ds/dt) = dq_ds * v_current
    points[1].vel.fpW = v_mid * dq_ds;
    points[2].vel.fpW = v_mid * dq_ds;
}


void Points_pop(PATH_WAYPOINT *points, uint8_t index) // 删除点points[index]
{
    if (index >= WAYPOINT_SIZE)
        return;
    if (index != WAYPOINT_SIZE - 1)
        memmove(&points[index], &points[index + 1], sizeof(PATH_WAYPOINT) * (WAYPOINT_SIZE - index - 1));
    memset(&points[WAYPOINT_SIZE - 1], 0, sizeof(PATH_WAYPOINT));
}

void Points_time_extension(PATH_WAYPOINT *point, float k_time){
    point->time *= k_time;
    point->vel.fpVx /= k_time;
    point->vel.fpVy /= k_time;
    point->vel.fpW /= k_time;
}

void yaw_plan(PATH_WAYPOINT *start_point, PATH_WAYPOINT *pass_point, PATH_WAYPOINT *end_point){
    // 1. 计算角度差值（考虑航向角回绕）
    float delta = norm_angle(end_point->pos.fpPosQ - start_point->pos.fpPosQ);
    
    float t1 = pass_point->time;                  // 从起点到中间点的时间
    float T = pass_point->time + end_point->time; // 总时间 t_sum

    // 防零除保护
    if (T <= 1e-5f) {
        pass_point->pos.fpPosQ = start_point->pos.fpPosQ;
        pass_point->vel.fpW = start_point->vel.fpW;
        return;
    }

    // 2. 获取边界速度条件
    float w0 = start_point->vel.fpW; // 起点角速度
    float w1 = end_point->vel.fpW;   // 终点角速度

    // 计算倒数以减少除法运算开销
    float inv_T = 1.0f / T;
    float inv_T2 = inv_T * inv_T;
    float inv_T3 = inv_T2 * inv_T;

    // 3. 计算三次多项式系数
    float a0 = start_point->pos.fpPosQ;
    float a1 = w0;
    float a2 = (3.0f * delta - (2.0f * w0 + w1) * T) * inv_T2;
    float a3 = (-2.0f * delta + (w0 + w1) * T) * inv_T3;

    // 4. 计算 t1 时刻的插值位置和速度
    float t1_2 = t1 * t1;
    float t1_3 = t1_2 * t1;

    // 角度插值
    pass_point->pos.fpPosQ = norm_angle(a0 + a1 * t1 + a2 * t1_2 + a3 * t1_3);
    // 速度插值（三次插值对应的速度是二次曲线）
    pass_point->vel.fpW = a1 + 2.0f * a2 * t1 + 3.0f * a3 * t1_2;
}

void yaw_plan_4points(PATH_WAYPOINT points[4]) {
    // 1. 计算各阶段时间及总时间
    float t1 = points[1].time;                               // 从起点到第1个中间点的时间
    float t2 = t1 + points[2].time;                          // 从起点到第2个中间点的时间
    float T = t2 + points[3].time;                           // 总时间 t_sum (从起点到终点)

    // 防零除保护
    if (T <= 1e-5f) {
        points[1].pos.fpPosQ = points[0].pos.fpPosQ;
        points[1].vel.fpW = points[0].vel.fpW;
        points[2].pos.fpPosQ = points[0].pos.fpPosQ;
        points[2].vel.fpW = points[0].vel.fpW;
        return;
    }

    // 2. 计算起点到终点的角度差值（考虑航向角回绕）
    float delta = norm_angle(points[3].pos.fpPosQ - points[0].pos.fpPosQ);
    
    // 3. 获取边界速度条件（起点和终点）
    float w0 = points[0].vel.fpW; // 起点角速度
    float w3 = points[3].vel.fpW; // 终点角速度

    // 计算倒数以减少除法运算开销
    float inv_T = 1.0f / T;
    float inv_T2 = inv_T * inv_T;
    float inv_T3 = inv_T2 * inv_T;

    // 4. 计算三次多项式系数
    float a0 = points[0].pos.fpPosQ;
    float a1 = w0;
    float a2 = (3.0f * delta - (2.0f * w0 + w3) * T) * inv_T2;
    float a3 = (-2.0f * delta + (w0 + w3) * T) * inv_T3;

    // 5. 计算 t1 时刻（第1个中间点）的插值位置和速度
    float t1_2 = t1 * t1;
    float t1_3 = t1_2 * t1;
    // 角度插值
    points[1].pos.fpPosQ = norm_angle(a0 + a1 * t1 + a2 * t1_2 + a3 * t1_3);
    // 速度插值
    points[1].vel.fpW = a1 + 2.0f * a2 * t1 + 3.0f * a3 * t1_2;

    // 6. 计算 t2 时刻（第2个中间点）的插值位置和速度
    float t2_2 = t2 * t2;
    float t2_3 = t2_2 * t2;
    // 角度插值
    points[2].pos.fpPosQ = norm_angle(a0 + a1 * t2 + a2 * t2_2 + a3 * t2_3);
    // 速度插值
    points[2].vel.fpW = a1 + 2.0f * a2 * t2 + 3.0f * a3 * t2_2;
}

void yaw_choose(PATH_WAYPOINT *start_point, PATH_WAYPOINT *pass_point, PATH_WAYPOINT *end_point, uint8_t flag)
{
    // 保护：防止空指针
    if (start_point == NULL || pass_point == NULL || end_point == NULL)
        return;

    // 直接获取已知的总时间 T
    float T = end_point->time;

    // 限制最小总时间，防止后续除零崩溃
    if (T < 1e-4f)
        T = 1e-4f;

    // 计算两点间的航向角差值 (考虑 -180 ~ 180 度跨越)
    float dq_min = norm_angle(end_point->pos.fpPosQ - start_point->pos.fpPosQ);
    float dq_total = 0.0f;

    // 根据旋转方向 flag，唯一确定 (-360, 360) 范围内的实际转角 dq_total
    if (flag == 1) // 1 表示逆时针旋转 (正方向)
    {
        dq_total = (dq_min >= 0.0f) ? dq_min : (dq_min + 360.0f);
    }
    else // 0 表示顺时针旋转 (负方向)
    {
        dq_total = (dq_min <= 0.0f) ? dq_min : (dq_min - 360.0f);
    }

    // 提取起始与终点状态
    float x0 = start_point->pos.fpPosX;
    float y0 = start_point->pos.fpPosY;
    float q0 = start_point->pos.fpPosQ;
    float vx0 = start_point->vel.fpVx;
    float vy0 = start_point->vel.fpVy;
    float W0 = start_point->vel.fpW;

    float x2 = end_point->pos.fpPosX;
    float y2 = end_point->pos.fpPosY;
    float vx2 = end_point->vel.fpVx;
    float vy2 = end_point->vel.fpVy;
    float W2 = end_point->vel.fpW;

    // 代入 u = 0.5 时的埃尔米特(Hermite)插值基函数计算中间点坐标与偏航角
    // 基函数系数为: h00 = 0.5, h10 = 0.125, h01 = 0.5, h11 = -0.125
    pass_point->pos.fpPosX = 0.5f * (x0 + x2) + 0.125f * T * (vx0 - vx2);
    pass_point->pos.fpPosY = 0.5f * (y0 + y2) + 0.125f * T * (vy0 - vy2);
    pass_point->pos.fpPosQ = norm_angle(q0 + 0.5f * dq_total + 0.125f * T * (W0 - W2));

    // 代入 u = 0.5 时的导数基函数计算中间点线速度与角速度 (对时间 t 求导需除以 T)
    // 导数系数为: dh00 = -1.5, dh10 = -0.25, dh01 = 1.5, dh11 = -0.25
    pass_point->vel.fpVx = 1.5f * (x2 - x0) / T - 0.25f * (vx0 + vx2);
    pass_point->vel.fpVy = 1.5f * (y2 - y0) / T - 0.25f * (vy0 + vy2);
    pass_point->vel.fpW  = 1.5f * dq_total / T - 0.25f * (W0 + W2);

    // 重新写入各分段的相对时间 (该点相对于上一个状态点的时间，中点平分总时间)
    pass_point->time = 0.5f * T;
    end_point->time  = 0.5f * T; 
}


