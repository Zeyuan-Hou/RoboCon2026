#include "steer_wheels.h"

#define WHEEL_HALF_LEN 207.5f // 四车轮组成的矩形y方向半长
#define WHEEL_HALF_WID 207.5f // 四车轮组成的矩形x方向半长
#define WHEEL_R 46            // 车轮半径
#define DEADZONE_LIN_VEL 40   // 线速度死区

#define MAX_WHEEL_SLIP_VEL 140// 允许的最大单轮速度偏差（单位: mm/s）。如果某轮测量速度与估算速度的偏差平方超过此阈值，则判定为打滑。
#define SLIP_THRESHOLD_SQR (MAX_WHEEL_SLIP_VEL * MAX_WHEEL_SLIP_VEL)

#define WHEEL_LU_DIR -1 // 1或者-1，用来标记方向
#define WHEEL_RU_DIR 1
#define WHEEL_LD_DIR 1
#define WHEEL_RD_DIR 1

#define VEL_FILTER_SIZE 5


void steer_optimize(STEER_WHEEL *wheel, float Q_des, float vel_des) // 将一个优化后的舵轮角度更新到结构体中
{
    float Q_diff = norm_angle(Q_des - wheel->pos);
    if (fabs(Q_diff) < 90.f)
    {
        wheel->vel = vel_des;
        wheel->pos = wheel->pos + Q_diff;
    }
    else
    {
        wheel->vel = -vel_des;
        wheel->pos = wheel->pos + Q_diff - sgnf(Q_diff) * 180.f;
    }
}

float acc_W_k = 1.f;

void SteerWheels_FfCalc(STEER_WHEELS *wheels, ST_VEL *local_vel, float ff_v_k1, float ff_v_k2, float ff_w_k1, float ff_w_k2)
{ // 二阶前馈计算
    wheels->td_Vx.aim = local_vel->fpVx / WHEEL_R;
    wheels->td_Vy.aim = local_vel->fpVy / WHEEL_R;
    wheels->td_W.aim = local_vel->fpW / WHEEL_R; // 提前计算这些系数，防止td的r项过大
    CalTD(&wheels->td_Vx);
    CalTD(&wheels->td_Vy);
    CalTD(&wheels->td_W);
    float ff_x = ff_v_k1 * wheels->td_Vx.aim + ff_v_k2 * wheels->td_Vx.x2;
    float ff_y = ff_v_k1 * wheels->td_Vy.aim + ff_v_k2 * wheels->td_Vy.x2;
    float ff_w = ff_w_k1 * wheels->td_W.aim + ff_w_k2 * wheels->td_W.x2;

    float alphaL = ff_w * WHEEL_HALF_LEN * DEG_TO_RAD;
    float alphaW = ff_w * WHEEL_HALF_WID * DEG_TO_RAD;

    float lu_Ax = ff_x - alphaL;
    float lu_Ay = ff_y - alphaW;
    float lu_ang = (wheels->leftup.steer.angle - wheels->leftup.steer_init_angle) * DEG_TO_RAD; // wheels->leftup.pos * DEG_TO_RAD;
    wheels->leftup.ff = WHEEL_LU_DIR * (lu_Ay * cosf(lu_ang) + lu_Ax * sinf(lu_ang));           //+sin是因为pos是顺时针坐标系

    float ru_Ax = ff_x - alphaL;
    float ru_Ay = ff_y + alphaW;
    float ru_ang = (wheels->rightup.steer.angle - wheels->rightup.steer_init_angle) * DEG_TO_RAD; // wheels->rightup.pos * DEG_TO_RAD;
    wheels->rightup.ff = WHEEL_RU_DIR * (ru_Ay * cosf(ru_ang) + ru_Ax * sinf(ru_ang));

    float ld_Ax = ff_x + alphaL;
    float ld_Ay = ff_y - alphaW;
    float ld_ang = (wheels->leftdown.steer.angle - wheels->leftdown.steer_init_angle) * DEG_TO_RAD; // wheels->leftdown.pos * DEG_TO_RAD;
    wheels->leftdown.ff = WHEEL_LD_DIR * (ld_Ay * cosf(ld_ang) + ld_Ax * sinf(ld_ang));

    float rd_Ax = ff_x + alphaL;
    float rd_Ay = ff_y + alphaW;
    float rd_ang = (wheels->rightdown.steer.angle - wheels->rightdown.steer_init_angle) * DEG_TO_RAD; // wheels->rightdown.pos * DEG_TO_RAD;
    wheels->rightdown.ff = WHEEL_RD_DIR * (rd_Ay * cosf(rd_ang) + rd_Ax * sinf(rd_ang));
}

void SteerWheels_distribute(STEER_WHEELS *wheels, ST_VEL *local_vel)
{
    // 定义物理尺寸：W为中心到轮子的横向距离(half_width)，L为中心到轮子的纵向距离(half_length)
    // 根据 X右Y前 坐标系：
    // 左前 (LeftUp):   x = -W, y = +L
    // 右前 (RightUp):  x = +W, y = +L
    // 左后 (LeftDown):  x = -W, y = -L
    // 右后 (RightDown): x = +W, y = -L

    // 旋转分量计算 (V = omega * r, 这里的 omega 需转换为弧度)
    float wL = local_vel->fpW * DEG_TO_RAD * WHEEL_HALF_LEN;
    float wW = local_vel->fpW * DEG_TO_RAD * WHEEL_HALF_WID;

    // 1. 运动学分解：计算每个轮子在 X 和 Y 方向上的速度向量
    // Vx_wheel = Vx_body - omega * y
    // Vy_wheel = Vy_body + omega * x

    // 左前 (LeftUp)
    float lu_Vx = local_vel->fpVx - wL;
    float lu_Vy = local_vel->fpVy - wW;

    // 右前 (RightUp)
    float ru_Vx = local_vel->fpVx - wL;
    float ru_Vy = local_vel->fpVy + wW;

    // 左后 (LeftDown)
    float ld_Vx = local_vel->fpVx + wL;
    float ld_Vy = local_vel->fpVy - wW;

    // 右后 (RightDown)
    float rd_Vx = local_vel->fpVx + wL;
    float rd_Vy = local_vel->fpVy + wW;

    // 2. 计算各轮目标线速度 (模长)
    float lu_lin_vel = hypotf(lu_Vx, lu_Vy);
    float ru_lin_vel = hypotf(ru_Vx, ru_Vy);
    float ld_lin_vel = hypotf(ld_Vx, ld_Vy);
    float rd_lin_vel = hypotf(rd_Vx, rd_Vy);

    // 3. 计算各轮目标角度并执行舵角优化
    // 要求：pos=0 为 Y轴正方向。atan2f(x, y) 刚好满足：当x=0,y=1时角度为0。

    // 左前
    if (lu_lin_vel > DEADZONE_LIN_VEL)
    {
        float lu_pos = atan2f(lu_Vx, lu_Vy) * RAD_TO_DEG;
        steer_optimize(&wheels->leftup, lu_pos, WHEEL_LU_DIR * lu_lin_vel / WHEEL_R);
    }
    else
    {
        wheels->leftup.vel = copysignf(lu_lin_vel / WHEEL_R, wheels->leftup.vel);
    }

    // 右前
    if (ru_lin_vel > DEADZONE_LIN_VEL)
    {
        float ru_pos = atan2f(ru_Vx, ru_Vy) * RAD_TO_DEG;
        steer_optimize(&wheels->rightup, ru_pos, WHEEL_RU_DIR * ru_lin_vel / WHEEL_R);
    }
    else
    {
        wheels->rightup.vel = copysignf(ru_lin_vel / WHEEL_R, wheels->rightup.vel);
    }

    // 左后
    if (ld_lin_vel > DEADZONE_LIN_VEL)
    {
        float ld_pos = atan2f(ld_Vx, ld_Vy) * RAD_TO_DEG;
        steer_optimize(&wheels->leftdown, ld_pos, WHEEL_LD_DIR * ld_lin_vel / WHEEL_R);
    }
    else
    {
        wheels->leftdown.vel = copysignf(ld_lin_vel / WHEEL_R, wheels->leftdown.vel);
    }

    // 右后
    if (rd_lin_vel > DEADZONE_LIN_VEL)
    {
        float rd_pos = atan2f(rd_Vx, rd_Vy) * RAD_TO_DEG;
        steer_optimize(&wheels->rightdown, rd_pos, WHEEL_RD_DIR * rd_lin_vel / WHEEL_R);
    }
    else
    {
        wheels->rightdown.vel = copysignf(rd_lin_vel / WHEEL_R, wheels->rightdown.vel);
    }
}

void SteerLock(STEER_WHEELS *wheels, float leftup_angle, float rightup_angle, float leftdown_angle, float rightdown_angle)
{ // 逆时针为正方向
    steer_optimize(&wheels->leftup, -leftup_angle, 0.0f);
    steer_optimize(&wheels->rightup, -rightup_angle, 0.0f);
    steer_optimize(&wheels->leftdown, -leftdown_angle, 0.0f);
    steer_optimize(&wheels->rightdown, -rightdown_angle, 0.0f);
}

void SteerFixed(STEER_WHEELS *wheels, float angle, float vel, float w)
{
    // 将角度转换为弧度
    float rad = angle * DEG_TO_RAD;
    float omega = w * DEG_TO_RAD;
    // 计算旋转速度在当前舵向上的投影分量
    // 这里的 W=WHEEL_HALF_WID, L=WHEEL_HALF_LEN
    // 根据公式：v_diff = omega * (x * cos(theta) + y * sin(theta))

    float cos_a = cosf(rad);
    float sin_a = sinf(rad);

    // 预计算中间变量
    float wW_comp = omega * WHEEL_HALF_WID * cos_a;
    float wL_comp = omega * WHEEL_HALF_LEN * sin_a;
    // 计算各轮在固定角度上的线速度
    // 左前 (x=-W, y=+L): v = vel + omega*(-W*cos - L*sin) = vel - wW_comp - wL_comp
    float lu_vel = vel - wW_comp + wL_comp;

    // 右前 (x=+W, y=+L): v = vel + omega*(+W*cos - L*sin) = vel + wW_comp - wL_comp
    float ru_vel = vel + wW_comp + wL_comp;

    // 左后 (x=-W, y=-L): v = vel + omega*(-W*cos + L*sin) = vel - wW_comp + wL_comp
    float ld_vel = vel - wW_comp - wL_comp;

    // 右后 (x=+W, y=-L): v = vel + omega*(+W*cos + L*sin) = vel + wW_comp + wL_comp
    float rd_vel = vel + wW_comp - wL_comp;
    // 执行舵角优化。

    if (lu_vel > DEADZONE_LIN_VEL)
        steer_optimize(&wheels->leftup, -angle, WHEEL_LU_DIR * lu_vel / WHEEL_R);
    else
        wheels->leftup.vel = 0;
    if (ru_vel > DEADZONE_LIN_VEL)
        steer_optimize(&wheels->rightup, -angle, WHEEL_RU_DIR * ru_vel / WHEEL_R);
    else
        wheels->rightup.vel = 0;
    if (ld_vel > DEADZONE_LIN_VEL)
        steer_optimize(&wheels->leftdown, -angle, WHEEL_LD_DIR * ld_vel / WHEEL_R);
    else
        wheels->leftdown.vel = 0;
    if (rd_vel > DEADZONE_LIN_VEL)
        steer_optimize(&wheels->rightdown, -angle, WHEEL_RD_DIR * rd_vel / WHEEL_R);
    else
        wheels->rightdown.vel = 0;
}

// --- 用户可调参数 ---

//轮子相关数组的命名顺序：lu,ru,ld,rd
const float sw_x_coords[4] = {-WHEEL_HALF_WID, WHEEL_HALF_WID, -WHEEL_HALF_WID, WHEEL_HALF_WID};
const float sw_y_coords[4] = {WHEEL_HALF_LEN, WHEEL_HALF_LEN, -WHEEL_HALF_LEN, -WHEEL_HALF_LEN};

const uint8_t watch_size = 10;
float sse_watch[watch_size] = {0}; // 滑动窗口平均滤波

float vx_filter_buf[VEL_FILTER_SIZE] = {0};
float vy_filter_buf[VEL_FILTER_SIZE] = {0};
///**
// * @brief 通用最小二乘法求解底盘速度（支持 4 轮、3 轮 或 2 轮）
// * @details 基于 A^T * A * x = A^T * b 的克莱姆法则解析解
// */
static void solve_kinematics_general_ls(const float vx_m[4], const float vy_m[4], const uint8_t active[4], ST_VEL *vel)
{
    uint8_t M = active[0] + active[1] + active[2] + active[3];

    // 如果有效轮子少于2个，属于欠静定状态，无法唯一解算，输出0以确保安全
    if (M < 2)
    {
        memset(vel, 0, sizeof(ST_VEL));
        return;
    }

    // 1. 计算最小二乘矩阵的各个累加元素
    float Sx = 0.0f, Sy = 0.0f, Sxx_yy = 0.0f;
    float B1 = 0.0f, B2 = 0.0f, B3 = 0.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        if (active[i])
        {
            Sx += sw_x_coords[i];
            Sy += sw_y_coords[i];
            Sxx_yy += (sw_x_coords[i] * sw_x_coords[i] + sw_y_coords[i] * sw_y_coords[i]);
            B1 += vx_m[i];
            B2 += vy_m[i];
            B3 += (sw_x_coords[i] * vy_m[i] - sw_y_coords[i] * vx_m[i]);
        }
    }

    float M_f = (float)M;
    
    // 2. 计算系数矩阵的行列式 D = M * (M * Sxx_yy - Sx^2 - Sy^2)
    float d = M_f * Sxx_yy - Sx * Sx - Sy * Sy;
    float D = M_f * d;

    // 防零除保护（如果轮子物理坐标重合，D可能为0。实际车体结构不可能发生）
    if (fabsf(D) < 1e-5f)
    {
        memset(vel, 0, sizeof(ST_VEL));
        return;
    }

    // 3. 计算未知数的行列式 (克莱姆法则展开)
    float D_vx = B1 * (M_f * Sxx_yy - Sx * Sx) - B2 * Sx * Sy + B3 * M_f * Sy;
    float D_vy = B2 * (M_f * Sxx_yy - Sy * Sy) - B1 * Sx * Sy - B3 * M_f * Sx;
    float D_w  = M_f * (M_f * B3 - B2 * Sx + B1 * Sy);

    // 4. 求解底盘速度
    vel->fpVx = D_vx / D;
    vel->fpVy = D_vy / D;
    vel->fpW = D_w / D;
}

float max_delta = 0;
ST_VEL last_valid_vel = {0};

void SteerWheels_solve(STEER_WHEELS *wheels, ST_VEL *cur_vel, float *residual_sse)
{
    float v_ix[4], v_iy[4];

    // 1. 数据还原 (速度单位变为 mm/s)
    float lu_ang = (wheels->leftup.steer.angle - wheels->leftup.steer_init_angle) * DEG_TO_RAD;
    float lu_v = WHEEL_LU_DIR * wheels->leftup.driver.speed * WHEEL_R;
    v_ix[0] = lu_v * sinf(lu_ang);
    v_iy[0] = lu_v * cosf(lu_ang);

    float ru_ang = (wheels->rightup.steer.angle - wheels->rightup.steer_init_angle) * DEG_TO_RAD;
    float ru_v = WHEEL_RU_DIR * wheels->rightup.driver.speed * WHEEL_R;
    v_ix[1] = ru_v * sinf(ru_ang);
    v_iy[1] = ru_v * cosf(ru_ang);

    float ld_ang = (wheels->leftdown.steer.angle - wheels->leftdown.steer_init_angle) * DEG_TO_RAD;
    float ld_v = WHEEL_LD_DIR * wheels->leftdown.driver.speed * WHEEL_R;
    v_ix[2] = ld_v * sinf(ld_ang);
    v_iy[2] = ld_v * cosf(ld_ang);

    float rd_ang = (wheels->rightdown.steer.angle - wheels->rightdown.steer_init_angle) * DEG_TO_RAD;
    float rd_v = WHEEL_RD_DIR * wheels->rightdown.driver.speed * WHEEL_R;
    v_ix[3] = rd_v * sinf(rd_ang);
    v_iy[3] = rd_v * cosf(rd_ang);
    
    uint8_t active[4] = {1, 1, 1, 1}; // 标记轮子是否启用
    uint8_t M = 4;                    // 当前参与计算的轮子数量
    ST_VEL ret_vel = {0};

    // 2. 第一次求解 (4轮默认全参与)
    solve_kinematics_general_ls(v_ix, v_iy, active, &ret_vel);
    
    // 保存4轮初始计算的速度解，专门用于计算4轮残差平方和
    ST_VEL vel_4wheel = ret_vel;

    // 3. 循环剔除：最多允许执行 2 次剔除（即最少保留 2 个轮子）,最多计算3次偏差
    while(M>1)
    {
        float max_r_sqr = -1.0f;
        uint8_t worst_wheel_idx = 5;

        // 计算当前有效轮子的偏差，找出偏差最大的轮子
        for (uint8_t i = 0; i < 4; i++)
        {
            if (active[i])
            {
                float vx_p = ret_vel.fpVx - ret_vel.fpW * sw_y_coords[i];
                float vy_p = ret_vel.fpVy + ret_vel.fpW * sw_x_coords[i];
                float dx = v_ix[i] - vx_p;
                float dy = v_iy[i] - vy_p;
                float r_sqr = dx * dx + dy * dy;

                if (r_sqr > max_r_sqr)
                {
                    max_r_sqr = r_sqr;
                    worst_wheel_idx = i;
                }
            }
        }
       

        // 如果最大偏差超过阈值，且当前有效轮子数大于2，则剔除并重新解算
        if (max_r_sqr > SLIP_THRESHOLD_SQR && worst_wheel_idx != 5 && M > 2)
        {
            active[worst_wheel_idx] = 0; // 剔除打滑轮
            M--;                         // 有效轮数减 1
            // 使用剩余轮子重新解算
            solve_kinematics_general_ls(v_ix, v_iy, active, &ret_vel);
        }
        else
        {
            // 如果没有打滑或者已经只剩 2 个轮子，则终止检查
            break;
        }
    }
    
    // 计算当前最终解(可能是2轮解)对应的最大偏差
    float final_max_r_sqr = -1.0f;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (active[i])
        {
            float vx_p = ret_vel.fpVx - ret_vel.fpW * sw_y_coords[i];
            float vy_p = ret_vel.fpVy + ret_vel.fpW * sw_x_coords[i];
            float dx = v_ix[i] - vx_p;
            float dy = v_iy[i] - vy_p;
            float r_sqr = dx * dx + dy * dy;

            if (r_sqr > final_max_r_sqr)
            {
                final_max_r_sqr = r_sqr;
            }
        }
    }
		 max_delta = final_max_r_sqr;

    // 判断：如果只剩两个轮子，且最终的最大偏差仍然大于阈值
    if (M <= 2 && final_max_r_sqr > SLIP_THRESHOLD_SQR)
    {
        // 此时解无效，回退到上一次偏差在阈值内的速度
        ret_vel = last_valid_vel;
    }
    else
    {
        // 否则当前解有效，更新历史有效值
        last_valid_vel = ret_vel;
    }

    // 4. 计算残差 (固定使用4轮计算的速度解 vel_4wheel，且计算全部4个轮子的误差)
    float total_res = 0;
    for (int i = 0; i < 4; i++)
    {
        float vx_p = vel_4wheel.fpVx - vel_4wheel.fpW * sw_y_coords[i];
        float vy_p = vel_4wheel.fpVy + vel_4wheel.fpW * sw_x_coords[i];
        float dx = (v_ix[i] - vx_p) / 100.0f; // 防止精度失真
        float dy = (v_iy[i] - vy_p) / 100.0f;
        total_res += dx * dx + dy * dy;
    }

    // 5. 滑动窗口平均滤波
    float total_watch = 0;
    for (uint8_t i = watch_size - 1; i > 0; i--)
    {
        sse_watch[i] = sse_watch[i - 1];
    }
    sse_watch[0] = total_res;
    for (uint8_t i = 0; i < watch_size; i++)
    {
        total_watch += sse_watch[i];
    }
    *residual_sse = total_watch / watch_size;

    // 6. 输出最终解算结果 (如果被判定无效，则输出的是 last_valid_vel)
    cur_vel->fpVx = ret_vel.fpVx;
    cur_vel->fpVy = ret_vel.fpVy;
    cur_vel->fpW = ret_vel.fpW * RAD_TO_DEG;
}

/**
 * @brief 陀螺仪辅助的底盘速度解算（带打滑剔除及 1000Hz/200Hz 速率匹配滤波）
 * @note 运行频率：1000Hz (1ms周期)
 * @param wheels 舵轮结构体指针 (1000Hz 更新)
 * @param cur_vel 输出的平滑底盘速度（Vx, Vy 单位 mm/s; W 单位 deg/s）
 * @param residual_sse 4轮计算的残差平方和滤波输出
 * @param omega 陀螺仪实时反馈的机体角速度 (200Hz 更新，在1ms循环中保持或更新)，单位为 deg/s
 */
void SteerWheels_solve_withGyro(STEER_WHEELS *wheels, ST_VEL *cur_vel, float *residual_sse, float omega)
{
    float v_ix[4], v_iy[4];

    // 1. 数据还原 (速度单位 mm/s)
    float lu_ang = (wheels->leftup.steer.angle - wheels->leftup.steer_init_angle) * DEG_TO_RAD;
    float lu_v = WHEEL_LU_DIR * wheels->leftup.driver.speed * WHEEL_R;
    v_ix[0] = lu_v * sinf(lu_ang);
    v_iy[0] = lu_v * cosf(lu_ang);

    float ru_ang = (wheels->rightup.steer.angle - wheels->rightup.steer_init_angle) * DEG_TO_RAD;
    float ru_v = WHEEL_RU_DIR * wheels->rightup.driver.speed * WHEEL_R;
    v_ix[1] = ru_v * sinf(ru_ang);
    v_iy[1] = ru_v * cosf(ru_ang);

    float ld_ang = (wheels->leftdown.steer.angle - wheels->leftdown.steer_init_angle) * DEG_TO_RAD;
    float ld_v = WHEEL_LD_DIR * wheels->leftdown.driver.speed * WHEEL_R;
    v_ix[2] = ld_v * sinf(ld_ang);
    v_iy[2] = ld_v * cosf(ld_ang);

    float rd_ang = (wheels->rightdown.steer.angle - wheels->rightdown.steer_init_angle) * DEG_TO_RAD;
    float rd_v = WHEEL_RD_DIR * wheels->rightdown.driver.speed * WHEEL_R;
    v_ix[3] = rd_v * sinf(rd_ang);
    v_iy[3] = rd_v * cosf(rd_ang);

    // 将输入的 deg/s 角速度转换为 rad/s 参与物理公式计算
    float omega_rad = omega * DEG_TO_RAD;

    // 2. 利用陀螺仪角速度 omega_rad，解耦计算每个轮子对底盘速度的独立估计值
    float V_x_est[4], V_y_est[4];
    for (uint8_t i = 0; i < 4; i++)
    {
        V_x_est[i] = v_ix[i] + omega_rad * sw_y_coords[i];
        V_y_est[i] = v_iy[i] - omega_rad * sw_x_coords[i];
    }
    
    // 3. 计算 4 轮全参与时的速度均值（用于4轮残差计算）
    ST_VEL vel_4wheel = {0};
    float sum_vx4 = 0.0f, sum_vy4 = 0.0f;
    for (uint8_t i = 0; i < 4; i++)
    {
        sum_vx4 += V_x_est[i];
        sum_vy4 += V_y_est[i];
    }
    vel_4wheel.fpVx = sum_vx4 / 4.0f;
    vel_4wheel.fpVy = sum_vy4 / 4.0f;
    vel_4wheel.fpW  = omega_rad;

    // 4. 循环剔除：每一次剔除距离当前有效均值最远的那个“打滑轮”
    uint8_t active[4] = {1, 1, 1, 1}; 
    uint8_t M = 4;                    
    ST_VEL ret_vel = vel_4wheel;      

    while (M > 2)
    {
        // 4.1 重新计算当前所有有效轮子的均值速度
        float sum_vx = 0.0f, sum_vy = 0.0f;
        for (uint8_t i = 0; i < 4; i++)
        {
            if (active[i])
            {
                sum_vx += V_x_est[i];
                sum_vy += V_y_est[i];
            }
        }
        ret_vel.fpVx = sum_vx / (float)M;
        ret_vel.fpVy = sum_vy / (float)M;

        // 4.2 计算每个有效轮子估计值到当前均值的偏差，找出最大偏差轮子
        float max_r_sqr = -1.0f;
        uint8_t worst_wheel_idx = 5;
        for (uint8_t i = 0; i < 4; i++)
        {
            if (active[i])
            {
                float dx = V_x_est[i] - ret_vel.fpVx;
                float dy = V_y_est[i] - ret_vel.fpVy;
                float r_sqr = dx * dx + dy * dy;

                if (r_sqr > max_r_sqr)
                {
                    max_r_sqr = r_sqr;
                    worst_wheel_idx = i;
                }
            }
        }

        // 4.3 如果偏差最大的轮子超过阈值，将其剔除并准备重新计算均值
        if (max_r_sqr > SLIP_THRESHOLD_SQR && worst_wheel_idx != 5 && M > 2)
        {
            active[worst_wheel_idx] = 0; 
            M--;                         
        }
        else
        {
            break;
        }
    }

    // 5. 兜底策略：如果只剩两个轮子，且最终两个轮子偏差依然大于阈值，使用历史有效值
    static ST_VEL last_valid_vel = {0};
    float final_max_r_sqr = -1.0f;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (active[i])
        {
            float dx = V_x_est[i] - ret_vel.fpVx;
            float dy = V_y_est[i] - ret_vel.fpVy;
            float r_sqr = dx * dx + dy * dy;
            if (r_sqr > final_max_r_sqr)
            {
                final_max_r_sqr = r_sqr;
            }
        }
    }

    if (M <= 2 && final_max_r_sqr > SLIP_THRESHOLD_SQR)
    {
        ret_vel = last_valid_vel;
    }
    else
    {
        last_valid_vel = ret_vel;
    }

    // 6. 计算残差 (固定使用4轮计算的初始解 vel_4wheel，且计算全部4个轮子的误差)
    float total_res = 0;
    for (int i = 0; i < 4; i++)
    {
        float vx_p = vel_4wheel.fpVx - omega_rad * sw_y_coords[i];
        float vy_p = vel_4wheel.fpVy + omega_rad * sw_x_coords[i];
        float dx = (v_ix[i] - vx_p) / 100.0f; 
        float dy = (v_iy[i] - vy_p) / 100.0f;
        total_res += dx * dx + dy * dy;
    }

    // 7. 残差的滑动窗口平均滤波（sse_watch）
    float total_watch = 0;
    for (uint8_t i = watch_size - 1; i > 0; i--)
    {
        sse_watch[i] = sse_watch[i - 1];
    }
    sse_watch[0] = total_res;
    for (uint8_t i = 0; i < watch_size; i++)
    {
        total_watch += sse_watch[i];
    }
    *residual_sse = total_watch / watch_size;

    // 8. 【新增核心点】后端 5 帧滑动窗口均值滤波（消除200Hz陀螺仪阶跃与编码器高频噪声）
    

    
    float sum_vx_filt = 0.0f;
    float sum_vy_filt = 0.0f;
    
    // 移位并压入新解算出的速度
    for (uint8_t i = VEL_FILTER_SIZE - 1; i > 0; i--)
    {
        vx_filter_buf[i] = vx_filter_buf[i - 1];
        vy_filter_buf[i] = vy_filter_buf[i - 1];
    }
    vx_filter_buf[0] = ret_vel.fpVx;
    vy_filter_buf[0] = ret_vel.fpVy;
    
    // 累加求均值
    for (uint8_t i = 0; i < VEL_FILTER_SIZE; i++)
    {
        sum_vx_filt += vx_filter_buf[i];
        sum_vy_filt += vy_filter_buf[i];
    }

    // 9. 输出最终解算结果 (转速 W 转换回 deg/s 输出)
    cur_vel->fpVx = sum_vx_filt / (float)VEL_FILTER_SIZE;
    cur_vel->fpVy = sum_vy_filt / (float)VEL_FILTER_SIZE;
    cur_vel->fpW  = omega; // 陀螺仪原始信号输入，不需要滤波
}



void GlobalVel_To_Local(ST_VEL *local_vel, ST_VEL *global_vel, float fpQ)
{ //-Q则是从local到global
    local_vel->fpW = global_vel->fpW;
    float cosQ = cosf(fpQ * DEG_TO_RAD);
    float sinQ = sinf(fpQ * DEG_TO_RAD);
    local_vel->fpVx = global_vel->fpVx * cosQ + global_vel->fpVy * sinQ;
    local_vel->fpVy = -global_vel->fpVx * sinQ + global_vel->fpVy * cosQ;
}
