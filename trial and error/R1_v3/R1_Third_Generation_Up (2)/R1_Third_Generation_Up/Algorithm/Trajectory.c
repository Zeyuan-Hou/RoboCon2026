#include "algorithm.h"
#include "Trajectory.h"
#include "Logic.h"
#include <stdbool.h>

// 输入：q0, q1, v_max (正值), T (总时间)
// 输出三角形速度曲线或梯形速度曲线参数
uint8_t Traj_Angle_Init(Trajectory* traj, float q0, float q1, float v_max, float T) 
{
    // 检查是否正在运动中
    if (traj->State == TRAJ_ACCEL || traj->State == TRAJ_CONST || traj->State == TRAJ_DECEL) {
        return 0;   // 运动中，拒绝初始化
    }

    // 设置轨迹类型为关节空间
    traj->Type = TRAJ_TYPE_JOINT;
    // 初始状态设为空闲（尚未开始运动）
    traj->State = TRAJ_IDLE;

    // 获取角度轨迹结构体指针
    AngleTraj* angle = &traj->Data.Angle;

    angle->q0 = q0;
    angle->q1 = q1;
    angle->v_max = fabsf(v_max);   // 确保非负
    angle->T = fmaxf(T, 1e-6f);    // 避免除零
    angle->sign = (q1 >= q0) ? 1.0f : -1.0f;
    angle->t_last = 0.0f;
    float delta = fabsf(q1 - q0);   // 位移绝对值

    // 情况1：位移为零，直接静止
    if (delta < 1e-6f) {
        angle->a = 0.0f;
        angle->t_acc = 0.0f;
        angle->t_const = angle->T;
        angle->v_peak = 0.0f;
        return 0;
    }

    // 尝试梯形曲线：需要满足 T >= delta / v_max
    float t_min_trapezoid = delta / angle->v_max;  // 最小时间（匀速段为零）
    if (angle->T > t_min_trapezoid) {
        float v_tri = 2.0f * delta / angle->T;     // 三角形曲线所需峰值速度
        if (v_tri <= angle->v_max) {
            // 三角形曲线（无匀速段）
            angle->v_peak = v_tri;
            angle->t_acc = angle->T / 2.0f;        // 加速时间 = 总时间的一半
            angle->t_const = 0.0f;
            angle->a = angle->v_peak / angle->t_acc;
        } else {
            // 梯形曲线
            angle->v_peak = angle->v_max;
            angle->t_acc = angle->T - delta / angle->v_max;   // 加速时间
            angle->t_const = 2.0f * delta / angle->v_max - angle->T;
            // 确保非负
            if (angle->t_acc < 0) angle->t_acc = 0;
            if (angle->t_const < 0) angle->t_const = 0;
            angle->a = angle->v_peak / angle->t_acc;
        }
    } else {
        // 时间太短，无法完成路程
        // 可根据需要设置错误标志，这里简单返回
        return 0;
    }

    // 最终调整，避免除零
    if (angle->t_acc < 1e-6f) {
        angle->a = 0.0f;
    }
    // 给 v_peak 和 a 带上方向
    angle->v_peak *= angle->sign;
    angle->a *= angle->sign;

    return 1;   // 成功初始化
}

#include <math.h>

/**
 * 根据当前时间更新云台空间轨迹的位置、速度、加速度，并自动切换状态
 * 注意：调用前需设置 traj->Data.Angle.t_last 为当前时间（秒）
 * @param traj  轨迹结构体指针（必须为 TRAJ_TYPE_JOINT 类型）
 * @param pos   输出当前位置（角度，单位度）
 * @param vel   输出当前速度（度/秒）
 * @param acc   输出当前加速度（度/秒?）
 */
void Traj_Angle_Update(Trajectory* traj, float* pos, float* vel, float* acc)
{
    // 类型检查：只处理关节空间轨迹
	// 若第一段运动未开始或运动已结束，直接返回
    if (traj->Type != TRAJ_TYPE_JOINT||traj->State==TRAJ_NOINIT||traj->State==TRAJ_DONE) 
    {
        return;
    }

    AngleTraj* angle = &traj->Data.Angle;
    float t = angle->t_last;   // 当前时间（秒）

    // 边界处理：时间未开始或已完成
    if (t <= 0.0f) 
    {
        *pos = angle->q0;
        *vel = 0.0f;
        *acc = 0.0f;
        traj->State = TRAJ_IDLE;   // 尚未开始
        return;
    }
    if (t >= angle->T) 
    {
        *pos = angle->q1;
        *vel = 0.0f;
        *acc = 0.0f;
        traj->State = TRAJ_DONE;   // 运动完成
        return;
    }

    float sign = angle->sign;
    float a_abs = fabsf(angle->a);
    float v_peak_abs = fabsf(angle->v_peak);
    float t_acc = angle->t_acc;
    float t_const = angle->t_const;
    float t_dec_start = t_acc + t_const;

    // 根据当前时间判定所处阶段，并更新状态
    if (t_const <= 1e-6f) {
        // 三角形曲线：只有加速和减速
        if (t <= t_acc) {
            traj->State = TRAJ_ACCEL;
            *acc = sign * a_abs;
            *vel = sign * a_abs * t;
            *pos = angle->q0 + 0.5f * sign * a_abs * t * t;
        } else {
            traj->State = TRAJ_DECEL;
            float t_dec = t - t_acc;
            *acc = -sign * a_abs;
            *vel = sign * (v_peak_abs - a_abs * t_dec);
            float s_acc = 0.5f * a_abs * t_acc * t_acc;
            float s_dec = v_peak_abs * t_dec - 0.5f * a_abs * t_dec * t_dec;
            *pos = angle->q0 + sign * (s_acc + s_dec);
        }
    } else {
        // 梯形曲线：加速、匀速、减速
        if (t <= t_acc) {
            traj->State = TRAJ_ACCEL;
            *acc = sign * a_abs;
            *vel = sign * a_abs * t;
            *pos = angle->q0 + 0.5f * sign * a_abs * t * t;
        } else if (t <= t_dec_start) {
            traj->State = TRAJ_CONST;
            *acc = 0.0f;
            *vel = sign * v_peak_abs;
            float s_acc = 0.5f * a_abs * t_acc * t_acc;
            *pos = angle->q0 + sign * (s_acc + v_peak_abs * (t - t_acc));
        } else {
            traj->State = TRAJ_DECEL;
            float t_dec = t - t_dec_start;
            *acc = -sign * a_abs;
            *vel = sign * (v_peak_abs - a_abs * t_dec);
            float s_acc = 0.5f * a_abs * t_acc * t_acc;
            float s_const = v_peak_abs * t_const;
            float s_dec = v_peak_abs * t_dec - 0.5f * a_abs * t_dec * t_dec;
            *pos = angle->q0 + sign * (s_acc + s_const + s_dec);
        }
    }

    // 数值安全清理
    if (fabsf(*vel) < 1e-6f) *vel = 0.0f;
    if (fabsf(*acc) < 1e-6f) *acc = 0.0f;
}

// /**
//  * @brief 初始化笛卡尔空间直线轨迹（梯形速度曲线）
//  * 
//  * 根据起点、终点、最大线速度、最大线加速度，自动计算梯形速度曲线的各项参数。
//  * 若位移过短无法达到最大速度，则退化为三角形速度曲线。
//  * 
//  * @param traj    轨迹结构体指针（需包含 State 和 Data.Line）
//  * @param xs,ys   起点坐标 (mm)
//  * @param xe,ye   终点坐标 (mm)
//  * @param V_max   最大线速度 (mm/s)，正值
//  * @param a_max   最大线加速度 (mm/s?)，正值
//  * @return 1=成功，0=失败（运动中或参数无效）
//  */
// uint8_t Traj_Line_Init(Trajectory* traj, float xs, float ys, float xe, float ye,
//                        float V_max, float a_max)
// {
//     // 若当前轨迹正在运行（加速、匀速、减速），禁止重新初始化
//     if (traj->State == TRAJ_ACCEL || traj->State == TRAJ_CONST || traj->State == TRAJ_DECEL) {
//         return 0;
//     }

//     // 设置轨迹类型为直线，初始状态为空闲
//     traj->Type = TRAJ_TYPE_LINE;
//     traj->State = TRAJ_IDLE;

//     // 获取直线轨迹参数指针，方便后续操作
//     LineTraj* line = &traj->Data.Line;

//     // 存储用户给定的起点、终点、速度、加速度
//     line->xs = xs;   line->ys = ys;
//     line->xe = xe;   line->ye = ye;
//     line->V_max = fabsf(V_max);   // 最大速度取绝对值，确保非负
//     line->a_max = fabsf(a_max);   // 最大加速度取绝对值
// 	line->t_last = 0.f;
	

//     // 计算位移矢量及其长度（总弧长）
//     float dx = xe - xs;
//     float dy = ye - ys;
//     line->L = sqrtf(dx*dx + dy*dy);

//     // 若起点与终点重合，轨迹长度为零，直接返回静止轨迹
//     if (line->L < 1e-6f) {
//         line->Vp = 0.0f;        // 峰值速度为零
//         line->t_acc = 0.0f;     // 加速时间为零
//         line->t_const = 0.0f;   // 匀速时间为零
//         line->s_acc = 0.0f;     // 加速段位移为零
//         line->T_total = 0.0f;   // 总时间为零
//         return 1;
//     }

//     // 计算直线方向的单位向量 (ux, uy)
//     line->ux = dx / line->L;
//     line->uy = dy / line->L;

//     /* ---------- 梯形速度曲线参数计算 ---------- */
//     // 假设以最大加速度加速到最大速度所需的加速时间 t_a
//     float t_a = line->V_max / line->a_max;
//     // 对应的加速段位移 s_a = 0.5 * a * t_a^2
//     float s_a = 0.5f * line->a_max * t_a * t_a;

//     // 判断能否达到最大速度：2倍加速段位移是否超过总弧长
//     if (2.0f * s_a >= line->L) {
//         // 无法达到最大速度 → 三角形速度曲线
//         // 实际峰值速度 Vp = sqrt(a_max * L)
//         line->Vp = sqrtf(line->a_max * line->L);
//         // 加速/减速时间 t_acc = Vp / a_max
//         line->t_acc = line->Vp / line->a_max;
//         // 加速段位移 s_acc = 0.5 * a * t_acc^2
//         line->s_acc = 0.5f * line->a_max * line->t_acc * line->t_acc;
//         // 匀速时间为零
//         line->t_const = 0.0f;
//         // 总时间 = 2 * t_acc
//         line->T_total = 2.0f * line->t_acc;
//     } else {
//         // 能达到最大速度 → 梯形速度曲线
//         line->Vp = line->V_max;                     // 峰值速度 = 最大速度
//         line->t_acc = t_a;                          // 加速时间 = t_a
//         line->s_acc = s_a;                          // 加速段位移 = s_a
//         // 匀速段时间 = (总弧长 - 2倍加速位移) / 峰值速度
//         line->t_const = (line->L - 2.0f * s_a) / line->V_max;
//         // 总时间 = 2 * 加速时间 + 匀速时间
//         line->T_total = 2.0f * t_a + line->t_const;
//     }

//     return 1;
// }

// /**
//  * @brief 更新直线轨迹，将末端笛卡尔坐标、速度、加速度直接存入解算结构体
//  * 
//  * 根据结构体中存储的 t_last 时间，计算当前时刻的弧长、速度、加速度，
//  * 然后转换为笛卡尔坐标和速度/加速度分量，并存入 ARM_BACKSOLVING 的对应字段。
//  * 同时自动更新轨迹状态机。
//  * 
//  * @param traj   轨迹结构体指针（必须已调用 Traj_Line_Init）
//  * @param arm    机械臂解算结构体指针（用于存储 x, y, dx, dy, ddx, ddy）
//  */
// void Traj_Line_Update(Trajectory* traj, ARM_BACKSOLVING* arm)
// {
//     // 类型检查：若非直线轨迹，直接返回
// 	// 若第一段运动未开始或运动已结束，直接返回
//     if (traj->Type != TRAJ_TYPE_LINE||traj->State==TRAJ_NOINIT||traj->State==TRAJ_DONE) 
//     {
//         return;
//     }

//     LineTraj* line = &traj->Data.Line;
//     float t = line->t_last;   // 用户需在调用前设置当前时间

//     /* ---------- 边界情况处理 ---------- */
//     if (t <= 0.0f) 
//     {
//         // 时间未开始：输出起点，速度、加速度为零，状态置为空闲
//         arm->x = line->xs; arm->y = line->ys;
//         arm->dx = 0.0f; arm->dy = 0.0f;
//         arm->ddx = 0.0f; arm->ddy = 0.0f;
//         traj->State = TRAJ_IDLE;
//         return;
//     }
//     if (t >= line->T_total) 
//     {
//         // 时间已结束：输出终点，速度、加速度为零，状态置为完成
//         arm->x = line->xe; arm->y = line->ye;
//         arm->dx = 0.0f; arm->dy = 0.0f;
//         arm->ddx = 0.0f; arm->ddy = 0.0f;
//         traj->State = TRAJ_DONE;
//         return;
//     }

//     /* ---------- 梯形速度曲线分段计算 ---------- */
//     float t_acc = line->t_acc;      // 加速阶段结束时间
//     float t_const = line->t_const;  // 匀速阶段持续时间
//     float a_abs = line->a_max;      // 加速度大小
//     float v_peak = line->Vp;        // 峰值速度（正值）
//     float s, v, a;                  // 弧长、速度、加速度（标量）

//     // 区分三角形曲线（t_const ≈ 0）和梯形曲线
//     if (t_const <= 1e-6f) 
//     {
//         // ---------- 三角形曲线：只有加速和减速 ----------
//         if (t <= t_acc) 
//         {
//             // 加速阶段
//             traj->State = TRAJ_ACCEL;
//             a = a_abs;                      // 加速度 = +a_max
//             v = a_abs * t;                  // 速度线性增加
//             s = 0.5f * a_abs * t * t;       // 位移 = 1/2 * a * t^2
//         } 
//         else 
//         {
//             // 减速阶段
//             traj->State = TRAJ_DECEL;
//             float t_dec = t - t_acc;        // 已减速的时间
//             a = -a_abs;                     // 加速度 = -a_max
//             v = v_peak - a_abs * t_dec;     // 速度线性减小
//             float s_acc = 0.5f * a_abs * t_acc * t_acc;   // 加速段位移
//             float s_dec = v_peak * t_dec - 0.5f * a_abs * t_dec * t_dec; // 减速段位移
//             s = s_acc + s_dec;              // 总位移 = 加速 + 减速
//         }
//     } 
//     else 
//     {
//         // ---------- 梯形曲线：加速、匀速、减速 ----------
//         float t_dec_start = t_acc + t_const;  // 减速阶段开始时间
//         if (t <= t_acc) 
//         {
//             // 加速阶段
//             traj->State = TRAJ_ACCEL;
//             a = a_abs;
//             v = a_abs * t;
//             s = 0.5f * a_abs * t * t;
//         } 
//         else if (t <= t_dec_start) 
//         {
//             // 匀速阶段
//             traj->State = TRAJ_CONST;
//             a = 0.0f;
//             v = v_peak;
//             float s_acc = 0.5f * a_abs * t_acc * t_acc;
//             s = s_acc + v_peak * (t - t_acc);   // 位移 = 加速段位移 + 匀速段位移
//         } 
//         else 
//         {
//             // 减速阶段
//             traj->State = TRAJ_DECEL;
//             float t_dec = t - t_dec_start;      // 已减速的时间
//             a = -a_abs;
//             v = v_peak - a_abs * t_dec;
//             float s_acc = 0.5f * a_abs * t_acc * t_acc;
//             float s_const = v_peak * t_const;
//             float s_dec = v_peak * t_dec - 0.5f * a_abs * t_dec * t_dec;
//             s = s_acc + s_const + s_dec;        // 加速 + 匀速 + 减速
//         }
//     }

//     // 数值安全：避免极小的浮点误差导致符号错误
//     if (fabsf(v) < 1e-6f) v = 0.0f;
//     if (fabsf(a) < 1e-6f) a = 0.0f;

//     /* ---------- 将标量弧长、速度、加速度转换为笛卡尔分量，存入结构体 ---------- */
//     arm->x = line->xs + s * line->ux;       // 位置 = 起点 + 弧长 * 方向向量
//     arm->y = line->ys + s * line->uy;
//     arm->dx = v * line->ux;                // 速度分量
//     arm->dy = v * line->uy;
//     arm->ddx = a * line->ux;               // 加速度分量
//     arm->ddy = a * line->uy;
// }

// /**
//  * @brief 初始化笛卡尔空间圆弧轨迹（梯形速度曲线）
//  * 
//  * 根据圆心、半径、起始角、扫过角、最大线速度、最大线加速度，
//  * 自动计算梯形速度曲线的各项参数。弧长 = 半径 * |扫过角|。
//  * 
//  * @param traj         轨迹结构体指针
//  * @param cx,cy        圆心坐标 (mm)
//  * @param r            圆弧半径 (mm)，正值
//  * @param start_angle  起始角度 (°)，从圆心指向起点的向量角
//  * @param sweep_angle  扫过角度 (°)，正值逆时针，负值顺时针
//  * @param V_max        最大线速度 (mm/s)，正值
//  * @param a_max        最大线加速度 (mm/s)，正值
//  * @return 1=成功，0=失败（运动中或参数无效）
//  */
// uint8_t Traj_Arc_Init(Trajectory* traj, float cx, float cy, float r,
//                       float start_angle, float sweep_angle,
//                       float V_max, float a_max)
// {
//     // 运动中禁止重新初始化
//     if (traj->State == TRAJ_ACCEL || traj->State == TRAJ_CONST || traj->State == TRAJ_DECEL) 
//     {
//         return 0;
//     }

//     // 设置类型为圆弧，初始状态空闲
//     traj->Type = TRAJ_TYPE_ARC;
//     traj->State = TRAJ_IDLE;

//     ArcTraj* arc = &traj->Data.Arc;
//     // 存储几何参数
//     arc->cx = cx;
//     arc->cy = cy;
//     arc->r = fabsf(r);                     // 半径取绝对值
//     arc->start_angle = start_angle*RADIAN;
//     arc->sweep_angle = sweep_angle*RADIAN;
//     arc->V_max = fabsf(V_max);
//     arc->a_max = fabsf(a_max);
//     arc->t_last = 0.0f;

//     // 总弧长 = 半径 * 扫过角绝对值
//     arc->L = arc->r * fabsf(arc->sweep_angle);
//     if (arc->L < 1e-6f) 
//     {
//         // 弧长为零 → 静止轨迹
//         arc->Vp = 0.0f;
//         arc->t_acc = 0.0f;
//         arc->t_const = 0.0f;
//         arc->s_acc = 0.0f;
//         arc->T_total = 0.0f;
//         return 1;
//     }

//     /* ---------- 梯形速度参数计算（与直线完全相同） ---------- */
//     float t_a = arc->V_max / arc->a_max;
//     float s_a = 0.5f * arc->a_max * t_a * t_a;

//     if (2.0f * s_a >= arc->L) 
//     {
//         // 三角形速度曲线
//         arc->Vp = sqrtf(arc->a_max * arc->L);
//         arc->t_acc = arc->Vp / arc->a_max;
//         arc->s_acc = 0.5f * arc->a_max * arc->t_acc * arc->t_acc;
//         arc->t_const = 0.0f;
//         arc->T_total = 2.0f * arc->t_acc;
//     } 
//     else 
//     {
//         // 梯形速度曲线
//         arc->Vp = arc->V_max;
//         arc->t_acc = t_a;
//         arc->s_acc = s_a;
//         arc->t_const = (arc->L - 2.0f * s_a) / arc->V_max;
//         arc->T_total = 2.0f * t_a + arc->t_const;
//     }

//     return 1;
// }

// /**
//  * @brief 更新圆弧轨迹，将末端笛卡尔坐标、速度、加速度直接存入解算结构体
//  * 
//  * 根据时间 t_last 计算当前弧长、速度、加速度，然后根据圆弧几何关系
//  * 计算出末端坐标、切向速度、切向加速度以及法向（向心）加速度，
//  * 最终合成笛卡尔坐标系下的速度/加速度分量，并存入 ARM_BACKSOLVING 的对应字段。
//  * 同时自动更新轨迹状态。
//  * 
//  * @param traj   轨迹结构体指针（必须已调用 Traj_Arc_Init）
//  * @param arm    机械臂解算结构体指针（用于存储 x, y, dx, dy, ddx, ddy）
//  */
// void Traj_Arc_Update(Trajectory* traj, ARM_BACKSOLVING* arm)
// {
//     // 类型检查
// 	// 若第一段运动未开始或运动已结束，直接返回
//     if (traj->Type != TRAJ_TYPE_ARC||traj->State==TRAJ_NOINIT||traj->State==TRAJ_DONE) 
//     {
//         return;
//     }

//     ArcTraj* arc = &traj->Data.Arc;
//     float t = arc->t_last;

//     /* ---------- 边界情况 ---------- */
//     if (t <= 0.0f) 
//     {
//         // 输出起点
//         float angle_start = arc->start_angle;
//         arm->x = arc->cx + arc->r * cosf(angle_start);
//         arm->y = arc->cy + arc->r * sinf(angle_start);
//         arm->dx = 0.0f; arm->dy = 0.0f;
//         arm->ddx = 0.0f; arm->ddy = 0.0f;
//         traj->State = TRAJ_IDLE;
//         return;
//     }
//     if (t >= arc->T_total) 
//     {
//         // 输出终点
//         float angle_end = arc->start_angle + arc->sweep_angle;
//         arm->x = arc->cx + arc->r * cosf(angle_end);
//         arm->y = arc->cy + arc->r * sinf(angle_end);
//         arm->dx = 0.0f; arm->dy = 0.0f;
//         arm->ddx = 0.0f; arm->ddy = 0.0f;
//         traj->State = TRAJ_DONE;
//         return;
//     }

//     /* ---------- 梯形速度曲线分段计算弧长 s、速度 v、切向加速度 a ---------- */
//     float t_acc = arc->t_acc;
//     float t_const = arc->t_const;
//     float a_abs = arc->a_max;
//     float v_peak = arc->Vp;
//     float s, v, a;   // 弧长、速度、切向加速度（标量）

//     if (t_const <= 1e-6f) 
//     {
//         // 三角形曲线
//         if (t <= t_acc) 
//         {
//             traj->State = TRAJ_ACCEL;
//             a = a_abs;
//             v = a_abs * t;
//             s = 0.5f * a_abs * t * t;
//         } 
//         else 
//         {
//             traj->State = TRAJ_DECEL;
//             float t_dec = t - t_acc;
//             a = -a_abs;
//             v = v_peak - a_abs * t_dec;
//             float s_acc = 0.5f * a_abs * t_acc * t_acc;
//             float s_dec = v_peak * t_dec - 0.5f * a_abs * t_dec * t_dec;
//             s = s_acc + s_dec;
//         }
//     } 
//     else 
//     {
//         // 梯形曲线
//         float t_dec_start = t_acc + t_const;
//         if (t <= t_acc) 
//         {
//             traj->State = TRAJ_ACCEL;
//             a = a_abs;
//             v = a_abs * t;
//             s = 0.5f * a_abs * t * t;
//         } 
//         else if (t <= t_dec_start) 
//         {
//             traj->State = TRAJ_CONST;
//             a = 0.0f;
//             v = v_peak;
//             float s_acc = 0.5f * a_abs * t_acc * t_acc;
//             s = s_acc + v_peak * (t - t_acc);
//         } 
//         else 
//         {
//             traj->State = TRAJ_DECEL;
//             float t_dec = t - t_dec_start;
//             a = -a_abs;
//             v = v_peak - a_abs * t_dec;
//             float s_acc = 0.5f * a_abs * t_acc * t_acc;
//             float s_const = v_peak * t_const;
//             float s_dec = v_peak * t_dec - 0.5f * a_abs * t_dec * t_dec;
//             s = s_acc + s_const + s_dec;
//         }
//     }

//     // 数值安全清理
//     if (fabsf(v) < 1e-6f) v = 0.0f;
//     if (fabsf(a) < 1e-6f) a = 0.0f;

//     /* ---------- 根据弧长计算当前角度 ---------- */
//     // 运动方向由 sweep_angle 的符号决定：正为逆时针，负为顺时针
//     float direction = (arc->sweep_angle > 0) ? 1.0f : -1.0f;
//     float angle = arc->start_angle + direction * (s / arc->r);

//     /* ---------- 计算笛卡尔坐标，存入结构体 ---------- */
//     arm->x = arc->cx + arc->r * cosf(angle);
//     arm->y = arc->cy + arc->r * sinf(angle);

//     /* ---------- 计算切向单位向量 ---------- */
//     // 对于逆时针圆弧 (direction=+1)，切向为 (-sin, cos)
//     // 对于顺时针圆弧 (direction=-1)，切向为 (sin, -cos)
//     // 统一公式：tx = -sin(angle) * direction, ty = cos(angle) * direction
//     float tx = -sinf(angle) * direction;
//     float ty =  cosf(angle) * direction;

//     // 速度 = 切向速度大小 × 切向单位向量
//     arm->dx = v * tx;
//     arm->dy = v * ty;

//     /* ---------- 计算加速度 ---------- */
//     // 法向（向心）加速度大小 an = v^2 / r，方向指向圆心
//     float an = v * v / arc->r;
//     // 法向单位向量 = 从圆弧上点指向圆心 = (-cos, -sin)
//     float nx = -cosf(angle);
//     float ny = -sinf(angle);

//     // 总加速度 = 切向加速度 × 切向向量 + 法向加速度 × 法向向量
//     arm->ddx = a * tx + an * nx;
//     arm->ddy = a * ty + an * ny;
// }

/**
 * 求解两杆机械臂的逆运动学，并计算关节角速度和角加速度
 * @param params 包含杆长、末端位置、速度、加速度的结构体指针
 * @return 1=成功，0=无解或奇异
 */
uint8_t BlockArm_BackSolve(ARM_BACKSOLVING *params)
{
    // 1. 逆运动学求解关节角度 theta1, theta2
    float d = sqrtf(params->x * params->x + params->y * params->y);
    if (d > params->L1 + params->L2 + EPS || d < fabsf(params->L1 - params->L2) - EPS) 
    {
        return 0; // 不可达
    }

    float beta = atan2f(params->y, params->x);
    float cos_alpha = (params->L1 * params->L1 + d * d - params->L2 * params->L2) / (2.0f * params->L1 * d);
    if (cos_alpha > 1.0f) cos_alpha = 1.0f;
    if (cos_alpha < -1.0f) cos_alpha = -1.0f;
    float alpha = acosf(cos_alpha);

    float theta1_cand[2] = { beta + alpha, beta - alpha };
    int found = 0;
    float theta1_rad = 0, theta2_rad = 0;

    for (int i = 0; i < 2; i++) 
    {
        float th1 = theta1_cand[i];
        if (th1 < 0.0f || th1 >  PI ) continue;

        float x1 = params->L1 * cosf(th1);
        float y1 = params->L1 * sinf(th1);
        float dx = params->x - x1;
        float dy = params->y - y1;
        float th2 = atan2f(dy, dx);

        if (th2 < -PI/2.0f || th2 > PI/2.0f) continue;
        if (th2 > th1 + EPS) continue;

        theta1_rad = th1;
        theta2_rad = th2;
        found = 1;
        break;
    }
    if (!found) return 0;

    // 转换为角度制存储
    params->theta1 = theta1_rad * DEG;
    params->theta2 = theta2_rad * DEG;

    // 2. 计算雅可比矩阵 J 及其行列式
    float J11 = -params->L1 * sinf(theta1_rad);
    float J12 = -params->L2 * sinf(theta2_rad);
    float J21 =  params->L1 * cosf(theta1_rad);
    float J22 =  params->L2 * cosf(theta2_rad);
    float det = J11 * J22 - J12 * J21;   // = L1*L2*sin(theta2 - theta1)

    if (fabsf(det) < EPS) 
    {
        return 0;   // 奇异位形，无法求逆
    }
    float inv_det = 1.0f / det;

    // 3. 由末端速度求关节角速度 (rad/s)
    float vx = params->dx;   // 注意单位：mm/s
    float vy = params->dy;
    float dtheta1_rad = inv_det * ( J22 * vx - J12 * vy);
    float dtheta2_rad = inv_det * (-J21 * vx + J11 * vy);

    // 存储角速度 (度/秒)
    params->dtheta1 = dtheta1_rad * DEG;
    params->dtheta2 = dtheta2_rad * DEG;

    // 4. 计算 \dot{J} 矩阵
    float dJ11 = -params->L1 * cosf(theta1_rad) * dtheta1_rad;
    float dJ12 = -params->L2 * cosf(theta2_rad) * dtheta2_rad;
    float dJ21 = -params->L1 * sinf(theta1_rad) * dtheta1_rad;
    float dJ22 = -params->L2 * sinf(theta2_rad) * dtheta2_rad;

    // 5. 由末端加速度求关节角加速度 (rad/s?)
    float ax = params->ddx;
    float ay = params->ddy;
    float rhs_x = ax - (dJ11 * dtheta1_rad + dJ12 * dtheta2_rad);
    float rhs_y = ay - (dJ21 * dtheta1_rad + dJ22 * dtheta2_rad);
    float ddtheta1_rad = inv_det * ( J22 * rhs_x - J12 * rhs_y);
    float ddtheta2_rad = inv_det * (-J21 * rhs_x + J11 * rhs_y);

    // 存储角加速度 (度/秒?)
    params->ddtheta1 = ddtheta1_rad * DEG;
    params->ddtheta2 = ddtheta2_rad * DEG;

    return 1;
}

/**
 * @brief 初始化带初末速度的笛卡尔直线轨迹
 * @param traj   轨迹结构体指针
 * @param xs,ys  起点坐标 (mm)
 * @param xe,ye  终点坐标 (mm)
 * @param Vs     初速度大小 (mm/s)，非负
 * @param Ve     末速度大小 (mm/s)，非负
 * @param V_max  最大允许速度 (mm/s)
 * @param a_max  最大允许加速度 (mm/s?)
 * @return 1=成功，0=失败（运动中或参数无效）
 */
uint8_t Traj_Line_Init(Trajectory* traj,
                          float xs, float ys, float xe, float ye,
                          float Vs, float Ve,
                          float V_max, float a_max)
{
    // 运动中禁止初始化
    if (traj->State == TRAJ_ACCEL || traj->State == TRAJ_CONST || traj->State == TRAJ_DECEL) {
        return 0;
    }

    traj->Type = TRAJ_TYPE_LINE;
    traj->State = TRAJ_IDLE;

    LineTraj* line = &traj->Data.Line;

    // 基础几何参数
    line->xs = xs;   line->ys = ys;
    line->xe = xe;   line->ye = ye;
    line->V_max = fabsf(V_max);
    line->a_max = fabsf(a_max);
    line->Vs = fmaxf(0.0f, fminf(Vs, line->V_max));
    line->Ve = fmaxf(0.0f, fminf(Ve, line->V_max));
    line->t_last = 0.0f;

    float dx = xe - xs;
    float dy = ye - ys;
    line->L = sqrtf(dx*dx + dy*dy);

    if (line->L < 1e-6f) {
        // 零位移
        line->Vp = 0.0f;
        line->t_acc = line->t_const = line->t_dec = 0.0f;
        line->T_total = 0.0f;
        line->ux = line->uy = 0.0f;
        return 1;
    }

    line->ux = dx / line->L;
    line->uy = dy / line->L;

    float a = line->a_max;
    float L = line->L;
    float vs = line->Vs;
    float ve = line->Ve;
    float vmax = line->V_max;

    // 计算从 vs 加速到 vmax 所需的时间和位移
    float t_a = (vmax - vs) / a;
    float s_a = (vs + vmax) * 0.5f * t_a;

    // 计算从 vmax 减速到 ve 所需的时间和位移
    float t_d = (vmax - ve) / a;
    float s_d = (vmax + ve) * 0.5f * t_d;

    // 判断能否达到 vmax
    if (s_a + s_d <= L) {
        // 梯形曲线（有匀速段）
        line->Vp = vmax;
        line->t_acc = t_a;
        line->t_dec = t_d;
        line->t_const = (L - s_a - s_d) / vmax;
        line->T_total = line->t_acc + line->t_const + line->t_dec;
    } else {
        // 无法达到 vmax，需计算实际峰值速度 Vp（三角形曲线）
        // 解方程：L = (Vp^2 - vs^2)/(2a) + (Vp^2 - ve^2)/(2a)
        // 即 2aL = 2Vp^2 - (vs^2 + ve^2)  => Vp = sqrt( aL + (vs^2+ve^2)/2 )
        float vp_sqr = a * L + (vs*vs + ve*ve) * 0.5f;
        float vp = sqrtf(fmaxf(0.0f, vp_sqr));
        // 检查 vp 是否超过 vmax（理论上不会，但浮点误差可能导致）
        if (vp > vmax + 1e-3f) {
            // 参数不合理，回退到 vmax 并调整时间（可忽略）
            vp = vmax;
        }
        line->Vp = vp;
        line->t_acc = (vp - vs) / a;
        line->t_dec = (vp - ve) / a;
        line->t_const = 0.0f;
        line->T_total = line->t_acc + line->t_dec;

        // 修正可能为负的时间（当 vs > vp 或 ve > vp 时）
        if (line->t_acc < 0) line->t_acc = 0;
        if (line->t_dec < 0) line->t_dec = 0;
    }

    // 防止除零等异常
    if (line->t_acc < 1e-6f) line->t_acc = 0.0f;
    if (line->t_dec < 1e-6f) line->t_dec = 0.0f;
    if (line->t_const < 1e-6f) line->t_const = 0.0f;

    return 1;
}

void Traj_Line_Update(Trajectory* traj, ARM_BACKSOLVING* arm)
{
    if (traj->Type != TRAJ_TYPE_LINE || traj->State == TRAJ_NOINIT || traj->State == TRAJ_DONE)
        return;

    LineTraj* line = &traj->Data.Line;
    float t = line->t_last;

    if (t <= 0.0f) {
        arm->x = line->xs; arm->y = line->ys;
        arm->dx = line->Vs * line->ux;
        arm->dy = line->Vs * line->uy;
        arm->ddx = arm->ddy = 0.0f;
        traj->State = TRAJ_IDLE;
        return;
    }
    if (t >= line->T_total) {
        arm->x = line->xe; arm->y = line->ye;
        arm->dx = line->Ve * line->ux;
        arm->dy = line->Ve * line->uy;
        arm->ddx = arm->ddy = 0.0f;
        traj->State = TRAJ_DONE;
        return;
    }

    float a_abs = line->a_max;
    float vs = line->Vs;
    float ve = line->Ve;
    float vp = line->Vp;
    float t_acc = line->t_acc;
    float t_const = line->t_const;
    float s, v, a;

    // 时间阶段划分
    float t1 = t_acc;
    float t2 = t_acc + t_const;

    if (t <= t1) {
        // 加速段（可能为减速，若 vs > vp 则加速度为负）
        traj->State = TRAJ_ACCEL;
        float acc_sign = (vp >= vs) ? 1.0f : -1.0f;
        a = acc_sign * a_abs;
        v = vs + a * t;
        s = vs * t + 0.5f * a * t * t;
    } else if (t <= t2) {
        // 匀速段
        traj->State = TRAJ_CONST;
        a = 0.0f;
        v = vp;
        s = vs * t1 + 0.5f * (vp - vs) * t1 + vp * (t - t1);
    } else {
        // 减速段
        traj->State = TRAJ_DECEL;
        float t_dec_elapsed = t - t2;
        float acc_sign = (ve >= vp) ? 1.0f : -1.0f;
        a = acc_sign * a_abs;
        v = vp + a * t_dec_elapsed;
        // 位移：前两段位移 + 当前减速段位移
        float s1 = vs * t1 + 0.5f * (vp - vs) * t1;
        float s2 = vp * t_const;
        float s3 = vp * t_dec_elapsed + 0.5f * a * t_dec_elapsed * t_dec_elapsed;
        s = s1 + s2 + s3;
    }

    // 数值清理
    if (fabsf(v) < 1e-6f) v = 0.0f;
    if (fabsf(a) < 1e-6f) a = 0.0f;

    // 输出笛卡尔量
    arm->x = line->xs + s * line->ux;
    arm->y = line->ys + s * line->uy;
    arm->dx = v * line->ux;
    arm->dy = v * line->uy;
    arm->ddx = a * line->ux;
    arm->ddy = a * line->uy;
}

uint8_t Traj_Arc_Init(Trajectory* traj,
                         float cx, float cy, float r,
                         float start_angle, float sweep_angle,
                         float Vs, float Ve,
                         float V_max, float a_max)
{
    if (traj->State == TRAJ_ACCEL || traj->State == TRAJ_CONST || traj->State == TRAJ_DECEL)
        return 0;

    traj->Type = TRAJ_TYPE_ARC;
    traj->State = TRAJ_IDLE;

    ArcTraj* arc = &traj->Data.Arc;
    arc->cx = cx;
    arc->cy = cy;
    arc->r = fabsf(r);
    arc->start_angle = start_angle * RADIAN;
    arc->sweep_angle = sweep_angle * RADIAN;
    arc->V_max = fabsf(V_max);
    arc->a_max = fabsf(a_max);
    arc->Vs = fmaxf(0.0f, fminf(Vs, arc->V_max));
    arc->Ve = fmaxf(0.0f, fminf(Ve, arc->V_max));
    arc->t_last = 0.0f;

    arc->L = arc->r * fabsf(arc->sweep_angle);
    if (arc->L < 1e-6f) {
        arc->Vp = arc->t_acc = arc->t_const = arc->t_dec = arc->T_total = 0.0f;
        return 1;
    }

    // 速度规划（与直线完全相同）
    float a = arc->a_max;
    float L = arc->L;
    float vs = arc->Vs;
    float ve = arc->Ve;
    float vmax = arc->V_max;

    float t_a = (vmax - vs) / a;
    float s_a = (vs + vmax) * 0.5f * t_a;
    float t_d = (vmax - ve) / a;
    float s_d = (vmax + ve) * 0.5f * t_d;

    if (s_a + s_d <= L) 
	{
        arc->Vp = vmax;
        arc->t_acc = t_a;
        arc->t_dec = t_d;
        arc->t_const = (L - s_a - s_d) / vmax;
        arc->T_total = arc->t_acc + arc->t_const + arc->t_dec;
    } 
	else 
	{
        float vp_sqr = a * L + (vs*vs + ve*ve) * 0.5f;
        float vp = sqrtf(fmaxf(0.0f, vp_sqr));
        if (vp > vmax + 1e-3f) vp = vmax;
        arc->Vp = vp;
        arc->t_acc = (vp - vs) / a;
        arc->t_dec = (vp - ve) / a;
        arc->t_const = 0.0f;
        arc->T_total = arc->t_acc + arc->t_dec;
        if (arc->t_acc < 0) arc->t_acc = 0;
        if (arc->t_dec < 0) arc->t_dec = 0;
    }

    if (arc->t_acc < 1e-6f) arc->t_acc = 0.0f;
    if (arc->t_dec < 1e-6f) arc->t_dec = 0.0f;
    if (arc->t_const < 1e-6f) arc->t_const = 0.0f;

    return 1;
}

void Traj_Arc_Update(Trajectory* traj, ARM_BACKSOLVING* arm)
{
    if (traj->Type != TRAJ_TYPE_ARC || traj->State == TRAJ_NOINIT || traj->State == TRAJ_DONE)
        return;

    ArcTraj* arc = &traj->Data.Arc;
    float t = arc->t_last;

    // 边界处理
    if (t <= 0.0f) {
        float ang = arc->start_angle;
        arm->x = arc->cx + arc->r * cosf(ang);
        arm->y = arc->cy + arc->r * sinf(ang);
        float dir = (arc->sweep_angle > 0) ? 1.0f : -1.0f;
        float tx = -sinf(ang) * dir;
        float ty =  cosf(ang) * dir;
        arm->dx = arc->Vs * tx;
        arm->dy = arc->Vs * ty;
        arm->ddx = arm->ddy = 0.0f;
        traj->State = TRAJ_IDLE;
        return;
    }
    if (t >= arc->T_total) {
        float ang = arc->start_angle + arc->sweep_angle;
        arm->x = arc->cx + arc->r * cosf(ang);
        arm->y = arc->cy + arc->r * sinf(ang);
        float dir = (arc->sweep_angle > 0) ? 1.0f : -1.0f;
        float tx = -sinf(ang) * dir;
        float ty =  cosf(ang) * dir;
        arm->dx = arc->Ve * tx;
        arm->dy = arc->Ve * ty;
        arm->ddx = arm->ddy = 0.0f;
        traj->State = TRAJ_DONE;
        return;
    }

    float a_abs = arc->a_max;
    float vs = arc->Vs;
    float ve = arc->Ve;
    float vp = arc->Vp;
    float t_acc = arc->t_acc;
    float t_const = arc->t_const;
    float s, v, a;

    float t1 = t_acc;
    float t2 = t_acc + t_const;

    if (t <= t1) {
        traj->State = TRAJ_ACCEL;
        float sign = (vp >= vs) ? 1.0f : -1.0f;
        a = sign * a_abs;
        v = vs + a * t;
        s = vs * t + 0.5f * a * t * t;
    } else if (t <= t2) {
        traj->State = TRAJ_CONST;
        a = 0.0f;
        v = vp;
        s = vs * t1 + 0.5f * (vp - vs) * t1 + vp * (t - t1);
    } else {
        traj->State = TRAJ_DECEL;
        float t_dec_elapsed = t - t2;
        float sign = (ve >= vp) ? 1.0f : -1.0f;
        a = sign * a_abs;
        v = vp + a * t_dec_elapsed;
        float s1 = vs * t1 + 0.5f * (vp - vs) * t1;
        float s2 = vp * t_const;
        float s3 = vp * t_dec_elapsed + 0.5f * a * t_dec_elapsed * t_dec_elapsed;
        s = s1 + s2 + s3;
    }

    if (fabsf(v) < 1e-6f) v = 0.0f;
    if (fabsf(a) < 1e-6f) a = 0.0f;

    // 根据弧长计算角度
    float dir = (arc->sweep_angle > 0) ? 1.0f : -1.0f;
    float angle = arc->start_angle + dir * (s / arc->r);

    arm->x = arc->cx + arc->r * cosf(angle);
    arm->y = arc->cy + arc->r * sinf(angle);

    // 切向单位向量
    float tx = -sinf(angle) * dir;
    float ty =  cosf(angle) * dir;

    arm->dx = v * tx;
    arm->dy = v * ty;

    // 向心加速度
    float an = v * v / arc->r;
    float nx = -cosf(angle);
    float ny = -sinf(angle);

    arm->ddx = a * tx + an * nx;
    arm->ddy = a * ty + an * ny;
}
