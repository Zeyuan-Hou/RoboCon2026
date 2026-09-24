#include "ramp_task.h"

BezierPoint P0;
BezierPoint P1;
BezierPoint P2;
BezierPoint Point_Ramp; //实时理论位置
BezierVel Velt_Ramp; //实时理论速度
Bezier bezier_use;
float ramp_speed = 10.0f;
/****************************************贝塞尔曲线****************************************/
// 初始化
void Bezier_init(Bezier *bezier, BezierPoint p0, BezierPoint p1, BezierPoint p2, float cruise_speed)
{
    if (cruise_speed < 10.0f) {
        cruise_speed = 10.0f;
    }
    bezier->p0 = p0;
    bezier->p1 = p1;
    bezier->p2 = p2;
    float length = 0.0f;
    BezierPoint prev = p0;
    bezier->velt = cruise_speed;
    for (int i = 0; i < 100; i++)
    {
        float t = (float)i / 100.0f;
        float mt = 1.0f - t;
        BezierPoint curr;
        curr.x = mt * mt * p0.x + 2.0f * t * mt * p1.x + t * t * p2.x;
        curr.y = mt * mt * p0.y + 2.0f * t * mt * p1.y + t * t * p2.y;
        length += sqrtf(powf(curr.x - prev.x, 2) + powf(curr.y - prev.y, 2));
        prev = curr;
    }
    bezier->length = length;
    bezier->total_time = bezier->length / cruise_speed * 1000.f;//ms
}
// 计算每一点的位置和速度
void Bezier_calc_by_time(const Bezier *bezier, uint32_t current_time, BezierPoint *position, BezierVel *velocity)
{
    if (current_time > bezier->total_time)
    {
        current_time = bezier->total_time;
    }
    float t = (float)current_time / bezier->total_time;
    if (t < 0.0f)
        t = 0.0f;
    if (t > 1.0f)
        t = 1.0f;
    float mt = 1.0f - t;
    // 计算位置
    position->x = mt * mt * bezier->p0.x + 2.0f * t * mt * bezier->p1.x + t * t * bezier->p2.x;
    position->y = mt * mt * bezier->p0.y + 2.0f * t * mt * bezier->p1.y + t * t * bezier->p2.y;
    // 计算速度
    float dx = 2.0f * mt * (bezier->p1.x - bezier->p0.x) + 2.0f * t * (bezier->p2.x - bezier->p1.x);
    float dy = 2.0f * mt * (bezier->p1.y - bezier->p0.y) + 2.0f * t * (bezier->p2.y - bezier->p1.y);

    velocity->magnitude = bezier->velt; // 默认匀速
    velocity->angle = atan2f(dy, dx);
    velocity->vx = velocity->magnitude * cosf(velocity->angle);//与车身角度有关？
    velocity->vy = velocity->magnitude * sinf(velocity->angle);
}
/***************************************恒功率控制*********************************************/
// 计算单个轮子的功率
static float calc_single_wheel_power(float fpu, float wheel_speed, float target_wheel_speed)
{
    float abs_fpu = fabsf(fpu);
    float abs_speed = fabsf(wheel_speed);
    float abs_target = fabsf(target_wheel_speed);

    if (abs_fpu > FPU_SATURATION)
    {
        float speed_error = abs_target - abs_speed;
        if (speed_error > 0.0f)
        {
            return 15000.0f * (abs_speed + speed_error * 0.5f);
        }
        else
        {
            return abs_fpu * (abs_speed + speed_error * 0.3f);
        }
    }
    else
    {
        return abs_fpu * abs_speed;
    }
}
float speed_adjust = 1.0f; // 最终速度调整系数
float total_power = 0.0f;  // 总等效功率（调试用）
float power_error = 0.0f;  // 功率误差（调试用）
float power_Kp = 0.1f;
float power_Ki = 3.0f;
float power_integral = 0.0f; // 积分项
// 恒功率控制
void constant_power_control(void)
{
    // 1.获取4个轮子参数
    float target_wheel_speed[4];
    float current_wheel_speed[4];
    float fpu[4];
    SpeedDistribute_Four_OmnidriectionalWhile(&nav.expect_robot_global_velt); // 计算每个轮子的期望速度
    target_wheel_speed[0] = chassis_run.wheel_1.fpDes;current_wheel_speed[0] = chassis_run.wheel_1.fpFB;fpu[0] = chassis_run.wheel_1.fpU;
    target_wheel_speed[1] = chassis_run.wheel_2.fpDes;current_wheel_speed[1] = chassis_run.wheel_2.fpFB;fpu[1] = chassis_run.wheel_2.fpU;
    target_wheel_speed[2] = chassis_run.wheel_3.fpDes;current_wheel_speed[2] = chassis_run.wheel_3.fpFB;fpu[2] = chassis_run.wheel_3.fpU;
    target_wheel_speed[3] = chassis_run.wheel_4.fpDes;current_wheel_speed[3] = chassis_run.wheel_4.fpFB;fpu[3] = chassis_run.wheel_4.fpU;
    // 2.计算整车等效功率
    total_power = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        total_power += calc_single_wheel_power(fpu[i], current_wheel_speed[i], target_wheel_speed[i]);
    }
    // 3.计算整车功率误差
    power_error = TARGET_POWER - total_power;
    // 4.PI运算
    float p_term = power_Kp * power_error;
    float i_term = 0.0f;
    // 抗积分饱和：只有当速度调整在限幅范围内时，才进行积分
    if (speed_adjust > SPEED_ADJUST_MIN && speed_adjust < SPEED_ADJUST_MAX)
    {
        power_integral += power_error * 0.001f * power_Ki; // 0.001是1ms周期转换
        // 积分限幅
        power_integral = fmaxf(fminf(power_integral, POWER_I_MAX), -POWER_I_MAX);
        i_term = power_integral;
    }
    else
    {
        // 达到限幅时，缓慢释放积分，防止积分饱和
        power_integral *= 0.99f;
    }
    // 5.计算最终速度调整系数
    speed_adjust = 1.0f + p_term + i_term;
    // 限幅
    speed_adjust = fmaxf(fminf(speed_adjust, SPEED_ADJUST_MAX), SPEED_ADJUST_MIN);
    // 6.更新期望速度
    nav.expect_robot_global_velt.fpX *= speed_adjust;
    nav.expect_robot_global_velt.fpY *= speed_adjust;
    nav.expect_robot_global_velt.fpW *= speed_adjust;
}
/**********************************************总状态机*************************************************/
//uint8_t ramp_state = 0;
float preload_factor=1.0f;// 预加载系数
void ramp_task(void)
{
    static uint8_t nav_inited = 0;
    static uint8_t prev_state = -1;
    static uint32_t time_ramp=0;

    if (ramp_state!= prev_state)/* 状态变化，重置标志 */
    {
        prev_state = ramp_state;
        nav_inited = 0;
        time_ramp=0;
    }
    switch (ramp_state)
    {
    case 0: // 根据出口走第一段路径
        if (!nav_inited)
        {
            nav.auto_path.number_point = vision_data_recieve.path_number + 23;
            nav_reach_state = 4;                                               // 时间停止
            nav.nav_state = NAV_POINT_TO_POINT;
            nav_inited = 1;
            if (nav.auto_path.number_point == 26 || nav.auto_path.number_point == 25)
            {
								P0.x=MELIN_X_11;
								P0.y=MELIN_Y_11;
                P2.x = RAMP_STAER_x_11; //-3450.f;//1 2 -3500.f;//1 -3593.20874f; //2 -3435;//3 -3450.f;//-3450.f 2号;// // 记得测！！;！
                P2.y = RAMP_STAER_Y; //-7470.771f;//1 2 -7408;//3 -7470.771f;
            }
            else if (nav.auto_path.number_point == 24)
            {
								P0.x=MELIN_X_10;
								P0.y=MELIN_Y_10;
                P2.x = RAMP_STAER_X_10; //-3500.f;;//1 2 -3500.f;//1 -3593.20874f; //2 -3435;//3 -3450.f;//-3450.f 2号;// // 记得测！！;！
                P2.y = RAMP_STAER_Y; //-7408;//1 2 -7408;//3 -7470.771f;
            }
						//chassis_run.wheel_1.fpKp=220.f;chassis_run.wheel_2.fpKp=220.f;chassis_run.wheel_3.fpKp=220.f;chassis_run.wheel_4.fpKp=220.f;
						
        }
        if (nav.nav_state == NAV_RAMP)
        {

            ramp_state = 1;
        }
        break;
    case 1: // 根据第一段结束的速度走贝塞尔曲线到达台阶前，同时预加载
        if (!nav_inited)
        {
            P1.x = P2.x;
            P1.y = P0.y;

            Bezier_init(&bezier_use, P0, P1, P2, RAMP_CRUISE_SPEED);
            nav_inited = 1;
            time_ramp = 0;
        }
        Bezier_calc_by_time(&bezier_use, time_ramp, &Point_Ramp, &Velt_Ramp); // 计算每一时刻理论位置速度
        time_ramp++;
        nav.auto_path.pos_pid.pid_x.fpDes = Point_Ramp.x;
        nav.auto_path.pos_pid.pid_y.fpDes = Point_Ramp.y;
        nav.auto_path.pos_pid.pid_w.fpDes = MELIN_ANGLE_11;       //改！！！
        PID_Calc(&nav.auto_path.pos_pid.pid_x, nav.auto_path.pos_pid.pid_x.fpDes, nav.auto_path.pos_pid.pid_x.fpFB);
        PID_Calc(&nav.auto_path.pos_pid.pid_y, nav.auto_path.pos_pid.pid_y.fpDes, nav.auto_path.pos_pid.pid_y.fpFB);
        PID_Calc_Angle(&nav.auto_path.pos_pid.pid_w, nav.auto_path.pos_pid.pid_w.fpDes, nav.auto_path.pos_pid.pid_w.fpFB);

        nav.expect_robot_global_velt.fpX = 0.9*Velt_Ramp.vx + nav.auto_path.pos_pid.pid_x.fpU ;
        nav.expect_robot_global_velt.fpY = 0.9*Velt_Ramp.vy + nav.auto_path.pos_pid.pid_y.fpU ;
        nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.pid_w.fpU;

         if (time_ramp >= bezier_use.total_time) // 准备上坡
        {
            nav.auto_path.pos_pid.pid_x.fpSumE = 0;
            nav.auto_path.pos_pid.pid_y.fpSumE = 0;
            nav.auto_path.pos_pid.pid_w.fpSumE = 0;
            ramp_state = 2;
            speed_adjust = 1.0f;
            power_integral = 0.0f;
        }

        break;
    case 2: // 恒功率控制上坡，其中可加动态重心补偿
        if (!nav_inited)
        {
            //nav.auto_path.pos_pid.pid_x.fpKp = 0.8f; // 改！！！！
            nav_inited = 1;
						nav.auto_path.pos_pid.pid_x.fpDes =RAMP_STAER_x_11; // 改！！！
				}
        time_ramp++;
				
        
        nav.auto_path.pos_pid.pid_y.fpDes = P2.y - RAMP_CRUISE_SPEED * (float)time_ramp / 1000.0f*cos(15.f*PI/180.f);//实时
        nav.auto_path.pos_pid.pid_w.fpDes = MELIN_ANGLE_11;     //改！！！

        PID_Calc(&nav.auto_path.pos_pid.pid_x, nav.auto_path.pos_pid.pid_x.fpDes, nav.auto_path.pos_pid.pid_x.fpFB);
        PID_Calc(&nav.auto_path.pos_pid.pid_y, nav.auto_path.pos_pid.pid_y.fpDes, nav.auto_path.pos_pid.pid_y.fpFB);
        PID_Calc_Angle(&nav.auto_path.pos_pid.pid_w, nav.auto_path.pos_pid.pid_w.fpDes, nav.auto_path.pos_pid.pid_w.fpFB);

        nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.pid_x.fpU;
        nav.expect_robot_global_velt.fpY = -RAMP_CRUISE_SPEED +nav.auto_path.pos_pid.pid_y.fpU ;
        nav.expect_robot_global_velt.fpW =	nav.auto_path.pos_pid.pid_w.fpU ;//-0.3f+0.12*nav.auto_path.pos_pid.pid_w.fpU;      //改！！！

        nav.expect_robot_global_velt.fpX *= preload_factor; // 重力前馈
        nav.expect_robot_global_velt.fpY *= 1;
        nav.expect_robot_global_velt.fpW *= preload_factor;
				

        if (nav.auto_path.pos_pid.pid_y.fpFB < RAMP_END_Y) // 到达台阶前20cm -9260.62109
        {
            // preload_factor = 1.0f;
             //nav.auto_path.pos_pid.pid_x.fpKp = 0.8f; // 解除强力锁X
            ramp_state = 3;
            //						nav.nav_state = NAV_LOCK;
            //						flag_lock =1;
            nav.auto_path.pos_pid.pid_x.fpSumE = 0;
            nav.auto_path.pos_pid.pid_y.fpSumE = 0;
            nav.auto_path.pos_pid.pid_w.fpSumE = 0;
        }
        break;
    case 5:
        if (!nav_inited)
        {
            nav.nav_state = NAV_LOCK;

            flag_lock = 1;
            nav.auto_path.pos_pid.pid_x.fpSumE = 0;
            nav.auto_path.pos_pid.pid_y.fpSumE = 0;
            nav.auto_path.pos_pid.pid_w.fpSumE = 0;
        }
        break;
    case 3: // 根据贝塞尔曲线再次转向
        if (!nav_inited)
        {
            P0.x = RAMP_STAER_x_11; // 根据实际XY再做改变
            P0.y = RAMP_END_Y ;
            P2.x = RAMP_CURVE_END_X;//-1485.62268f; // 记得测！！！
            P2.y = RAMP_CURVE_END_Y;
            P1.x = P0.x;
            P1.y = P2.y;

            Bezier_init(&bezier_use, P0, P1, P2, RAMP_CRUISE_SPEED);
            nav_inited = 1;
            time_ramp = 0;
        }
				
				
        Bezier_calc_by_time(&bezier_use, time_ramp, &Point_Ramp, &Velt_Ramp); // 计算每一时刻理论位置速度
        time_ramp++;
        nav.auto_path.pos_pid.pid_x.fpDes = Point_Ramp.x;
        nav.auto_path.pos_pid.pid_y.fpDes = Point_Ramp.y;
        nav.auto_path.pos_pid.pid_w.fpDes = MELIN_ANGLE_11;
        PID_Calc(&nav.auto_path.pos_pid.pid_x, nav.auto_path.pos_pid.pid_x.fpDes, nav.auto_path.pos_pid.pid_x.fpFB);
        PID_Calc(&nav.auto_path.pos_pid.pid_y, nav.auto_path.pos_pid.pid_y.fpDes, nav.auto_path.pos_pid.pid_y.fpFB);
        PID_Calc_Angle(&nav.auto_path.pos_pid.pid_w, nav.auto_path.pos_pid.pid_w.fpDes, nav.auto_path.pos_pid.pid_w.fpFB);
				
			

        nav.expect_robot_global_velt.fpX = 0.9*Velt_Ramp.vx + nav.auto_path.pos_pid.pid_x.fpU ;
        nav.expect_robot_global_velt.fpY = 0.9*Velt_Ramp.vy+ nav.auto_path.pos_pid.pid_y.fpU ;
				nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.pid_w.fpU;


        if (time_ramp >= bezier_use.total_time)
        {
            inner_send[1] = 14;
            flag_lock = 1;
            nav.auto_path.pos_pid.pid_x.fpSumE = 0;
            nav.auto_path.pos_pid.pid_y.fpSumE = 0;
            nav.auto_path.pos_pid.pid_w.fpSumE = 0;
            ramp_state = 4;
            speed_adjust = 1.0f;
            power_integral = 0.0f;
        }
        break;
    case 4: //匀减速停止
        if (!nav_inited)
        {
            nav.auto_path.number_point = 28;
            nav_reach_state=7;
            nav.nav_state = NAV_POINT_TO_POINT;
            nav_inited = 1;
        }
        if (nav.nav_state == NAV_LOCK)
        {
            ramp_state = 6;
        }
        break;
    }
}
