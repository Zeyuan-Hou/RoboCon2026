#include "navigation.h"

float LineAccelStep = 20.f;
int flag_record = 1;
float StartX = 0.f, StartY = 0.f, StartQ = 0.f, t1=0.f, t2=0.f, t3=0.f, t_run=0.f, dt=0.001f;

void navigation(){
    switch (nav.nav_state)
    {
    case NAV_INIT:
        chassis_run.pid_state = VELT_LOOP;
        chassis_run.feed_forward_state = WITHOUT_FORWARD;
        // PID目标速度为0
        chassis_run.rightup.fpDes = 0;
        chassis_run.leftup.fpDes = 0;
        chassis_run.leftdown.fpDes = 0;
        chassis_run.rightdown.fpDes = 0;

        chassis_steer_angle.feed_forward_state = WITHOUT_FORWARD;
        chassis_steer_angle.leftup_td.aim = leftup_init_angle;
        chassis_steer_angle.rightup_td.aim = rightup_init_angle;
        chassis_steer_angle.leftdown_td.aim = leftdown_init_angle;
        chassis_steer_angle.rightdown_td.aim = rightdown_init_angle;
        break;

    case NAV_OFF:
        chassis_run.pid_state = OPEN_LOOP;
        chassis_run.feed_forward_state = WITHOUT_FORWARD;
        // PID输出为0
        chassis_run.rightup.fpU = 0;
        chassis_run.leftup.fpU = 0;
        chassis_run.leftdown.fpU = 0;
        chassis_run.rightdown.fpU = 0;
        break;

    case NAV_LOCK:
        chassis_run.pid_state = VELT_LOOP;
        chassis_run.feed_forward_state = WITHOUT_FORWARD;

        if (flag_lock == 1) // 记录下刚按下时的坐标，后续fpDes不会再更新
        {
            nav.auto_path.pos_pid.x.fpDes = nav.auto_path.pos_pid.x.fpFB;
            nav.auto_path.pos_pid.y.fpDes = nav.auto_path.pos_pid.y.fpFB;
            nav.auto_path.pos_pid.w.fpDes = nav.auto_path.pos_pid.w.fpFB;

            flag_lock = 0;
        }
        PID_Calc(&nav.auto_path.pos_pid.x, nav.auto_path.pos_pid.x.fpDes, nav.auto_path.pos_pid.x.fpFB);
        PID_Calc(&nav.auto_path.pos_pid.y, nav.auto_path.pos_pid.y.fpDes, nav.auto_path.pos_pid.y.fpFB);
        PID_Calc(&nav.auto_path.pos_pid.w, nav.auto_path.pos_pid.w.fpDes, nav.auto_path.pos_pid.w.fpFB);

        nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.x.fpU;
        nav.expect_robot_global_velt.fpY = nav.auto_path.pos_pid.y.fpU;
        nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU;

        SpeedDistribute_Four_OmnidirectionalWheel(&nav);
        break;

    case NAV_LOCAL_MANUAL:
        // chassis_run.pid_state = VELT_LOOP;
        // chassis_run.feed_forward_state = WITHOUT_FORWARD;

        // // CalculateVelocities(&Js_Value, &nav, 500, 10000, 500, 10000, 500, 8); // 这里算出来单位都是mm，这里手柄分配最大速度是3500mm/s，转速75°/s
        // CalculateVelocities(&Js_Value, &nav, 800, 1500, 800, 1500, 800, 50);

        // ramp_signal(&nav.expect_robot_global_velt.fpX, nav.auto_path.basic_velt.fpVx, LineAccelStep);
        // ramp_signal(&nav.expect_robot_global_velt.fpY, nav.auto_path.basic_velt.fpVy, LineAccelStep);
        // nav.expect_robot_global_velt.fpW = nav.auto_path.basic_velt.fpW;

        nav.expect_robot_global_velt.fpY =300, nav.expect_robot_global_velt.fpX = 00, nav.expect_robot_global_velt.fpW = 0;
        SpeedDistribute_Four_OmnidirectionalWheel(&nav); // yaw轴传入的是角度
        //SpeedDistribute_Four_SteeringWheel(&nav);
        break;

    case NAV_GLOBAL_MANUAL:
        // chassis_run.pid_state = VELT_LOOP;
        // chassis_run.feed_forward_state = WITHOUT_FORWARD;

        // CalculateVelocities(&Js_Value, &nav, 500, 2500, 500, 2500, 500, 75); // 这里算出来单位都是mm，这里手柄分配最大速度是3500mm/s，转速75°/s

        // ramp_signal(&nav.expect_robot_global_velt.fpX, nav.auto_path.basic_velt.fpVx, LineAccelStep);
        // ramp_signal(&nav.expect_robot_global_velt.fpY, nav.auto_path.basic_velt.fpVy, LineAccelStep);

        // if (flag_global_manual == 1){
        //     Chassis_Global_Yaw_Pid.fpDes = fpSumPosQ / 10; // 记录此时的车身旋转累计角度值
        //     flag_global_manual = 0;
        // }

        // if (fabs(nav.expect_robot_global_velt.fpW) > 10){
        //     Chassis_Global_Yaw_Pid.fpDes += 0.12 * Sgn(nav.expect_robot_global_velt.fpW);
        // }else{
        //     nav.auto_path.basic_velt.fpW = 0;
        // }
        // Chassis_Global_Yaw_Pid.fpFB = fpSumPosQ / 10;
        // PID_Calc(&Chassis_Global_Yaw_Pid, Chassis_Global_Yaw_Pid.fpDes, Chassis_Global_Yaw_Pid.fpFB);

        // nav.expect_robot_global_velt.fpW = Chassis_Global_Yaw_Pid.fpU + nav.auto_path.basic_velt.fpW;

        // SpeedDistribute_Four_OmnidirectionalWheel(&nav);
        // break;

    case NAV_AUTO_PATH:
        chassis_run.pid_state = VELT_LOOP;
        chassis_run.feed_forward_state = WITHOUT_FORWARD;

        switch (nav.auto_path.number)
        {
            case 1:
                if (flag_record) // 初始化加速度，运动时间，位移etc
                {
                    StartX = nav.auto_path.pos_pid.x.fpFB;
                    StartY = nav.auto_path.pos_pid.y.fpFB;
                    StartQ = 0;

                    nav.auto_path.pos_pid.x.fpSumE = 0;
                    nav.auto_path.pos_pid.y.fpSumE = 0;
                    nav.auto_path.pos_pid.w.fpSumE = 0;

                    delta_x = 4800 - StartX;
                    delta_y = 1000 - StartY;   
                    delta_q = 0 - StartQ;            

                    v_max = 1000;
                    a_max = 1000;

                    alpha = atan2f(delta_y, delta_x);
                    if (Geometric_mean(delta_x, delta_y) - pow(v_max, 2) / a_max > 0)
                    {
                        t1 = v_max / a_max;
                        t2 = (Geometric_mean(delta_x, delta_y) - t1 * t1 * a_max) / v_max;
                        t3 = t1;
                    }
                    else
                    {
                        t1 = sqrt(Geometric_mean(delta_x, delta_y) / a_max);
                        t2 = 0;
                        t3 = t1;
                    }
                    w = delta_q / (0.8f * (t1 + t2 + t3));

                    flag_record = 0;
                }

                t_run = dt * nav.auto_path.run_time;
                if (t_run < 0.8f * (t1 + t2 + t3))
                {
                    nav.auto_path.pos_pid.w.fpDes = StartQ + w * t_run;
                    nav.auto_path.basic_velt.fpW = w;
                }
                else if (t_run < t1 + t2 + t3 + 0.1f)
                {
                    nav.auto_path.pos_pid.w.fpDes = StartQ + delta_q;
                    nav.auto_path.basic_velt.fpW = 0;
                }
                else
                {
                    nav.auto_path.pos_pid.w.fpSumE = 0;
                }

                if (t_run < t1)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + a_max * cos(alpha) * pow(t_run, 2) / 2.f;
                    nav.auto_path.pos_pid.y.fpDes = StartY + a_max * sin(alpha) * pow(t_run, 2) / 2.f;
                    nav.auto_path.basic_velt.fpVx = a_max * cos(alpha) * t_run;
                    nav.auto_path.basic_velt.fpVy = a_max * sin(alpha) * t_run;
                }
                else if (t_run < t1 + t2)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + a_max * cos(alpha) * pow(t1, 2) / 2 + v_max * cos(alpha) * (t_run - t1);
                    nav.auto_path.pos_pid.y.fpDes = StartY + a_max * sin(alpha) * pow(t1, 2) / 2 + v_max * sin(alpha) * (t_run - t1);
                    nav.auto_path.basic_velt.fpVx = v_max * cos(alpha);
                    nav.auto_path.basic_velt.fpVy = v_max * sin(alpha);
                }
                else if (t_run < t1 + t2 + t3)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + delta_x - a_max * cos(alpha) * pow(t1 + t2 + t3 - t_run, 2) / 2.f;
                    nav.auto_path.pos_pid.y.fpDes = StartY + delta_y - a_max * sin(alpha) * pow(t1 + t2 + t3 - t_run, 2) / 2.f;
                    nav.auto_path.basic_velt.fpVx = a_max * cos(alpha) * (t1 + t2 + t3 - t_run);
                    nav.auto_path.basic_velt.fpVy = a_max * sin(alpha) * (t1 + t2 + t3 - t_run);
                }
                else if (t_run < t1 + t2 + t3 + 0.1f)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + delta_x;
                    nav.auto_path.pos_pid.y.fpDes = StartY + delta_y;
                    nav.auto_path.basic_velt.fpVx = 0;
                    nav.auto_path.basic_velt.fpVy = 0;
                }
                else
                {
                    nav.auto_path.run_time = 0;
                    flag_record = 1;
                    //nav.nav_state = NAV_OFF;
                    nav.auto_path.number = 2;

                    nav.auto_path.pos_pid.x.fpSumE = 0;
                    nav.auto_path.pos_pid.y.fpSumE = 0;
                    nav.auto_path.pos_pid.w.fpSumE = 0;
                }
                break;

            case 2:
                if (flag_record) // 初始化加速度，运动时间，位移etc
                {
                    StartX = nav.auto_path.pos_pid.x.fpFB;
                    StartY = nav.auto_path.pos_pid.y.fpFB;
                    StartQ = 0;

                    nav.auto_path.pos_pid.x.fpSumE = 0;
                    nav.auto_path.pos_pid.y.fpSumE = 0;
                    nav.auto_path.pos_pid.w.fpSumE = 0;

                    delta_x = 5300 - StartX;
                    delta_y = 7400 - StartY;
                    delta_q = 0 - StartQ;

                    v_max = 1000;
                    a_max = 500;

                    alpha = atan2f(delta_y, delta_x);
                    if (Geometric_mean(delta_x, delta_y) - pow(v_max, 2) / a_max > 0)
                    {
                        t1 = v_max / a_max;
                        t2 = (Geometric_mean(delta_x, delta_y) - t1 * t1 * a_max) / v_max;
                        t3 = t1;
                    }
                    else
                    {
                        t1 = sqrt(Geometric_mean(delta_x, delta_y) / a_max);
                        t2 = 0;
                        t3 = t1;
                    }
                    w = delta_q / (0.8f * (t1 + t2 + t3));

                    flag_record = 0;
                }

                t_run = dt * nav.auto_path.run_time;
                if (t_run < 0.8f * (t1 + t2 + t3))
                {
                    nav.auto_path.pos_pid.w.fpDes = StartQ + w * t_run;
                    nav.auto_path.basic_velt.fpW = w;
                }
                else if (t_run < t1 + t2 + t3 + 0.1f)
                {
                    nav.auto_path.pos_pid.w.fpDes = StartQ + delta_q;
                    nav.auto_path.basic_velt.fpW = 0;
                }
                else
                {
                    nav.auto_path.pos_pid.w.fpSumE = 0;
                }

                if (t_run < t1)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + a_max * cos(alpha) * pow(t_run, 2) / 2.f;
                    nav.auto_path.pos_pid.y.fpDes = StartY + a_max * sin(alpha) * pow(t_run, 2) / 2.f;
                    nav.auto_path.basic_velt.fpVx = a_max * cos(alpha) * t_run;
                    nav.auto_path.basic_velt.fpVy = a_max * sin(alpha) * t_run;
                }
                else if (t_run < t1 + t2)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + a_max * cos(alpha) * pow(t1, 2) / 2 + v_max * cos(alpha) * (t_run - t1);
                    nav.auto_path.pos_pid.y.fpDes = StartY + a_max * sin(alpha) * pow(t1, 2) / 2 + v_max * sin(alpha) * (t_run - t1);
                    nav.auto_path.basic_velt.fpVx = v_max * cos(alpha);
                    nav.auto_path.basic_velt.fpVy = v_max * sin(alpha);
                }
                else if (t_run < t1 + t2 + t3)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + delta_x - a_max * cos(alpha) * pow(t1 + t2 + t3 - t_run, 2) / 2.f;
                    nav.auto_path.pos_pid.y.fpDes = StartY + delta_y - a_max * sin(alpha) * pow(t1 + t2 + t3 - t_run, 2) / 2.f;
                    nav.auto_path.basic_velt.fpVx = a_max * cos(alpha) * (t1 + t2 + t3 - t_run);
                    nav.auto_path.basic_velt.fpVy = a_max * sin(alpha) * (t1 + t2 + t3 - t_run);
                }
                else if (t_run < t1 + t2 + t3 + 0.1f)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + delta_x;
                    nav.auto_path.pos_pid.y.fpDes = StartY + delta_y;
                    nav.auto_path.basic_velt.fpVx = 0;
                    nav.auto_path.basic_velt.fpVy = 0;
                }
                else
                {
                    nav.auto_path.run_time = 0;
                    flag_record = 1;
                    //nav.nav_state = NAV_OFF;
                    nav.auto_path.number = 3;

                    nav.auto_path.pos_pid.x.fpSumE = 0;
                    nav.auto_path.pos_pid.y.fpSumE = 0;
                    nav.auto_path.pos_pid.w.fpSumE = 0;
                }
                break;

            case 3:
                if (flag_record) // 初始化加速度，运动时间，位移etc
                {
                    StartX = nav.auto_path.pos_pid.x.fpFB;
                    StartY = nav.auto_path.pos_pid.y.fpFB;
                    StartQ = 0;

                    nav.auto_path.pos_pid.x.fpSumE = 0;
                    nav.auto_path.pos_pid.y.fpSumE = 0;
                    nav.auto_path.pos_pid.w.fpSumE = 0;

                    delta_x = 900 - StartX;
                    delta_y = 7550 - StartY;
                    delta_q = 0 - StartQ;

                    v_max = 1000;
                    a_max = 500;

                    alpha = atan2f(delta_y, delta_x);
                    if (Geometric_mean(delta_x, delta_y) - pow(v_max, 2) / a_max > 0)
                    {
                        t1 = v_max / a_max;
                        t2 = (Geometric_mean(delta_x, delta_y) - t1 * t1 * a_max) / v_max;
                        t3 = t1;
                    }
                    else
                    {
                        t1 = sqrt(Geometric_mean(delta_x, delta_y) / a_max);
                        t2 = 0;
                        t3 = t1;
                    }
                    w = delta_q / (0.8f * (t1 + t2 + t3));

                    flag_record = 0;
                }

                t_run = dt * nav.auto_path.run_time;
                if (t_run < 0.8f * (t1 + t2 + t3))
                {
                    nav.auto_path.pos_pid.w.fpDes = StartQ + w * t_run;
                    nav.auto_path.basic_velt.fpW = w;
                }
                else if (t_run < t1 + t2 + t3 + 0.1f)
                {
                    nav.auto_path.pos_pid.w.fpDes = StartQ + delta_q;
                    nav.auto_path.basic_velt.fpW = 0;
                }
                else
                {
                    nav.auto_path.pos_pid.w.fpSumE = 0;
                }

                if (t_run < t1)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + a_max * cos(alpha) * pow(t_run, 2) / 2.f;
                    nav.auto_path.pos_pid.y.fpDes = StartY + a_max * sin(alpha) * pow(t_run, 2) / 2.f;
                    nav.auto_path.basic_velt.fpVx = a_max * cos(alpha) * t_run;
                    nav.auto_path.basic_velt.fpVy = a_max * sin(alpha) * t_run;
                }
                else if (t_run < t1 + t2)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + a_max * cos(alpha) * pow(t1, 2) / 2 + v_max * cos(alpha) * (t_run - t1);
                    nav.auto_path.pos_pid.y.fpDes = StartY + a_max * sin(alpha) * pow(t1, 2) / 2 + v_max * sin(alpha) * (t_run - t1);
                    nav.auto_path.basic_velt.fpVx = v_max * cos(alpha);
                    nav.auto_path.basic_velt.fpVy = v_max * sin(alpha);
                }
                else if (t_run < t1 + t2 + t3)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + delta_x - a_max * cos(alpha) * pow(t1 + t2 + t3 - t_run, 2) / 2.f;
                    nav.auto_path.pos_pid.y.fpDes = StartY + delta_y - a_max * sin(alpha) * pow(t1 + t2 + t3 - t_run, 2) / 2.f;
                    nav.auto_path.basic_velt.fpVx = a_max * cos(alpha) * (t1 + t2 + t3 - t_run);
                    nav.auto_path.basic_velt.fpVy = a_max * sin(alpha) * (t1 + t2 + t3 - t_run);
                }
                else if (t_run < t1 + t2 + t3 + 0.1f)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + delta_x;
                    nav.auto_path.pos_pid.y.fpDes = StartY + delta_y;
                    nav.auto_path.basic_velt.fpVx = 0;
                    nav.auto_path.basic_velt.fpVy = 0;
                }
                else
                {
                    nav.auto_path.run_time = 0;
                    flag_record = 1;
                    //nav.nav_state = NAV_OFF;
                    nav.auto_path.number = 4;

                    nav.auto_path.pos_pid.x.fpSumE = 0;
                    nav.auto_path.pos_pid.y.fpSumE = 0;
                    nav.auto_path.pos_pid.w.fpSumE = 0;
                }
                break;

            case 4:
                if (flag_record) // 初始化加速度，运动时间，位移etc
                {
                    StartX = nav.auto_path.pos_pid.x.fpFB;
                    StartY = nav.auto_path.pos_pid.y.fpFB;
                    StartQ = 0;

                    nav.auto_path.pos_pid.x.fpSumE = 0;
                    nav.auto_path.pos_pid.y.fpSumE = 0;
                    nav.auto_path.pos_pid.w.fpSumE = 0;

                    delta_x = 600 - StartX;
                    delta_y = 700 - StartY;
                    delta_q = 0 - StartQ;

                    v_max = 700;
                    a_max = 500;

                    alpha = atan2f(delta_y, delta_x);
                    if (Geometric_mean(delta_x, delta_y) - pow(v_max, 2) / a_max > 0)
                    {
                        t1 = v_max / a_max;
                        t2 = (Geometric_mean(delta_x, delta_y) - t1 * t1 * a_max) / v_max;
                        t3 = t1;
                    }
                    else
                    {
                        t1 = sqrt(Geometric_mean(delta_x, delta_y) / a_max);
                        t2 = 0;
                        t3 = t1;
                    }
                    w = delta_q / (0.8f * (t1 + t2 + t3));

                    flag_record = 0;
                }

                t_run = dt * nav.auto_path.run_time;
                if (t_run < 0.8f * (t1 + t2 + t3))
                {
                    nav.auto_path.pos_pid.w.fpDes = StartQ + w * t_run;
                    nav.auto_path.basic_velt.fpW = w;
                }
                else if (t_run < t1 + t2 + t3 + 0.1f)
                {
                    nav.auto_path.pos_pid.w.fpDes = StartQ + delta_q;
                    nav.auto_path.basic_velt.fpW = 0;
                }
                else
                {
                    nav.auto_path.pos_pid.w.fpSumE = 0;
                }

                if (t_run < t1)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + a_max * cos(alpha) * pow(t_run, 2) / 2.f;
                    nav.auto_path.pos_pid.y.fpDes = StartY + a_max * sin(alpha) * pow(t_run, 2) / 2.f;
                    nav.auto_path.basic_velt.fpVx = a_max * cos(alpha) * t_run;
                    nav.auto_path.basic_velt.fpVy = a_max * sin(alpha) * t_run;
                }
                else if (t_run < t1 + t2)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + a_max * cos(alpha) * pow(t1, 2) / 2 + v_max * cos(alpha) * (t_run - t1);
                    nav.auto_path.pos_pid.y.fpDes = StartY + a_max * sin(alpha) * pow(t1, 2) / 2 + v_max * sin(alpha) * (t_run - t1);
                    nav.auto_path.basic_velt.fpVx = v_max * cos(alpha);
                    nav.auto_path.basic_velt.fpVy = v_max * sin(alpha);
                }
                else if (t_run < t1 + t2 + t3)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + delta_x - a_max * cos(alpha) * pow(t1 + t2 + t3 - t_run, 2) / 2.f;
                    nav.auto_path.pos_pid.y.fpDes = StartY + delta_y - a_max * sin(alpha) * pow(t1 + t2 + t3 - t_run, 2) / 2.f;
                    nav.auto_path.basic_velt.fpVx = a_max * cos(alpha) * (t1 + t2 + t3 - t_run);
                    nav.auto_path.basic_velt.fpVy = a_max * sin(alpha) * (t1 + t2 + t3 - t_run);
                }
                else if (t_run < t1 + t2 + t3 + 0.1f)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + delta_x;
                    nav.auto_path.pos_pid.y.fpDes = StartY + delta_y;
                    nav.auto_path.basic_velt.fpVx = 0;
                    nav.auto_path.basic_velt.fpVy = 0;
                }
                else
                {
                    nav.auto_path.run_time = 0;
                    flag_record = 1;
                    //nav.nav_state = NAV_OFF;
                    nav.auto_path.number = 5;

                    nav.auto_path.pos_pid.x.fpSumE = 0;
                    nav.auto_path.pos_pid.y.fpSumE = 0;
                    nav.auto_path.pos_pid.w.fpSumE = 0;
                }
                break;

            case 5:
                if (flag_record) // 初始化加速度，运动时间，位移etc
                {
                    StartX = nav.auto_path.pos_pid.x.fpFB;
                    StartY = nav.auto_path.pos_pid.y.fpFB;
                    StartQ = 0;

                    nav.auto_path.pos_pid.x.fpSumE = 0;
                    nav.auto_path.pos_pid.y.fpSumE = 0;
                    nav.auto_path.pos_pid.w.fpSumE = 0;

                    delta_x = -250 - StartX;
                    delta_y = 750 - StartY;
                    delta_q = 0 - StartQ;

                    v_max = 400;
                    a_max = 400;

                    alpha = atan2f(delta_y, delta_x);
                    if (Geometric_mean(delta_x, delta_y) - pow(v_max, 2) / a_max > 0)
                    {
                        t1 = v_max / a_max;
                        t2 = (Geometric_mean(delta_x, delta_y) - t1 * t1 * a_max) / v_max;
                        t3 = t1;
                    }
                    else
                    {
                        t1 = sqrt(Geometric_mean(delta_x, delta_y) / a_max);
                        t2 = 0;
                        t3 = t1;
                    }
                    w = delta_q / (0.8f * (t1 + t2 + t3));

                    flag_record = 0;
                }

                t_run = dt * nav.auto_path.run_time;
                if (t_run < 0.8f * (t1 + t2 + t3))
                {
                    nav.auto_path.pos_pid.w.fpDes = StartQ + w * t_run;
                    nav.auto_path.basic_velt.fpW = w;
                }
                else if (t_run < t1 + t2 + t3 + 0.1f)
                {
                    nav.auto_path.pos_pid.w.fpDes = StartQ + delta_q;
                    nav.auto_path.basic_velt.fpW = 0;
                }
                else
                {
                    nav.auto_path.pos_pid.w.fpSumE = 0;
                }

                if (t_run < t1)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + a_max * cos(alpha) * pow(t_run, 2) / 2.f;
                    nav.auto_path.pos_pid.y.fpDes = StartY + a_max * sin(alpha) * pow(t_run, 2) / 2.f;
                    nav.auto_path.basic_velt.fpVx = a_max * cos(alpha) * t_run;
                    nav.auto_path.basic_velt.fpVy = a_max * sin(alpha) * t_run;
                }
                else if (t_run < t1 + t2)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + a_max * cos(alpha) * pow(t1, 2) / 2 + v_max * cos(alpha) * (t_run - t1);
                    nav.auto_path.pos_pid.y.fpDes = StartY + a_max * sin(alpha) * pow(t1, 2) / 2 + v_max * sin(alpha) * (t_run - t1);
                    nav.auto_path.basic_velt.fpVx = v_max * cos(alpha);
                    nav.auto_path.basic_velt.fpVy = v_max * sin(alpha);
                }
                else if (t_run < t1 + t2 + t3)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + delta_x - a_max * cos(alpha) * pow(t1 + t2 + t3 - t_run, 2) / 2.f;
                    nav.auto_path.pos_pid.y.fpDes = StartY + delta_y - a_max * sin(alpha) * pow(t1 + t2 + t3 - t_run, 2) / 2.f;
                    nav.auto_path.basic_velt.fpVx = a_max * cos(alpha) * (t1 + t2 + t3 - t_run);
                    nav.auto_path.basic_velt.fpVy = a_max * sin(alpha) * (t1 + t2 + t3 - t_run);
                }
                else if (t_run < t1 + t2 + t3 + 0.1f)
                {
                    nav.auto_path.pos_pid.x.fpDes = StartX + delta_x;
                    nav.auto_path.pos_pid.y.fpDes = StartY + delta_y;
                    nav.auto_path.basic_velt.fpVx = 0;
                    nav.auto_path.basic_velt.fpVy = 0;
                }
                else
                {
                    nav.auto_path.run_time = 0;
                    flag_record = 1;
                    nav.nav_state = NAV_OFF;
                    //nav.auto_path.number = 5;

                    nav.auto_path.pos_pid.x.fpSumE = 0;
                    nav.auto_path.pos_pid.y.fpSumE = 0;
                    nav.auto_path.pos_pid.w.fpSumE = 0;
                }
                break;

            default:
                break;
        }


        //path_choose(&nav); // 选择哪个打点路径:这里主要使用精细校准模式

        PID_Calc(&nav.auto_path.pos_pid.x, nav.auto_path.pos_pid.x.fpDes, nav.auto_path.pos_pid.x.fpFB);
        PID_Calc(&nav.auto_path.pos_pid.y, nav.auto_path.pos_pid.y.fpDes, nav.auto_path.pos_pid.y.fpFB);
        PID_Calc(&nav.auto_path.pos_pid.w, nav.auto_path.pos_pid.w.fpDes, nav.auto_path.pos_pid.w.fpFB);

        nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.x.fpU + nav.auto_path.basic_velt.fpVx;
        nav.expect_robot_global_velt.fpY = nav.auto_path.pos_pid.y.fpU + nav.auto_path.basic_velt.fpVy;
        nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU + nav.auto_path.basic_velt.fpW;

        SpeedDistribute_Four_OmnidirectionalWheel(&nav);
        break;

    case NAV_PERMUTATION_PATH:
        chassis_run.pid_state = VELT_LOOP;
        chassis_run.feed_forward_state = WITHOUT_FORWARD;

        // if (flag_permutation_path == 1) // 保证只有在一段大路径的每一段小路径开头才进行选择路径
        // {                               // 或者是两段大路径切换时才进行选择，防止多次进行重复打点
        //     path_permutation_choose(&nav);
        //     flag_permutation_path = 0;
        // }
        // NavPosition(&nav, &Path_Permuta);

        // // if (flag_rotation == 1)
        // // {
        // //     NavRotation(&nav, &Path_Permuta);
        // // }

        // CheckPathEnd(&nav, &Path_Permuta);

        PID_Calc(&nav.auto_path.pos_pid.x, nav.auto_path.pos_pid.x.fpDes, nav.auto_path.pos_pid.x.fpFB);
        PID_Calc(&nav.auto_path.pos_pid.y, nav.auto_path.pos_pid.y.fpDes, nav.auto_path.pos_pid.y.fpFB);
        PID_Calc(&nav.auto_path.pos_pid.w, nav.auto_path.pos_pid.w.fpDes, nav.auto_path.pos_pid.w.fpFB);

        nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.x.fpU + nav.auto_path.basic_velt.fpVx;
        nav.expect_robot_global_velt.fpY = nav.auto_path.pos_pid.y.fpU + nav.auto_path.basic_velt.fpVy;
        nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU + nav.auto_path.basic_velt.fpW;

        SpeedDistribute_Four_OmnidirectionalWheel(&nav);
        break;

    default:
        break;
    }
}
