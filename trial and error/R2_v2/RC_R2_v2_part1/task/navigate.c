#include "navigate.h"

void Navigate_Task()
{
    switch (nav.nav_state)
    {
    case CHASSIS_INIT: // 初始化，舵轮转向电机速度环转一圈找零点
        chassis_run2.rightup.fpDes = 0;
        chassis_run2.leftup.fpDes = 0;
        chassis_run2.down.fpDes = 0;
        leftup_turn_motor.ControlLoop_State = SPEED_LOOP;
        rightup_turn_motor.ControlLoop_State = SPEED_LOOP;
        down_turn_motor.ControlLoop_State = SPEED_LOOP;
        if (!leftup_init_flag)
            leftup_turn_motor.Input_v = 35.0f;
        if (!rightup_init_flag)
            rightup_turn_motor.Input_v = 28.0f;
        if (!down_init_flag)
            down_turn_motor.Input_v = 35.0f;
        if (travel_switch_mode[0])
        {
            leftup_init_flag = 1;
            leftup_turn_motor.Input_v = 0.0f;
        }
        if (!travel_switch_mode[1])
        {
            rightup_init_flag = 1;
            rightup_turn_motor.Input_v = 0.0f;
        }
        if (travel_switch_mode[2])
        {
            down_init_flag = 1;
            down_turn_motor.Input_v = 0.0f;
        }
        if (leftup_init_flag && rightup_init_flag && down_init_flag)
        {
            leftup_init_angle = leftup_turn_motor.angle;
            rightup_init_angle = rightup_turn_motor.angle;
            down_init_angle = down_turn_motor.angle;
            nav.nav_state = CHASSIS_INIT_DONE;
            rightup_turn_motor.pid_inner.fpSumE = 0;
            leftup_turn_motor.pid_inner.fpSumE = 0;
            down_turn_motor.pid_inner.fpSumE = 0;
            leftup_init_flag = 0;
            rightup_init_flag = 0;
            down_init_flag = 0;
        }
        break;

    case CHASSIS_INIT_DONE: // 初始化完成，舵轮转向电机位于零点，转入串级pid控制
        leftup_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        rightup_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        down_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        leftup_turn_motor.Input = leftup_init_angle;
        rightup_turn_motor.Input = rightup_init_angle;
        down_turn_motor.Input = down_init_angle;
        chassis_run2.rightup.fpDes = 0;
        chassis_run2.leftup.fpDes = 0;
        chassis_run2.down.fpDes = 0;
        break;

    case CHASSIS_OFF: // 底盘无输出
        chassis_run2.rightup.fpU = 0;
        chassis_run2.leftup.fpU = 0;
        chassis_run2.down.fpU = 0;
        leftup_turn_motor.motor_current = 0;
        rightup_turn_motor.motor_current = 0;
        down_turn_motor.motor_current = 0;
        chassis_run2.leftup.fpUKi = 0.f;
        chassis_run2.rightup.fpUKi = 0.f;
        chassis_run2.down.fpUKi = 0.f;
        leftup_turn_motor.pid_inner.fpSumE = 0.f;
        leftup_turn_motor.pid_outer.fpSumE = 0.f;
        rightup_turn_motor.pid_inner.fpSumE = 0.f;
        rightup_turn_motor.pid_outer.fpSumE = 0.f;
        down_turn_motor.pid_inner.fpSumE = 0.f;
        down_turn_motor.pid_outer.fpSumE = 0.f;
        break;

    case RC_LOCAL: // 以车体坐标系遥控
        leftup_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        rightup_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        down_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        CalculateVelocities(&Js_Value, &nav, 200, 4000, 200, 4000, 200, 270); // 线速度：mm/s 角速度：度/s
        ramp_signal(&nav.expect_robot_global_velt.fpX, nav.auto_path.basic_velt.fpVx, LineAccelStep);
        ramp_signal(&nav.expect_robot_global_velt.fpY, nav.auto_path.basic_velt.fpVy, LineAccelStep);
        nav.expect_robot_global_velt.fpW = nav.auto_path.basic_velt.fpW / 180.f * PI;
        SpeedDistribute_Four_SteeringWheel(&nav);
        break;

    case RC_GLOBAL: // 以全局坐标系遥控，和RC_LOCAL一样处理，在速度分配时根据陀螺仪的yaw角转换速度
        leftup_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        rightup_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        down_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        CalculateVelocities(&Js_Value, &nav, 200, 2500, 200, 2500, 200, 150); // 线速度：mm/s 角速度：度/s
        ramp_signal(&nav.expect_robot_global_velt.fpX, nav.auto_path.basic_velt.fpVx, LineAccelStep);
        ramp_signal(&nav.expect_robot_global_velt.fpY, nav.auto_path.basic_velt.fpVy, LineAccelStep);
        nav.expect_robot_global_velt.fpW = nav.auto_path.basic_velt.fpW / 180.f * PI;
        SpeedDistribute_Four_SteeringWheel(&nav);
        break;

    case SEMIAUTO_UP_DOWN_STAIRS: // 手操校准车身到台阶的距离以及yaw角后，一键上下台阶的导航部分
        UP_DOWN_Velt_Set(&nav);
        SpeedDistribute_Four_SteeringWheel(&nav);
        break;

    // case NAV_PERMUTATION_PATH: // 看看是否需要进入初始化函数 8
    //     chassis_run.feed_forward_state = WITH_FORWARD;

    //     if (flag_permutation_path == 1)    // 保证只有在一段大路径的每一段小路径开头才进行选择路径
    //     {                                  // 或者是两段大路径切换时才进行选择，防止多次进行重复打点
    //                                        // SET_NAV_PATH_PERMUTATION();
    //         path_permutation_choose(&nav); // 选择大段路径
    //         flag_permutation_path = 0;
    //     }
    //     NavPosition(&nav, &Path_Permuta);
    //     NavRotation(&nav, &Path_Permuta); // 看看是否有转向
    //     CheckPathEnd(&nav, &Path_Permuta);

    //     PID_Calc_Angle(&nav.auto_path.pos_pid.w);
    //     if (flag_rotation == 0)
    //     {
    //         PID_Calc_Pos(&nav.auto_path.pos_pid.x);
    //         PID_Calc_Pos(&nav.auto_path.pos_pid.y);
    //         nav.auto_path.pos_pid.w.fpU = 0;
    //     }

    //     nav.expect_robot_global_velt.fpX = nav.auto_path.auto_path_vel.fpVx + nav.auto_path.pos_pid.x.fpU;
    //     nav.expect_robot_global_velt.fpY = nav.auto_path.auto_path_vel.fpVy + nav.auto_path.pos_pid.y.fpU;
    //     nav.expect_robot_global_velt.fpW = (nav.auto_path.auto_path_vel.fpW + nav.auto_path.pos_pid.w.fpU) * PI / 180.0f; // 没用上fpw

    //     SpeedDistribute_Four_SteeringWheel(&nav);
    //     break;

    case NAV_POINT_TO_POINT:
        leftup_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        rightup_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        down_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        if (Path_Point.flag_point_to_point)
        {
            path_point_choose(&nav);
            Path_Point.flag_point_to_point = 0;
        }
        
        Point_to_Point(&Path_Point);
        SpeedDistribute_Four_SteeringWheel(&nav);
        break;

    case NAV_LOCK:
        if (flag_lock == 1) // 记录下刚按下时的坐标，后续fpDes不会再更新
        {
            SET_NAV_PATH_PERMUTATION();
            nav.auto_path.pos_pid.x.fpU = 0;
            nav.auto_path.pos_pid.y.fpU = 0;
            nav.auto_path.pos_pid.w.fpU = 0;
            flag_lock = 0;
        }
        nav.expect_robot_global_velt.fpX = 0;
        nav.expect_robot_global_velt.fpY = 0;
        nav.expect_robot_global_velt.fpW = 0;
        SpeedDistribute_Four_SteeringWheel(&nav);
        break;

    default:
        break;
    }
}
