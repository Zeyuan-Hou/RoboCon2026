#include "navigate.h"

void Navigate_Task()
{
    switch (nav.nav_state)
    {
    case CHASSIS_INIT: // 初始化，舵轮转向电机速度环转一圈找零点
        chassis_run.feed_forward_state = WITHOUT_FORWARD;
        chassis_run.rightup.fpDes = 0;
        chassis_run.leftup.fpDes = 0;
        chassis_run.leftdown.fpDes = 0;
        chassis_run.rightdown.fpDes = 0;
        leftup_turn_motor.ControlLoop_State = SPEED_LOOP;
        rightup_turn_motor.ControlLoop_State = SPEED_LOOP;
        leftdown_turn_motor.ControlLoop_State = SPEED_LOOP;
        rightdown_turn_motor.ControlLoop_State = SPEED_LOOP;
        if (!leftup_init_flag)
            leftup_turn_motor.Input_v = 30.0f;
        if (!rightup_init_flag)
            rightup_turn_motor.Input_v = 30.0f;
        if (!leftdown_init_flag)
            leftdown_turn_motor.Input_v = 30.0f;
        if (!rightdown_init_flag)
            rightdown_turn_motor.Input_v = 30.0f;
        if (!travel_switch_mode[0])
        {
            leftup_init_flag = 1;
            leftup_turn_motor.Input_v = 0.0f;
        }
        if (!travel_switch_mode[2])
        {
            rightup_init_flag = 1;
            rightup_turn_motor.Input_v = 0.0f;
        }
        if (travel_switch_mode[3])
        {
            rightdown_init_flag = 1;
            rightdown_turn_motor.Input_v = 0.0f;
        }
        if (travel_switch_mode[4])
        {
            leftdown_init_flag = 1;
            leftdown_turn_motor.Input_v = 0.0f;
        }
        if (leftup_init_flag && rightup_init_flag && leftdown_init_flag && rightdown_init_flag)
        {
            leftup_init_angle = leftup_turn_motor.angle;
            rightup_init_angle = rightup_turn_motor.angle;
            leftdown_init_angle = leftdown_turn_motor.angle;
            rightdown_init_angle = rightdown_turn_motor.angle;
            nav.nav_state = CHASSIS_INIT_DONE;
            leftdown_turn_motor.pid_inner.fpSumE = 0;
            rightup_turn_motor.pid_inner.fpSumE = 0;
            leftup_turn_motor.pid_inner.fpSumE = 0;
            rightdown_turn_motor.pid_inner.fpSumE = 0;
            leftup_init_flag = 0;
            rightup_init_flag = 0;
            leftdown_init_flag = 0;
            rightdown_init_flag = 0;
            nav.expect_robot_global_velt.fpX = 0.f;
            nav.expect_robot_global_velt.fpY = 0.f;
            nav.expect_robot_global_velt.fpW = 0.f;
        }
        CS_Change(NULL);
        
        
        break;

    case CHASSIS_INIT_DONE: // 初始化完成，舵轮转向电机位于零点，转入串级pid控制
        chassis_run.feed_forward_state = WITHOUT_FORWARD;
        leftup_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        rightup_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        leftdown_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        rightdown_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        leftup_turn_motor.Input = leftup_init_angle;
        rightup_turn_motor.Input = rightup_init_angle;
        leftdown_turn_motor.Input = leftdown_init_angle;
        rightdown_turn_motor.Input = rightdown_init_angle;
        chassis_run.rightup.fpDes = 0;
        chassis_run.leftup.fpDes = 0;
        chassis_run.leftdown.fpDes = 0;
        chassis_run.rightdown.fpDes = 0;
        CS_Change(NULL);
        break;

    case CHASSIS_OFF: // 底盘无输出
        chassis_run.feed_forward_state = WITHOUT_FORWARD;
        chassis_run.rightup.fpU = 0;
        chassis_run.leftup.fpU = 0;
        chassis_run.leftdown.fpU = 0;
        chassis_run.rightdown.fpU = 0;
        leftup_turn_motor.motor_current = 0;
        rightup_turn_motor.motor_current = 0;
        leftdown_turn_motor.motor_current = 0;
        rightdown_turn_motor.motor_current = 0;
        chassis_run.leftup.fpSumE = 0.f;
        chassis_run.rightup.fpSumE = 0.f;
        chassis_run.leftdown.fpSumE = 0.f;
        chassis_run.rightdown.fpSumE = 0.f;
        leftup_turn_motor.pid_inner.fpSumE = 0.f;
        leftup_turn_motor.pid_outer.fpSumE = 0.f;
        rightup_turn_motor.pid_inner.fpSumE = 0.f;
        rightup_turn_motor.pid_outer.fpSumE = 0.f;
        leftdown_turn_motor.pid_inner.fpSumE = 0.f;
        leftdown_turn_motor.pid_outer.fpSumE = 0.f;
        rightdown_turn_motor.pid_inner.fpSumE = 0.f;
        rightdown_turn_motor.pid_outer.fpSumE = 0.f;
        CS_Change(NULL);
        break;

    case RC_LOCAL: // 以车体坐标系遥控
        chassis_run.feed_forward_state = WITH_FORWARD;
        leftup_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        rightup_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        leftdown_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        rightdown_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        CalculateVelocities(&Js_Value, &nav, 500, 2500, 500, 2500, 500, 150); // 线速度：mm/s 角速度：度/s
        ramp_signal(&nav.expect_robot_global_velt.fpX, nav.auto_path.basic_velt.fpVx, LineAccelStep);
        ramp_signal(&nav.expect_robot_global_velt.fpY, nav.auto_path.basic_velt.fpVy, LineAccelStep);
        // nav.expect_robot_global_velt.fpX = v_x;
        // nav.expect_robot_global_velt.fpY = v_y;
        // nav.expect_robot_global_velt.fpW = v_w / 180.f * PI;
        nav.expect_robot_global_velt.fpW = nav.auto_path.basic_velt.fpW / 180.f * PI;
        //SpeedDistribute_Four_SteeringWheel(&nav);
        CS_Change(&CS_highspeed);
        break;

    case RC_GLOBAL: // 以全局坐标系遥控，和RC_LOCAL一样处理，在速度分配时根据陀螺仪的yaw角转换速度
        chassis_run.feed_forward_state = WITH_FORWARD;
        leftup_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        rightup_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        leftdown_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        rightdown_turn_motor.ControlLoop_State = MULTIPLE_LOOP;
        CalculateVelocities(&Js_Value, &nav, 500, 3000, 500, 3000, 500, 300); // 线速度：mm/s 角速度：度/s
        ramp_signal(&nav.expect_robot_global_velt.fpX, nav.auto_path.basic_velt.fpVx, LineAccelStep);
        ramp_signal(&nav.expect_robot_global_velt.fpY, nav.auto_path.basic_velt.fpVy, LineAccelStep);
        nav.expect_robot_global_velt.fpW = nav.auto_path.basic_velt.fpW / 180.f * PI;
        //SpeedDistribute_Four_SteeringWheel(&nav);
        CS_Change(&CS_highspeed);
        break;

    case SEMIAUTO_UP_DOWN_STAIRS: // 手操校准车身到台阶的距离以及yaw角后，一键上下台阶的导航部分
        chassis_run.feed_forward_state = WITH_FORWARD;
        UP_DOWN_Velt_Set(&nav);
        //SpeedDistribute_Four_SteeringWheel(&nav);
        CS_Change(&CS_highspeed);
        break;

    case NAV_PERMUTATION_PATH: // 看看是否需要进入初始化函数 8
        chassis_run.feed_forward_state = WITH_FORWARD;

        if (flag_permutation_path == 1) // 保证只有在一段大路径的每一段小路径开头才进行选择路径
        {                               // 或者是两段大路径切换时才进行选择，防止多次进行重复打点
          // SET_NAV_PATH_PERMUTATION();
            path_permutation_choose(&nav); // 选择大段路径
            flag_permutation_path = 0;
						nav.auto_path.pos_pid.w.fpDes=nav.auto_path.pos_pid.w.fpFB;
        }
        NavPosition(&nav, &Path_Permuta);
				
				if(flag_rotation == 1){NavRotation(&nav, &Path_Permuta);} // 看看是否有转向
        
        CheckPathEnd(&nav, &Path_Permuta);

        //PID_Calc_Angle(&nav.auto_path.pos_pid.w);
        if (flag_rotation == 0)
        {
            PID_Calc_Pos(&nav.auto_path.pos_pid.x);
            PID_Calc_Pos(&nav.auto_path.pos_pid.y);
            PID_Calc_Angle(&nav.auto_path.pos_pid.w);
        }

        nav.expect_robot_global_velt.fpX = nav.auto_path.auto_path_vel.fpVx + nav.auto_path.pos_pid.x.fpU;
        nav.expect_robot_global_velt.fpY = nav.auto_path.auto_path_vel.fpVy + nav.auto_path.pos_pid.y.fpU;
        nav.expect_robot_global_velt.fpW = (nav.auto_path.auto_path_vel.fpW + nav.auto_path.pos_pid.w.fpU) * PI / 180.0f; // 没用上fpw

        //SpeedDistribute_Four_SteeringWheel(&nav);
        CS_Change(&CS_lowspeed);
        break;

    case NAV_POINT_TO_POINT:
        chassis_run.feed_forward_state = WITH_FORWARD;
        if (flag_point_to_point)
        {
            path_point_choose(&nav);
            flag_point_to_point = 0;
        }
        if (flag_point_block)
        {
            path_get_block(&nav);
            flag_point_block = 0;
        }			
        Point_to_Point(&Path_Point);
        //SpeedDistribute_Four_SteeringWheel(&nav);
        CS_Change(&CS_lowspeed);
        break;

    case NAV_LOCK:
        chassis_run.feed_forward_state = WITHOUT_FORWARD;

        if (flag_lock == 1) // 记录下刚按下时的坐标，后续fpDes不会再更新
        {
            SET_NAV_PATH_PERMUTATION();

            nav.auto_path.pos_pid.x.fpU = 0;
            nav.auto_path.pos_pid.y.fpU = 0;
            nav.auto_path.pos_pid.w.fpU = 0;

            //							nav.auto_path.pos_pid.x.fpDes =nav.auto_path.pos_pid.x.fpFB;
            //							nav.auto_path.pos_pid.y.fpDes =nav.auto_path.pos_pid.y.fpFB;
            //							nav.auto_path.pos_pid.w.fpDes =nav.auto_path.pos_pid.w.fpFB;

            flag_lock = 0;
        }
        //						PID_Calc_Pos(&nav.auto_path.pos_pid.x);
        //						PID_Calc_Pos(&nav.auto_path.pos_pid.y);
        //						PID_Calc_Pos(&nav.auto_path.pos_pid.w);
        //
        //						nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.x.fpU ;
        //						nav.expect_robot_global_velt.fpY = nav.auto_path.pos_pid.y.fpU ;
        //						nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU*PI/180.0f ;
				  nav.expect_robot_global_velt.fpX=0;
				nav.expect_robot_global_velt.fpY =0;
				nav.expect_robot_global_velt.fpW =0;

        // float d_x = fabs(nav.auto_path.pos_pid.x.fpDes - nav.auto_path.pos_pid.x.fpFB);
        // float d_y = fabs(nav.auto_path.pos_pid.y.fpDes - nav.auto_path.pos_pid.y.fpFB);
        // float d_w = fabs(nav.auto_path.pos_pid.w.fpDes - nav.auto_path.pos_pid.w.fpFB);
        // if ()

        //SpeedDistribute_Four_SteeringWheel(&nav);
				if(flag_path_1 ==3 && part_over_flag==1) CS_Change(&CS_pos);
        else CS_Change(&CS_lowspeed);
        break;

    case TEST_VEL:
        chassis_run.feed_forward_state = WITH_FORWARD;
        nav.expect_robot_global_velt.fpX = v_x;
        nav.expect_robot_global_velt.fpY = v_y;
        nav.expect_robot_global_velt.fpW = v_w / 180.f * PI;
        nav.expect_robot_global_velt.fpW = nav.auto_path.basic_velt.fpW / 180.f * PI;
        //SpeedDistribute_Four_SteeringWheel(&nav);
        CS_Change(&CS_lowspeed);
        break;

    case TEST_POS:
        chassis_run.feed_forward_state = WITH_FORWARD;
        nav.auto_path.pos_pid.x.fpDes = pos_x;
        nav.auto_path.pos_pid.y.fpDes = pos_y;
        nav.auto_path.pos_pid.w.fpDes = yaw;
        // if (fabs(nav.auto_path.pos_pid.w.fpFB - nav.auto_path.pos_pid.w.fpDes) > 180.f)
        // {
        //     if (nav.auto_path.pos_pid.w.fpFB > nav.auto_path.pos_pid.w.fpDes)
        //     {
        //         nav.auto_path.pos_pid.w.fpDes += 360.f;
        //     }
        //     else
        //     {
        //         nav.auto_path.pos_pid.w.fpDes -= 360.f;
        //     }
        // }
        PID_Calc(&nav.auto_path.pos_pid.x, nav.auto_path.pos_pid.x.fpDes, nav.auto_path.pos_pid.x.fpFB);
        PID_Calc(&nav.auto_path.pos_pid.y, nav.auto_path.pos_pid.y.fpDes, nav.auto_path.pos_pid.y.fpFB);
        PID_Calc_Angle(&nav.auto_path.pos_pid.w);
        nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.x.fpU;
        nav.expect_robot_global_velt.fpY = nav.auto_path.pos_pid.y.fpU;
        nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU / 180.f * PI;
        //SpeedDistribute_Four_SteeringWheel(&nav);
        CS_Change(&CS_highspeed);
        break;

    default:
        break;
    }
}
