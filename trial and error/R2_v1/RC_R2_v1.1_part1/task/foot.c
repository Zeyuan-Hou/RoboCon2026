#include "foot.h"

void Foot_WorkLoop()
{
    switch (foot.foot_state)
    {
    case FOOT_INIT: // 小脚当前状态--->小脚不使用状态
        // 记录每个状态的运行时间，预留为其它用途
        if (foot_motor_runtime_flag == 0)
        {
            foot_motor_runtime = 0;
            foot_motor_runtime_flag = 1;
        }
        // 导航状态复位
        nav.auto_path.run_time_flag = 0;
        nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
        // nav.nav_state = RC_LOCAL;
        //  是否开启重力前馈
        foot.up_tor_feedforward_flag = 0;
        foot.down_tor_feedforward_flag = 0;
        // 设置pid及td参数
        j60_motor_cmd_down.kp_ = 24.f, j60_motor_cmd_down.kd_ = 4.f;
        j60_motor_cmd_down.j60_td.r = 50000.f, j60_motor_cmd_down.j60_td.h = 0.001f, j60_motor_cmd_down.j60_td.T = 0.001f;
        go1_send_left.K_P = 0.9f, go1_send_left.K_W = 0.07f;
        go1_send_left.td.r = 3000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
        go1_send_right.K_P = 0.9f, go1_send_right.K_W = 0.07f;
        go1_send_right.td.r = 3000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
        // 设置目标角度
        foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f - 90.f) / 9.0515f + 21.33f + go1_left_0;
        foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f - 90.f) / -9.0515f - 21.33f + go1_right_0;
        foot.down_aim_angle = -75.f / 45.f + 2.f;
        // 由位置反馈判断是否状态递进
        if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 0.633f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 0.633f && fabs(j60_motor_data_down.position_ - foot.down_aim_angle) < 0.1f)
        {
            foot.foot_state = FOOT_CLEAR;
            foot_motor_runtime_flag = 0;
        }
        break;

    case FOOT_CLEAR: // 临时状态，取消重力前馈，设置保守的pid及td参数，用于纯手操时一个状态结束后的缓冲
        nav.auto_path.run_time_flag = 0;
        nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
        // nav.nav_state = RC_LOCAL;
        foot_motor_runtime_flag = 0;
        foot.up_tor_feedforward_flag = 0;
        foot.down_tor_feedforward_flag = 0;
        j60_motor_cmd_down.kp_ = 30.f, j60_motor_cmd_down.kd_ = 4.f;
        j60_motor_cmd_down.j60_td.r = 50000.f, j60_motor_cmd_down.j60_td.h = 0.001f, j60_motor_cmd_down.j60_td.T = 0.001f;
        go1_send_left.K_P = 0.95f, go1_send_left.K_W = 0.06f;
        go1_send_left.td.r = 3000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
        go1_send_right.K_P = 0.95f, go1_send_right.K_W = 0.06f;
        go1_send_right.td.r = 3000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
        break;

    case FOOT_UP_PREPARE: // 小脚当前状态--->准备上台阶
        if (foot_motor_runtime_flag == 0)
        {
            foot_motor_runtime = 0;
            foot_motor_runtime_flag = 1;
        }
        nav.auto_path.run_time_flag = 0;
        nav.nav_state = SEMIAUTO_UP_DOWN_STAIRS;
        nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
        foot.up_tor_feedforward_flag = 0;
        foot.down_tor_feedforward_flag = 0;
        j60_motor_cmd_down.kp_ = 100.f, j60_motor_cmd_down.kd_ = 5.f;
        j60_motor_cmd_down.j60_td.r = 5000000.f, j60_motor_cmd_down.j60_td.h = 0.001f, j60_motor_cmd_down.j60_td.T = 0.001f;
        go1_send_left.K_P = 1.f, go1_send_left.K_W = 0.08f;
        go1_send_left.td.r = 10000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
        go1_send_right.K_P = 1.f, go1_send_right.K_W = 0.08f;
        go1_send_right.td.r = 10000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
        foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f + 8.f) / 9.0515f + 21.33f + go1_left_0;
        foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f + 8.f) / -9.0515f - 21.33f + go1_right_0;
        foot.down_aim_angle = -60.f / 45.f + 2.f; // 修改后脚加速过程前的角度，节省空间
        if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 0.633f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 0.633f && fabs(j60_motor_data_down.position_ - foot.down_aim_angle) < 0.1f)
        {
            foot.foot_state = FOOT_UPSTAIRS;
            foot_motor_runtime_flag = 0;
        }
        break;

    case FOOT_UPSTAIRS: // 小脚当前状态--->上台阶中
        switch (foot.foot_up)
        {
        case UP1: // 小脚当前状态--->迅速站立
            if (foot_motor_runtime_flag == 0)
            {
                foot_motor_runtime = 0;
                foot_motor_runtime_flag = 1;
                foot.up_tor_feedforward_flag = 1;
                nav.auto_path.run_time_flag = 0;
                nav.nav_state = SEMIAUTO_UP_DOWN_STAIRS;
                nav.auto_path.up_down_state = UP_APPROACH_1;
                // 降低加速过程的目标速度，防止前脚撞台阶
            }
						
						if(fabs(j60_motor_data_down.position_ - foot.down_aim_angle) < 0.2f){
							backfoot_vel=200;
							K_backfoot_vel=10;
						}
						
            if (Foot_DT35_distance_check(1))
            {
                foot.down_tor_feedforward_flag = 1;
                j60_motor_cmd_down.kp_ = 600.f, j60_motor_cmd_down.kd_ = 5.f;//5.f
                j60_motor_cmd_down.j60_td.r = 10000000000.f, j60_motor_cmd_down.j60_td.h = 0.001f, j60_motor_cmd_down.j60_td.T = 0.001f;
                foot.down_aim_angle = 90.f / 45.f + 2.f;
            }
            if (Foot_DT35_distance_check(0))
            {
								if(cyt_flag==0){
									cyt_flag=1;
									cyt_cnt=0;
								}
                go1_send_left.K_P = 2.4f, go1_send_left.K_W = 0.1f;
                go1_send_left.td.r = 200000000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
                go1_send_right.K_P = 2.4f, go1_send_right.K_W = 0.1f;
                go1_send_right.td.r = 200000000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
								foot.leftup_aim_angle =(foot.up_foot_circle_count * 360.f + 89.f) / 9.0515f + 21.33f + go1_left_0;
								foot.rightup_aim_angle =(foot.up_foot_circle_count * 360.f + 89.f) / -9.0515f - 21.33f + go1_right_0;
								j60_motor_cmd_down.kp_ = 800.f, j60_motor_cmd_down.kd_ = 2.f;
								foot.down_aim_angle =85.f / 45.f + 2.f;
								cyt_vel=K_Vel*(0.8+0.2*(500-cyt_cnt)/500);
//                foot.leftup_aim_angle = RampSignal((foot.up_foot_circle_count * 360.f + 8.f) / 9.0515f + 21.33f + go1_left_0,(foot.up_foot_circle_count * 360.f + 89.f) / 9.0515f + 21.33f + go1_left_0,cyt_cnt,450);
//                foot.rightup_aim_angle =RampSignal((foot.up_foot_circle_count * 360.f + 8.f) / -9.0515f - 21.33f + go1_right_0,(foot.up_foot_circle_count * 360.f + 89.f) / -9.0515f - 21.33f + go1_right_0,cyt_cnt,450);
//                foot.down_aim_angle =RampSignal(20.f / 45.f + 2.f, 90.f / 45.f + 2.f,cyt_cnt,350);
            }
            if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 1.266f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 1.266f && fabs(j60_motor_data_down.position_ - foot.down_aim_angle) < 0.1f && go1_send_left.K_P == 2.4f && j60_motor_cmd_down.kp_ == 800.f)
            { // 到达目标角度 && 通过dt35的检测
                foot.foot_up = UP2;
								cyt_flag=0;
								cyt_vel=0;
                foot_motor_runtime_flag = 0;
            }
            break;

        case UP2: // 小脚当前状态--->前脚收起
//            if (foot_motor_runtime_flag == 0 && travel_switch_mode[6])
//            { // 向下的红外行程开关检测到车身由空中下降到台阶上，开始计时
//                foot_motor_runtime = 0;
//                foot_motor_runtime_flag = 1;
//            }
            // 底盘瞬间提供较大速度，保证底盘前轮能上台阶
            nav.auto_path.run_time_flag = 0;
            nav.nav_state = SEMIAUTO_UP_DOWN_STAIRS;
            nav.auto_path.up_down_state = UP_APPROACH_2;
						//后脚主动轮提供动力
						backfoot_vel=250;
						K_backfoot_vel=10;
							// 无论是否成功上台阶，及时调整温和的pid参数，防止j60过热使能
            j60_motor_cmd_down.kp_ = 600.f, j60_motor_cmd_down.kd_ = 5.f;
            j60_motor_cmd_down.j60_td.r = 50000.f, j60_motor_cmd_down.j60_td.h = 0.001f, j60_motor_cmd_down.j60_td.T = 0.001f;
            go1_send_left.K_P = 1.5f, go1_send_left.K_W = 0.08f;
            go1_send_left.td.r = 3000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
            go1_send_right.K_P = 1.5f, go1_send_right.K_W = 0.08f;
            go1_send_right.td.r = 3000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
            if (foot_motor_runtime_flag == 1 && foot_motor_runtime > 130)
            { // 红外行程开关检测到前轮台130ms后
							
							foot.up_tor_feedforward_flag = 0;
              foot.down_tor_feedforward_flag = 1;
              foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f + 180.f) / 9.0515f + 21.33f + go1_left_0;
              foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f + 180.f) / -9.0515f - 21.33f + go1_right_0;
							if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 0.633f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 0.633f ){
								foot.foot_up = UP3;
                foot_motor_runtime_flag = 0;
								foot.down_aim_angle =90.f / 45.f + 2.f;
							}
            }
            if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 0.633f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 0.633f )
            { // 到达目标角度
								if (foot_motor_runtime_flag == 0 && travel_switch_mode[6])
								{ // 向下的红外行程开关检测到车身由空中下降到台阶上，开始计时
										foot_motor_runtime = 0;
										foot_motor_runtime_flag = 1;
								}
                
            }
            break;

        case UP3: // 小脚当前状态--->后脚收起
						
            if (foot_motor_runtime_flag == 0)
            {
                foot_motor_runtime = 0;
                foot_motor_runtime_flag = 1;
                nav.auto_path.run_time_flag = 0;
                nav.nav_state = SEMIAUTO_UP_DOWN_STAIRS;
                nav.auto_path.up_down_state = UP_UPSTAIRS;
            }
						if(travel_switch_mode[7]==1){
							foot.down_aim_angle = 0.f / 45.f + 2.f;
						}
            if (foot_motor_runtime > 180)
            {
								backfoot_vel=0;
								K_backfoot_vel=0;
                foot.up_tor_feedforward_flag = 0;
                foot.down_tor_feedforward_flag = 0;
                j60_motor_cmd_down.kp_ = 100.f, j60_motor_cmd_down.kd_ = 4.f;
                j60_motor_cmd_down.j60_td.r = 500000000.f, j60_motor_cmd_down.j60_td.h = 0.001f, j60_motor_cmd_down.j60_td.T = 0.001f;
                go1_send_left.K_P = 0.9f, go1_send_left.K_W = 0.07f;
                go1_send_left.td.r = 10000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
                go1_send_right.K_P = 0.9f, go1_send_right.K_W = 0.07f;
                go1_send_right.td.r = 10000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
                foot.down_aim_angle = 0.f / 45.f + 2.f;
                nav.auto_path.run_time_flag = 0;
                nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
                // nav.nav_state = RC_LOCAL;
            }
            if (fabs(j60_motor_data_down.position_ - foot.down_aim_angle) < 0.3f && foot.down_tor_feedforward_flag == 0)
            {
								backfoot_reset=0;
                foot.foot_state = FOOT_INIT;
                foot.foot_up = UP1;
                foot_motor_runtime_flag = 0;
                foot.up_foot_circle_count++;
                // 上台阶完成，go1电机圈数加一
            }
            break;

        case UP4: // 临时状态，保留上一个状态重力前馈状态，设置保守的pid及td参数，用于纯手操时一个状态结束后的缓冲
            foot_motor_runtime_flag = 0;
            j60_motor_cmd_down.kp_ = 24.f, j60_motor_cmd_down.kd_ = 4.f;
            j60_motor_cmd_down.j60_td.r = 50000.f, j60_motor_cmd_down.j60_td.h = 0.001f, j60_motor_cmd_down.j60_td.T = 0.001f;
            go1_send_left.K_P = 0.9f, go1_send_left.K_W = 0.07f;
            go1_send_left.td.r = 3000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
            go1_send_right.K_P = 0.9f, go1_send_right.K_W = 0.07f;
            go1_send_right.td.r = 3000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
            break;

        default:
            break;
        }
        break;

    case FOOT_DOWN_PREPARE: // 小脚当前状态--->准备下台阶
        if (foot_motor_runtime_flag == 0)
        {
            foot_motor_runtime = 0;
            foot_motor_runtime_flag = 1;
        }
        nav.auto_path.run_time_flag = 0;
        nav.nav_state = SEMIAUTO_UP_DOWN_STAIRS;
        nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
        foot.up_tor_feedforward_flag = 0;
        foot.down_tor_feedforward_flag = 0;
        j60_motor_cmd_down.kp_ = 24.f, j60_motor_cmd_down.kd_ = 4.f;
        j60_motor_cmd_down.j60_td.r = 50000.f, j60_motor_cmd_down.j60_td.h = 0.001f, j60_motor_cmd_down.j60_td.T = 0.001f;
        go1_send_left.K_P = 0.9f, go1_send_left.K_W = 0.07f;
        go1_send_left.td.r = 3000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
        go1_send_right.K_P = 0.9f, go1_send_right.K_W = 0.07f;
        go1_send_right.td.r = 3000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
        foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f - 190.f) / 9.0515f + 21.33f + go1_left_0;
        foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f - 190.f) / -9.0515f - 21.33f + go1_right_0;
        foot.down_aim_angle = 18.f / 45.f + 2.f;
        if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 0.633f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 0.633f && fabs(j60_motor_data_down.position_ - foot.down_aim_angle) < 0.1f)
        {
            foot.foot_state = FOOT_DOWNSTAIRS;
            foot_motor_runtime_flag = 0;
        }
        break;

    case FOOT_DOWNSTAIRS: // 小脚当前状态--->下台阶中
        switch (foot.foot_down)
        {
        case DOWN1: // 小脚当前状态--->后脚放下
						backfoot_vel=-150;
						K_backfoot_vel=10;
            if (foot_motor_runtime_flag == 0)
            {
                foot_motor_runtime = 0;
                foot_motor_runtime_flag = 1;
                nav.auto_path.run_time_flag = 0;
                nav.nav_state = SEMIAUTO_UP_DOWN_STAIRS;
                nav.auto_path.up_down_state = DOWN_APPROACH_AND_DOWNSTAIRS;
            }
            foot.up_tor_feedforward_flag = 0;
            foot.down_tor_feedforward_flag = 1;
            j60_motor_cmd_down.kp_ = 30.f, j60_motor_cmd_down.kd_ = 4.f;
            j60_motor_cmd_down.j60_td.r = 50000.f, j60_motor_cmd_down.j60_td.h = 0.001f, j60_motor_cmd_down.j60_td.T = 0.001f;
            go1_send_left.K_P = 1.f, go1_send_left.K_W = 0.08f;
            go1_send_left.td.r = 3000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
            go1_send_right.K_P = 1.f, go1_send_right.K_W = 0.08f;
            go1_send_right.td.r = 3000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
            foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f - 190.f) / 9.0515f + 21.33f + go1_left_0;
            foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f - 190.f) / -9.0515f - 21.33f + go1_right_0;
            foot.down_aim_angle = 90.f / 45.f + 2.f;
            if (fabs(j60_motor_data_down.position_ - foot.down_aim_angle) < 0.4f)
            {
                foot.foot_down = DOWN2;
                foot_motor_runtime_flag = 0;
            }
            break;

        case DOWN2: // 小脚当前状态--->前脚放下
            if (foot_motor_runtime_flag == 0)
            {
                foot_motor_runtime = 0;
                foot_motor_runtime_flag = 1;
            }
            if (foot_motor_runtime > 500)
            {
                foot.up_tor_feedforward_flag = 1;
                foot.down_tor_feedforward_flag = 1;
                j60_motor_cmd_down.kp_ = 30.f, j60_motor_cmd_down.kd_ = 4.f;
                j60_motor_cmd_down.j60_td.r = 50000.f, j60_motor_cmd_down.j60_td.h = 0.001f, j60_motor_cmd_down.j60_td.T = 0.001f;
                go1_send_left.K_P = 1.f, go1_send_left.K_W = 0.08f;
                go1_send_left.td.r = 3000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
                go1_send_right.K_P = 1.f, go1_send_right.K_W = 0.08f;
                go1_send_right.td.r = 3000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
                foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f - 270.f) / 9.0515f + 21.33f + go1_left_0;
                foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f - 270.f) / -9.0515f - 21.33f + go1_right_0;
                foot.down_aim_angle = 90.f / 45.f + 2.f;
            }
            if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 1.266f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 1.266f && foot.up_tor_feedforward_flag == 1)
            {
                foot.foot_down = DOWN3;
                foot_motor_runtime_flag = 0;
                nav.auto_path.run_time_flag = 0;
                nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
                // nav.nav_state = RC_LOCAL;
            }
            break;

        case DOWN3: // 小脚当前状态--->慢慢坐下
            if (foot_motor_runtime_flag == 0)
            {
                foot_motor_runtime = 0;
                foot_motor_runtime_flag = 1;
            }
            foot.up_tor_feedforward_flag = 1;
            foot.down_tor_feedforward_flag = 1;
            j60_motor_cmd_down.kp_ = 30.f, j60_motor_cmd_down.kd_ = 4.f;
            j60_motor_cmd_down.j60_td.r = 8.f, j60_motor_cmd_down.j60_td.h = 0.001f, j60_motor_cmd_down.j60_td.T = 0.001f;
            go1_send_left.K_P = 1.f, go1_send_left.K_W = 0.08f;
            go1_send_left.td.r = 35.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
            go1_send_right.K_P = 1.f, go1_send_right.K_W = 0.08f;
            go1_send_right.td.r = 35.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
            foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f - 350.f) / 9.0515f + 21.33f + go1_left_0;
            foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f - 350.f) / -9.0515f - 21.33f + go1_right_0;
            foot.down_aim_angle = 20.f / 45.f + 2.f;
            if (foot_motor_runtime > 1000)
            {
                // 定时关闭重力前馈，防止小脚因重力前馈阻碍到达目标位置
                foot.up_tor_feedforward_flag = 0;
                foot.down_tor_feedforward_flag = 0;
            }
            if ((fabs(j60_motor_data_down.position_ - foot.down_aim_angle) < 0.5f && fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 2.f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 2.f) || foot_motor_runtime > 1200)
            { // 到达目标位置 || 超时摔下台阶  或逻辑用以保证执行go1电机转圈计数减一
                foot.foot_state = FOOT_INIT;
                foot.foot_down = DOWN1;
                foot_motor_runtime_flag = 0;
								backfoot_vel=0;
								K_backfoot_vel=0;
                foot.up_foot_circle_count--;
                // 下台阶完成，go1电机圈数减一
            }
            break;

        case DOWN4: // 临时状态，保留上一个状态重力前馈状态，保留上一个状态的pid及td参数，用于纯手操时一个状态结束后的缓冲
            break;

        default:
            break;
        }
        break;

    // 以下为测试状态，准备上台阶--->站立--->坐下--->准备上台阶循环
    case FOOT_TEST_SIT:
        if (foot_motor_runtime_flag == 0)
        {
            foot_motor_runtime = 0;
            foot_motor_runtime_flag = 1;
        }
        foot.up_tor_feedforward_flag = 1;
        foot.down_tor_feedforward_flag = 1;
        j60_motor_cmd_down.kp_ = 24.f, j60_motor_cmd_down.kd_ = 4.f;
        j60_motor_cmd_down.j60_td.r = 8.f, j60_motor_cmd_down.j60_td.h = 0.001f, j60_motor_cmd_down.j60_td.T = 0.001f;
        go1_send_left.K_P = 0.9f, go1_send_left.K_W = 0.07f;
        go1_send_left.td.r = 30.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
        go1_send_right.K_P = 0.9f, go1_send_right.K_W = 0.07f;
        go1_send_right.td.r = 30.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
        foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f + 10.f) / 9.0515f + 21.33f + go1_left_0;
        foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f + 10.f) / -9.0515f - 21.33f + go1_right_0;
        foot.down_aim_angle = 20.f / 45.f + 2.f;
        if (foot_motor_runtime >= 900)
        {
            foot.up_tor_feedforward_flag = 0;
            foot.down_tor_feedforward_flag = 0;
        }
        if ((fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 2.f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 2.f && fabs(j60_motor_data_down.position_ - foot.down_aim_angle) < 0.5f) || foot_motor_runtime > 1200)
        {
            foot.foot_state = FOOT_INIT;
            foot.foot_down = DOWN1;
            foot.foot_up = UP1;
            foot_motor_runtime_flag = 0;
        }
        break;

    case FOOT_TEST_STAND:
        if (foot_motor_runtime_flag == 0)
        {
            foot_motor_runtime = 0;
            foot_motor_runtime_flag = 1;
            nav.auto_path.run_time_flag = 0;
            nav.nav_state = SEMIAUTO_UP_DOWN_STAIRS;
            nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
        }
        foot.up_tor_feedforward_flag = 1;
        foot.down_tor_feedforward_flag = 1;
        j60_motor_cmd_down.kp_ = 200.f, j60_motor_cmd_down.kd_ = 5.f;
        j60_motor_cmd_down.j60_td.r = 100000000.f, j60_motor_cmd_down.j60_td.h = 0.001f, j60_motor_cmd_down.j60_td.T = 0.001f;
        go1_send_left.K_P = 2.f, go1_send_left.K_W = 0.1f;
        go1_send_left.td.r = 2000000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
        go1_send_right.K_P = 2.f, go1_send_right.K_W = 0.1f;
        go1_send_right.td.r = 2000000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
        foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f + 85.f) / 9.0515f + 21.33f + go1_left_0;
        foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f + 85.f) / -9.0515f - 21.33f + go1_right_0;
        foot.down_aim_angle = 85.f / 45.f + 2.f;
        if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 1.266f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 1.266f && fabs(j60_motor_data_down.position_ - foot.down_aim_angle) < 0.3f)
        {
            foot.foot_state = FOOT_CLEAR;
            foot.foot_up = UP1;
            foot.foot_down = DOWN1;
            foot_motor_runtime_flag = 0;
        }
        break;

    case FOOT_TEST_STAND_PREPARE: // 小脚当前状态--->准备上台阶
        if (foot_motor_runtime_flag == 0)
        {
            foot_motor_runtime = 0;
            foot_motor_runtime_flag = 1;
        }
        nav.auto_path.run_time_flag = 0;
        nav.nav_state = SEMIAUTO_UP_DOWN_STAIRS;
        nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
        foot.up_tor_feedforward_flag = 0;
        foot.down_tor_feedforward_flag = 0;
        j60_motor_cmd_down.kp_ = 100.f, j60_motor_cmd_down.kd_ = 5.f;
        j60_motor_cmd_down.j60_td.r = 5000000.f, j60_motor_cmd_down.j60_td.h = 0.001f, j60_motor_cmd_down.j60_td.T = 0.001f;
        go1_send_left.K_P = 1.f, go1_send_left.K_W = 0.08f;
        go1_send_left.td.r = 10000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
        go1_send_right.K_P = 1.f, go1_send_right.K_W = 0.08f;
        go1_send_right.td.r = 10000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
        foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f + 8.f) / 9.0515f + 21.33f + go1_left_0;
        foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f + 8.f) / -9.0515f - 21.33f + go1_right_0;
        foot.down_aim_angle = -60.f / 45.f + 2.f; // 修改后脚加速过程前的角度，节省空间
        if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 0.633f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 0.633f && fabs(j60_motor_data_down.position_ - foot.down_aim_angle) < 0.1f)
        {
            foot.foot_state = FOOT_TEST_STAND;
            foot_motor_runtime_flag = 0;
        }
        break;

    default:
        break;
    }
}

bool Foot_DT35_distance_check(uint8_t mode)
{
    if (mode == 0)
    {
        if (dt35_now.dt35_voltage_3 < 3300.f)
            return 1;
        else
            return 0;
    }

    if (mode == 1)
    {
        if (dt35_now.dt35_voltage_3 < 3400.f&&dt35_now.dt35_voltage_3 > 3300.f)
            return 1;
        else
            return 0;
    }
}
