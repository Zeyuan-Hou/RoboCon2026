#include "foot.h"

// FOOT_INIT ---> FOOT_CLEAR
// FOOT_UPSTAIRS_LV1 {UP11 ---> UP12 ---> UP13 ---> UP14} ---> FOOT_INIT ---> FOOT_CLEAR
// FOOT_UPSTAIRS_LV2 {UP21 ---> UP22 ---> UP23 ---> UP24} ---> FOOT_INIT ---> FOOT_CLEAR
// FOOT_DOWNSTAIRS_LV1 {DOWN11 ---> DOWN12 ---> DOWN13 ---> DOWN14} ---> FOOT_INIT ---> FOOT_CLEAR
// FOOT_DOWNSTAIRS_LV2 {DOWN21 ---> DOWN22 ---> DOWN23 ---> DOWN24} ---> FOOT_INIT ---> FOOT_CLEAR

// 未调试状态：
// FOOT_TEST_STAND_PREPARE ---> FOOT_CLEAR    FOOT_TEST_STAND ---> FOOT_CLEAR    FOOT_TEST_SIT ---> FOOT_INIT ---> FOOT_CLEAR

void Foot_WorkLoop()
{
    switch (foot.foot_state)
    {
    case FOOT_INIT: // 小脚当前状态--->小脚不使用状态
        // 导航状态复位
        nav.auto_path.run_time_flag = 0;
        nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
        // 小脚保持收缩状态
        Foot_Shorten();
        //  是否开启重力前馈
        foot.up_tor_feedforward_flag = 0;
        foot.down_tor_feedforward_flag = 0;
        // 设置pid及td参数
        j60_motor_cmd_down2.kp_ = 25.f, j60_motor_cmd_down2.kd_ = 4.f;
        j60_motor_cmd_down2.j60_td.r = 100000.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
        go1_send_left.K_P = 1.f, go1_send_left.K_W = 0.06f;
        go1_send_left.td.r = 1000000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
        go1_send_right.K_P = 1.f, go1_send_right.K_W = 0.06f;
        go1_send_right.td.r = 1000000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
        // 设置目标角度
        foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f - 90.f) / -9.0515f - 21.56f + go1_left_0;
        foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f - 90.f) / 9.0515f + 21.38f + go1_right_0;
        foot.down_aim_angle = -50.f / -57.f - 1.7f;
        // 由位置反馈判断是否状态递进
        if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 0.633f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 0.633f && fabs(j60_motor_data_down2.position_ - foot.down_aim_angle) < 0.1f)
        {
            foot.foot_state = FOOT_CLEAR;
        }
        break;

    case FOOT_CLEAR: // 临时状态，取消重力前馈，设置保守的pid及td参数，用于纯手操时一个状态结束后的缓冲，等待下一次遥控器或状态机的指令输入
        foot.runtime_flag = 0;
        foot.runtime = 0;
        nav.auto_path.run_time_flag = 0;
        nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
        foot.up_tor_feedforward_flag = 0;
        foot.down_tor_feedforward_flag = 0;
        j60_motor_cmd_down2.kp_ = 25.f, j60_motor_cmd_down2.kd_ = 4.f;
        j60_motor_cmd_down2.j60_td.r = 1000.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
        go1_send_left.K_P = 0.6f, go1_send_left.K_W = 0.06f;
        go1_send_left.td.r = 5000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
        go1_send_right.K_P = 0.6f, go1_send_right.K_W = 0.06f;
        go1_send_right.td.r = 5000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
        break;

    case FOOT_UPSTAIRS_LV1: // 小脚当前状态--->上200台阶
        switch (foot.foot_up_lv1)
        {
        case UP11: // --->小脚触地准备上台阶
            nav.auto_path.run_time_flag = 0;
            nav.nav_state = SEMIAUTO_UP_DOWN_STAIRS;
            nav.auto_path.up_down_state = UP_APPROACH_1;
            Foot_Shorten();
            foot.up_tor_feedforward_flag = 0;
            foot.down_tor_feedforward_flag = 0;
            j60_motor_cmd_down2.kp_ = 25.f, j60_motor_cmd_down2.kd_ = 4.f;
            j60_motor_cmd_down2.j60_td.r = 100000.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
            go1_send_left.K_P = 1.f, go1_send_left.K_W = 0.06f;
            go1_send_left.td.r = 1000000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
            go1_send_right.K_P = 1.f, go1_send_right.K_W = 0.06f;
            go1_send_right.td.r = 1000000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
            foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f + 18.f) / -9.0515f - 21.5f + go1_left_0;
            foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f + 18.f) / 9.0515f + 21.5f + go1_right_0;
            foot.down_aim_angle = 12.f / -57.f - 1.7f;
            if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 0.633f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 0.633f && fabs(j60_motor_data_down2.position_ - foot.down_aim_angle) < 0.1f)
            {
                foot.foot_up_lv1 = UP12;
            }
            break;

        case UP12: // ---> 小脚站立，主动轮驱动车身向前
            foot.runtime_flag = 0;
            foot.up_tor_feedforward_flag = 1;
            foot.down_tor_feedforward_flag = 1;
            j60_motor_cmd_down2.kp_ = 150.f, j60_motor_cmd_down2.kd_ = 10.f;
            j60_motor_cmd_down2.j60_td.r = 100000.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
            go1_send_left.K_P = 2.8f, go1_send_left.K_W = 0.2f;
            go1_send_left.td.r = 1000000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
            go1_send_right.K_P = 2.8f, go1_send_right.K_W = 0.2f;
            go1_send_right.td.r = 1000000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
            foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f + 88.f) / -9.0515f - 21.5f + go1_left_0;
            foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f + 88.f) / 9.0515f + 21.5f + go1_right_0;
            foot.down_aim_angle = 80.f / -57.f - 1.7f;
            if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 1.266f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 1.266f && fabs(j60_motor_data_down2.position_ - foot.down_aim_angle) < 0.3f)
            {
                foot.foot_up_lv1 = UP13;
                // 起立后，启动小脚主动轮
                nav.auto_path.run_time_flag = 0;
                nav.nav_state = SEMIAUTO_UP_DOWN_STAIRS;
                nav.auto_path.up_down_state = UP_APPROACH_2;
            }
            break;

        case UP13: // ---> 收起前脚
            if (foot.runtime_flag == 0 && travel_switch_mode[3])
            { // 向下的红外行程开关检测到车身由空中下降到台阶上，开始计时
                foot.runtime = 0;
                foot.runtime_flag = 1;
                // 到达台阶后，关闭小脚主动轮，由舵轮驱动车身向前
                nav.auto_path.run_time_flag = 0;
                nav.auto_path.up_down_state = UP_UPSTAIRS;
            }
            if (foot.runtime_flag == 1 && foot.runtime > 5)
            { // 红外行程开关检测到前轮台阶5ms后
                foot.up_tor_feedforward_flag = 0;
                foot.down_tor_feedforward_flag = 1;
                go1_send_left.K_P = 1.5f, go1_send_left.K_W = 0.08f;
                go1_send_left.td.r = 1000000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
                go1_send_right.K_P = 1.5f, go1_send_right.K_W = 0.08f;
                go1_send_right.td.r = 1000000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
                foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f + 180.f) / -9.0515f - 21.5f + go1_left_0;
                foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f + 180.f) / 9.0515f + 21.5f + go1_right_0;
            }
            if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 0.633f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 0.633f && foot.up_tor_feedforward_flag == 0)
            { // 到达目标角度 && 前脚成功上台阶
                foot.foot_up_lv1 = UP14;
                foot.runtime_flag = 0;
            }
            break;

        case UP14: // ---> 收起后脚
            if (foot.runtime_flag == 0 && travel_switch_mode[4])
            {
                foot.runtime = 0;
                foot.runtime_flag = 1;
                nav.nav_state = NAV_POINT_TO_POINT; // 开始收后脚时，底盘启动闭环控制
            }
            if (foot.runtime_flag == 1 && foot.runtime > 5)
            {
                foot.up_tor_feedforward_flag = 0;
                foot.down_tor_feedforward_flag = 0;
                j60_motor_cmd_down2.kp_ = 25.f, j60_motor_cmd_down2.kd_ = 4.f;
                j60_motor_cmd_down2.j60_td.r = 100000.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
                foot.down_aim_angle = 0.f / -57.f - 1.7f;
                nav.auto_path.run_time_flag = 0;
                nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
            }
            if (fabs(j60_motor_data_down2.position_ - foot.down_aim_angle) < 0.3f && foot.down_tor_feedforward_flag == 0)
            {
                foot.foot_state = FOOT_INIT;
                foot.foot_up_lv1 = UP11;
                foot.runtime_flag = 0;
                foot.up_foot_circle_count++;
                // 上台阶完成，go1电机圈数加一
            }
            break;

        default:
            break;
        }
        break;

    case FOOT_UPSTAIRS_LV2: // 小脚当前状态--->上400台阶
        switch (foot.foot_up_lv2)
        {
        case UP21: // --->小脚触地准备上台阶
            nav.auto_path.run_time_flag = 0;
            nav.nav_state = SEMIAUTO_UP_DOWN_STAIRS;
            nav.auto_path.up_down_state = UP_APPROACH_3;
            // Foot_Lengthen();
            foot.up_tor_feedforward_flag = 0;
            foot.down_tor_feedforward_flag = 0;
            j60_motor_cmd_down2.kp_ = 25.f, j60_motor_cmd_down2.kd_ = 4.f;
            j60_motor_cmd_down2.j60_td.r = 100000.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
            go1_send_left.K_P = 1.f, go1_send_left.K_W = 0.06f;
            go1_send_left.td.r = 1000000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
            go1_send_right.K_P = 1.f, go1_send_right.K_W = 0.06f;
            go1_send_right.td.r = 1000000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
            foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f + 18.f) / -9.0515f - 21.5f + go1_left_0;
            foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f + 18.f) / 9.0515f + 21.5f + go1_right_0;
            foot.down_aim_angle = 12.f / -57.f - 1.7f;
            if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 0.633f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 0.633f && fabs(j60_motor_data_down2.position_ - foot.down_aim_angle) < 0.1f)
            {
                foot.foot_up_lv2 = UP22;
                Foot_Lengthen();
            }
            break;

        case UP22: // ---> 小脚站立，主动轮驱动车身向前
            foot.runtime_flag = 0;
            foot.up_tor_feedforward_flag = 1;
            foot.down_tor_feedforward_flag = 1;
            j60_motor_cmd_down2.kp_ = 300.f, j60_motor_cmd_down2.kd_ = 10.f;
            j60_motor_cmd_down2.j60_td.r = 100000.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
            go1_send_left.K_P = 2.8f, go1_send_left.K_W = 0.2f;
            go1_send_left.td.r = 1000000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
            go1_send_right.K_P = 3.f, go1_send_right.K_W = 0.2f;
            go1_send_right.td.r = 1000000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
            foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f + 89.f) / -9.0515f - 21.56f + go1_left_0;
            foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f + 90.7f) / 9.0515f + 21.38f + go1_right_0;
            foot.down_aim_angle = 78.f / -57.f - 1.7f;
            if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 0.633f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 0.633f && fabs(j60_motor_data_down2.position_ - foot.down_aim_angle) < 0.15f)
            {
                foot.foot_up_lv2 = UP23;
                // 起立后，启动小脚主动轮
                nav.auto_path.run_time_flag = 0;
                nav.nav_state = SEMIAUTO_UP_DOWN_STAIRS;
                nav.auto_path.up_down_state = UP_APPROACH_4;
                // 完成起立后，小脚伸长
                //Foot_Lengthen();
            }
            break;

        case UP23: // ---> 收起前脚
            if (foot.runtime_flag == 0 && j60_motor_data_down2.position_ < (88.f / -57.f - 1.7f))
            {
                go1_send_left.K_P = 0.7f, go1_send_left.K_W = 0.08f;
                go1_send_right.K_P = 0.7f, go1_send_right.K_W = 0.08f;
            }
            if (foot.runtime_flag == 0 && travel_switch_mode[3])
            { // 向下的红外行程开关检测到车身由空中下降到台阶上，开始计时
                foot.runtime = 0;
                foot.runtime_flag = 1;
                // 到达台阶后，关闭小脚主动轮，由舵轮驱动车身向前
                nav.auto_path.run_time_flag = 0;
                nav.auto_path.up_down_state = UP_UPSTAIRS;
            }
            if (foot.runtime_flag == 1 && foot.runtime > 50)
            { // 红外行程开关检测到前轮台阶5ms后
                foot.up_tor_feedforward_flag = 0;
                foot.down_tor_feedforward_flag = 1;
                go1_send_left.K_P = 1.6f, go1_send_left.K_W = 0.08f;
                go1_send_left.td.r = 1000000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
                go1_send_right.K_P = 1.6f, go1_send_right.K_W = 0.08f;
                go1_send_right.td.r = 1000000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
                foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f + 180.f) / -9.0515f - 21.5f + go1_left_0;
                foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f + 180.f) / 9.0515f + 21.5f + go1_right_0;
            }
            if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 0.633f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 0.633f && foot.up_tor_feedforward_flag == 0)
            { // 到达目标角度 && 前脚成功上台阶
                foot.foot_up_lv2 = UP24;
                foot.runtime_flag = 0;
            }
            break;

        case UP24: // ---> 收起后脚
            if (foot.runtime_flag == 0 && travel_switch_mode[4])
            {
                foot.runtime = 0;
                foot.runtime_flag = 1;
                nav.nav_state = NAV_POINT_TO_POINT; // 开始收后脚时，底盘启动闭环控制
            }
            if (foot.runtime_flag == 1 && foot.runtime > 5)
            {
                foot.up_tor_feedforward_flag = 0;
                foot.down_tor_feedforward_flag = 0;
                j60_motor_cmd_down2.kp_ = 25.f, j60_motor_cmd_down2.kd_ = 4.f;
                j60_motor_cmd_down2.j60_td.r = 100000.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
                foot.down_aim_angle = 0.f / -57.f - 1.7f;
                nav.auto_path.run_time_flag = 0;
                nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
                Foot_Shorten();
            }
            if (fabs(j60_motor_data_down2.position_ - foot.down_aim_angle) < 0.3f && foot.down_tor_feedforward_flag == 0)
            {
                foot.foot_state = FOOT_INIT;
                foot.foot_up_lv2 = UP21;
                foot.runtime_flag = 0;
                foot.up_foot_circle_count++;
                // 上台阶完成，go1电机圈数加一
            }
            break;

        default:
            break;
        }
        break;

    case FOOT_DOWNSTAIRS_LV1: // 小脚当前状态--->下200台阶
        switch (foot.foot_down_lv1)
        {
        case DOWN11: // --->小脚触地准备下台阶
            nav.auto_path.run_time_flag = 0;
            nav.nav_state = SEMIAUTO_UP_DOWN_STAIRS;
            nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
            Foot_Shorten();
            foot.up_tor_feedforward_flag = 0;
            foot.down_tor_feedforward_flag = 0;
            j60_motor_cmd_down2.kp_ = 20.f, j60_motor_cmd_down2.kd_ = 3.f;
            j60_motor_cmd_down2.j60_td.r = 1000.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
            go1_send_left.K_P = 0.6f, go1_send_left.K_W = 0.06f;
            go1_send_left.td.r = 5000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
            go1_send_right.K_P = 0.6f, go1_send_right.K_W = 0.06f;
            go1_send_right.td.r = 5000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
            foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f - 190.f) / -9.0515f - 21.5f + go1_left_0;
            foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f - 190.f) / 9.0515f + 21.5f + go1_right_0;
            foot.down_aim_angle = 10.f / -57.f - 1.7f;
            if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 0.633f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 0.633f && fabs(j60_motor_data_down2.position_ - foot.down_aim_angle) < 0.1f)
            {
                foot.foot_down_lv1 = DOWN12;
            }
            break;

        case DOWN12: // ---> 后脚放下，舵轮驱动车身向后
            nav.auto_path.run_time_flag = 0;
            nav.auto_path.up_down_state = DOWN_APPROACH_AND_DOWNSTAIRS;
            foot.up_tor_feedforward_flag = 0;
            foot.down_tor_feedforward_flag = 1;
            j60_motor_cmd_down2.kp_ = 60.f, j60_motor_cmd_down2.kd_ = 8.f;
            j60_motor_cmd_down2.j60_td.r = 1000.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
            foot.down_aim_angle = 80.f / -57.f - 1.7f;
            if (fabs(j60_motor_data_down2.position_ - foot.down_aim_angle) < 0.3f)
            {
                foot.foot_down_lv1 = DOWN13;
            }
            break;

        case DOWN13: // ---> 前脚放下，车身运动停止
            if (foot.runtime_flag == 0)
            {
                foot.runtime = 0;
                foot.runtime_flag = 1;
            }
            if (foot.runtime > 400)
            {
                foot.up_tor_feedforward_flag = 1;
                foot.down_tor_feedforward_flag = 1;
                go1_send_left.K_P = 1.5f, go1_send_left.K_W = 0.08;
                go1_send_left.td.r = 3000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
                go1_send_right.K_P = 1.5f, go1_send_right.K_W = 0.08f;
                go1_send_right.td.r = 3000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
                foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f - 270.f) / -9.0515f - 21.5f + go1_left_0;
                foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f - 270.f) / 9.0515f + 21.5f + go1_right_0;
            }
            if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 1.266f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 1.266f && foot.up_tor_feedforward_flag == 1)
            {
                foot.foot_down_lv1 = DOWN14;
                foot.runtime_flag = 0;
            }
            break;

        case DOWN14: // ---> 小脚坐下，完成下台阶
            if (foot.runtime_flag == 0)
            {
                foot.runtime = 0;
                foot.runtime_flag = 1;
            }
            nav.auto_path.run_time_flag = 0;
            nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
            foot.up_tor_feedforward_flag = 1;
            foot.down_tor_feedforward_flag = 1;
            j60_motor_cmd_down2.kp_ = 35.f, j60_motor_cmd_down2.kd_ = 6.f;
            j60_motor_cmd_down2.j60_td.r = 10.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
            go1_send_left.K_P = 1.5f, go1_send_left.K_W = 0.08f;
            go1_send_left.td.r = 45.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
            go1_send_right.K_P = 1.5f, go1_send_right.K_W = 0.08f;
            go1_send_right.td.r = 50.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
            foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f - 350.f) / -9.0515f - 21.5f + go1_left_0;
            foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f - 350.f) / 9.0515f + 21.5f + go1_right_0;
            foot.down_aim_angle = 10.f / -57.f - 1.7f;
            if (foot.runtime > 800)
            {
                // 定时关闭重力前馈，防止小脚因重力前馈阻碍到达目标位置
                foot.up_tor_feedforward_flag = 0;
                foot.down_tor_feedforward_flag = 0;
            }
            if ((fabs(j60_motor_data_down2.position_ - foot.down_aim_angle) < 0.3f && fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 1.266f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 1.266f) || foot.runtime > 1000)
            { // 到达目标位置 || 超时摔下台阶  或逻辑用以保证执行go1电机转圈计数减一
                foot.foot_state = FOOT_INIT;
                foot.foot_down_lv1 = DOWN11;
                foot.runtime_flag = 0;
                foot.up_foot_circle_count--;
                // 下台阶完成，go1电机圈数减一
            }
            break;

        default:
            break;
        }
        break;

    case FOOT_DOWNSTAIRS_LV2: // 小脚当前状态--->下400台阶
        switch (foot.foot_down_lv2)
        {
        case DOWN21: // --->小脚触地准备下台阶
            if (foot.runtime_flag == 0)
            {
                foot.runtime = 0;
                foot.runtime_flag = 1;
            }
            nav.auto_path.run_time_flag = 0;
            nav.nav_state = SEMIAUTO_UP_DOWN_STAIRS;
            nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
            Foot_Lengthen();
            foot.up_tor_feedforward_flag = 0;
            foot.down_tor_feedforward_flag = 0;
            if (foot.runtime > 2000)
            {
                j60_motor_cmd_down2.kp_ = 20.f, j60_motor_cmd_down2.kd_ = 3.f;
                j60_motor_cmd_down2.j60_td.r = 1000.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
                go1_send_left.K_P = 0.6f, go1_send_left.K_W = 0.06f;
                go1_send_left.td.r = 5000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
                go1_send_right.K_P = 0.6f, go1_send_right.K_W = 0.06f;
                go1_send_right.td.r = 5000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
                foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f - 190.f) / -9.0515f - 21.5f + go1_left_0;
                foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f - 190.f) / 9.0515f + 21.5f + go1_right_0;
                foot.down_aim_angle = 10.f / -57.f - 1.7f;
            }
            if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 0.633f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 0.633f && fabs(j60_motor_data_down2.position_ - foot.down_aim_angle) < 0.1f && foot.runtime > 2600)
            {
                foot.foot_down_lv2 = DOWN22;
                foot.runtime_flag = 0;
            }
            break;

        case DOWN22: // ---> 后脚放下，舵轮驱动车身向后
            nav.auto_path.run_time_flag = 0;
            nav.auto_path.up_down_state = DOWN_APPROACH_AND_DOWNSTAIRS;
            foot.up_tor_feedforward_flag = 0;
            foot.down_tor_feedforward_flag = 1;
            j60_motor_cmd_down2.kp_ = 90.f, j60_motor_cmd_down2.kd_ = 8.f;
            j60_motor_cmd_down2.j60_td.r = 1000.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
            foot.down_aim_angle = 75.f / -57.f - 1.7f;
            if (fabs(j60_motor_data_down2.position_ - foot.down_aim_angle) < 0.2f)
            {
                foot.foot_down_lv2 = DOWN23;
            }
            break;

        case DOWN23: // ---> 前脚放下，车身运动停止
            if (foot.runtime_flag == 0)
            {
                foot.runtime = 0;
                foot.runtime_flag = 1;
            }
            if (foot.runtime > 400)
            {
                foot.up_tor_feedforward_flag = 1;
                foot.down_tor_feedforward_flag = 1;
                go1_send_left.K_P = 1.5f, go1_send_left.K_W = 0.08;
                go1_send_left.td.r = 3000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
                go1_send_right.K_P = 1.5f, go1_send_right.K_W = 0.08f;
                go1_send_right.td.r = 3000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
                foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f - 275.f) / -9.0515f - 21.5f + go1_left_0;
                foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f - 275.f) / 9.0515f + 21.5f + go1_right_0;
            }
            if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 1.266f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 1.266f && foot.up_tor_feedforward_flag == 1)
            {
                foot.foot_down_lv2 = DOWN24;
                foot.runtime_flag = 0;
                Foot_Shorten();
            }
            break;

        case DOWN24: // ---> 小脚坐下，完成下台阶
            if (foot.runtime_flag == 0)
            {
                foot.runtime = 0;
                foot.runtime_flag = 1;
            }
            nav.auto_path.run_time_flag = 0;
            nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
            foot.up_tor_feedforward_flag = 1;
            foot.down_tor_feedforward_flag = 1;
            j60_motor_cmd_down2.kp_ = 35.f, j60_motor_cmd_down2.kd_ = 6.f;
            j60_motor_cmd_down2.j60_td.r = 10.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
            go1_send_left.K_P = 1.5f, go1_send_left.K_W = 0.08f;
            go1_send_left.td.r = 35.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
            go1_send_right.K_P = 1.5f, go1_send_right.K_W = 0.08f;
            go1_send_right.td.r = 40.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
            foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f - 350.f) / -9.0515f - 21.5f + go1_left_0;
            foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f - 350.f) / 9.0515f + 21.5f + go1_right_0;
            foot.down_aim_angle = 10.f / -57.f - 1.7f;
            if (foot.runtime > 900)
            {
                // 定时关闭重力前馈，防止小脚因重力前馈阻碍到达目标位置
                foot.up_tor_feedforward_flag = 0;
                foot.down_tor_feedforward_flag = 0;
            }
            if ((fabs(j60_motor_data_down2.position_ - foot.down_aim_angle) < 0.3f && fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 1.266f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 1.266f) || foot.runtime > 1000)
            { // 到达目标位置 || 超时摔下台阶  或逻辑用以保证执行go1电机转圈计数减一
                foot.foot_state = FOOT_INIT;
                foot.foot_down_lv2 = DOWN21;
                foot.runtime_flag = 0;
                foot.up_foot_circle_count--;
                // 下台阶完成，go1电机圈数减一
            }
            break;

        default:
            break;
        }
        break;

    // 没调过，慎用！！可进一步完善
    // 以下为测试状态，准备上台阶--->站立--->坐下--->准备上台阶循环
    case FOOT_TEST_STAND_PREPARE: // 小脚当前状态--->准备上台阶
        nav.auto_path.run_time_flag = 0;
        nav.nav_state = SEMIAUTO_UP_DOWN_STAIRS;
        nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
        foot.up_tor_feedforward_flag = 0;
        foot.down_tor_feedforward_flag = 0;
        j60_motor_cmd_down2.kp_ = 20.f, j60_motor_cmd_down2.kd_ = 3.f;
        j60_motor_cmd_down2.j60_td.r = 1000.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
        go1_send_left.K_P = 0.6f, go1_send_left.K_W = 0.06f;
        go1_send_left.td.r = 5000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
        go1_send_right.K_P = 0.6f, go1_send_right.K_W = 0.06f;
        go1_send_right.td.r = 5000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
        foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f + 18.f) / -9.0515f - 21.5f + go1_left_0;
        foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f + 18.f) / 9.0515f + 21.5f + go1_right_0;
        foot.down_aim_angle = 12.f / -57.f - 1.7f;
        if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 0.633f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 0.633f && fabs(j60_motor_data_down2.position_ - foot.down_aim_angle) < 0.1f)
        {
            foot.foot_state = FOOT_CLEAR;
            foot.runtime_flag = 0;
        }
        break;

    case FOOT_TEST_STAND: // 小脚当前状态--->站立
        nav.auto_path.run_time_flag = 0;
        nav.nav_state = SEMIAUTO_UP_DOWN_STAIRS;
        nav.auto_path.up_down_state = UP_DOWN_STATE_OFF;
        foot.up_tor_feedforward_flag = 1;
        foot.down_tor_feedforward_flag = 1;
        j60_motor_cmd_down2.kp_ = 35.f, j60_motor_cmd_down2.kd_ = 6.f;
        j60_motor_cmd_down2.j60_td.r = 1000.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
        go1_send_left.K_P = 2.f, go1_send_left.K_W = 0.1f;
        go1_send_left.td.r = 20000.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
        go1_send_right.K_P = 2.f, go1_send_right.K_W = 0.1f;
        go1_send_right.td.r = 20000.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
        foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f + 88.f) / -9.0515f - 21.5f + go1_left_0;
        foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f + 88.f) / 9.0515f + 21.5f + go1_right_0;
        foot.down_aim_angle = 89.f / -57.f - 1.7f;
        if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 1.266f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 1.266f && fabs(j60_motor_data_down2.position_ - foot.down_aim_angle) < 0.3f)
        {
            foot.foot_state = FOOT_CLEAR;
        }
        break;

    case FOOT_TEST_SIT: // 小脚当前状态--->坐下
        if (foot.runtime_flag == 0)
        {
            foot.runtime = 0;
            foot.runtime_flag = 1;
        }
        foot.up_tor_feedforward_flag = 1;
        foot.down_tor_feedforward_flag = 1;
        j60_motor_cmd_down2.kp_ = 35.f, j60_motor_cmd_down2.kd_ = 6.f;
        j60_motor_cmd_down2.j60_td.r = 8.f, j60_motor_cmd_down2.j60_td.h = 0.001f, j60_motor_cmd_down2.j60_td.T = 0.001f;
        go1_send_left.K_P = 1.5f, go1_send_left.K_W = 0.08f;
        go1_send_left.td.r = 35.f, go1_send_left.td.h = 0.001f, go1_send_left.td.T = 0.001f;
        go1_send_right.K_P = 1.5f, go1_send_right.K_W = 0.08f;
        go1_send_right.td.r = 35.f, go1_send_right.td.h = 0.001f, go1_send_right.td.T = 0.001f;
        foot.leftup_aim_angle = (foot.up_foot_circle_count * 360.f - 350.f) / -9.0515f - 21.5f + go1_left_0;
        foot.rightup_aim_angle = (foot.up_foot_circle_count * 360.f - 350.f) / 9.0515f + 21.5f + go1_right_0;
        foot.down_aim_angle = 10.f / -57.f - 1.7f;
        if (foot.runtime >= 1000)
        {
            foot.up_tor_feedforward_flag = 0;
            foot.down_tor_feedforward_flag = 0;
        }
        if (fabs(go1_recv_left.Pos - foot.leftup_aim_angle) < 2.f && fabs(go1_recv_right.Pos - foot.rightup_aim_angle) < 2.f && fabs(j60_motor_data_down2.position_ - foot.down_aim_angle) < 0.5f || foot.runtime > 1200)
        {
            foot.foot_state = FOOT_INIT;
            foot.foot_up_lv1 = UP11;
            foot.foot_up_lv2 = UP21;
            foot.foot_down_lv1 = DOWN11;
            foot.foot_down_lv2 = DOWN21;
            foot.runtime_flag = 0;
        }
        break;

    default:
        break;
    }
}
