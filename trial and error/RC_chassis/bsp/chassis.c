#include "chassis.h"

chassis_run_des friction_compensation_current;
void chassis_friction_compensation(void)
{
    // 补偿摩擦力
    friction_compensation_current.leftup = 2000 * chassis_run.leftup.fpDes / 7.0f;
    friction_compensation_current.rightup = 2000 * chassis_run.rightup.fpDes / 7.0f;
    friction_compensation_current.rightdown = 2000 * chassis_run.rightdown.fpDes / 7.0f;
    friction_compensation_current.leftdown = 2000 * chassis_run.leftdown.fpDes / 7.0f;

    friction_compensation_current.leftup = ClipFloat(friction_compensation_current.leftup, -2000.0f, 2000.0f);
    friction_compensation_current.rightup = ClipFloat(friction_compensation_current.rightup, -2000.0f, 2000.0f);
    friction_compensation_current.rightdown = ClipFloat(friction_compensation_current.rightdown, -2000.0f, 2000.0f);
    friction_compensation_current.leftdown = ClipFloat(friction_compensation_current.leftdown, -2000.0f, 2000.0f);
}

/****************************************************************************************************
函数名称: void SpeedDistribute_Four_OmnidriectionalWhile(cNav *p_nav)

函数功能: 根据全局坐标下的速度期望，给四个全向轮分配速度
备   注:传入的角速度是弧度制
比之前写的底盘结算更加严谨，考虑到了各种数值，以使最后的的各种速度转化到轮子上是真实所需的速度
****************************************************************************************************/
chassis_velt_t velt_w;
ST_VECTOR expect_robot_local_Velt;
chassis_run_des straight_des, rotation_des;
ST_VECTOR pos_leftup = {-LENGTH, WIDTH, 0, 0, 0};
ST_VECTOR pos_rightup = {LENGTH, WIDTH, 0, 0, 0};
ST_VECTOR pos_rightdown = {LENGTH, -WIDTH, 0, 0, 0};
ST_VECTOR pos_leftdown = {-LENGTH, -WIDTH, 0, 0, 0}; // 四个全向轮距离车中心的位矢，直线运动不需要该参数，只有旋转运动需要
void SpeedDistribute_Four_OmnidirectionalWheel(ST_Nav *p_nav)
{
    /*		  |
          /   |   \ (LENGTH,WIDTH)
             \|/
        -------------->
             /|\
          \   |   /
              |
        四个全向轮
    */
    float fpQ=0;
    if (p_nav->nav_state == NAV_LOCAL_MANUAL || p_nav->nav_state == NAV_GLOBAL_MANUAL){// 以车身坐标系操控
        fpQ = 0;
    }else{
        fpQ = ConvertAngle(stRobot.stPos.fpPosQ * RADIAN_10); // 在自动导航下，获取车身的偏航角(°)，转化为弧度，便于利用cos计算
    }

    Concert_coorindnate(&p_nav->expect_robot_global_velt, &expect_robot_local_Velt, fpQ); // 全局坐标系（正直角坐标系）的速度分配到局部坐标系（斜的直角坐标系）

    expect_robot_local_Velt.fpW = p_nav->expect_robot_global_velt.fpW * RADIAN;
    expect_robot_local_Velt.type = CARTESIAN;

    straight_des.rightup = (-sin(PI / 4) * expect_robot_local_Velt.fpX - sin(PI / 4) * expect_robot_local_Velt.fpY) / R_WHEEL * RUN_GEAR_RATIO;
    straight_des.rightdown = (-sin(PI / 4) * expect_robot_local_Velt.fpX + sin(PI / 4) * expect_robot_local_Velt.fpY) / R_WHEEL * RUN_GEAR_RATIO;
    straight_des.leftdown = (sin(PI / 4) * expect_robot_local_Velt.fpX + sin(PI / 4) * expect_robot_local_Velt.fpY) / R_WHEEL * RUN_GEAR_RATIO;
    straight_des.leftup = (sin(PI / 4) * expect_robot_local_Velt.fpX - sin(PI / 4) * expect_robot_local_Velt.fpY) / R_WHEEL * RUN_GEAR_RATIO;

    rotation_des.leftup = R_ROBOT * expect_robot_local_Velt.fpW / R_WHEEL * RUN_GEAR_RATIO;
    rotation_des.rightup = R_ROBOT * expect_robot_local_Velt.fpW / R_WHEEL * RUN_GEAR_RATIO;
    rotation_des.rightdown = R_ROBOT * expect_robot_local_Velt.fpW / R_WHEEL * RUN_GEAR_RATIO;
    rotation_des.leftdown = R_ROBOT * expect_robot_local_Velt.fpW / R_WHEEL * RUN_GEAR_RATIO;

    // 目标速度
    chassis_run.leftup.fpDes = straight_des.leftup + rotation_des.leftup; // 最后将车身直行和车身旋转时电机所需的角速度加起来分配到电机即可
    chassis_run.rightup.fpDes = straight_des.rightup + rotation_des.rightup;
    chassis_run.rightdown.fpDes = straight_des.rightdown + rotation_des.rightdown;
    chassis_run.leftdown.fpDes = straight_des.leftdown + rotation_des.leftdown;

    //chassis_feed_forward(straight_des, rotation_des);
    //chassis_friction_compensation();
}

chassis_run_des feed_forward_current;
chassis_run_des pre_straight_speed_fpDes, now_straight_speed_fpDes;
float K_Feed_Forward = 3;        // 8;
float K1_Feed_Forward_LU = 4000; // 15000;
float K1_Feed_Forward_RU = 4000; // 15000;
float K1_Feed_Forward_RD = 4000; // 15000;
float K1_Feed_Forward_LD = 4000; // 15000;
void chassis_feed_forward(chassis_run_des straight_des, chassis_run_des rotation_des)
{
    now_straight_speed_fpDes.leftup = straight_des.leftup + rotation_des.leftup;
    now_straight_speed_fpDes.rightup = straight_des.rightup + rotation_des.rightup;
    now_straight_speed_fpDes.rightdown = straight_des.rightdown + rotation_des.rightdown;
    now_straight_speed_fpDes.leftdown = straight_des.leftdown + rotation_des.leftdown;

    feed_forward_current.leftup = ClipFloat((now_straight_speed_fpDes.leftup - pre_straight_speed_fpDes.leftup) * K1_Feed_Forward_LU + K_Feed_Forward * now_straight_speed_fpDes.leftup, -4000, 4000);
    feed_forward_current.rightup = ClipFloat((now_straight_speed_fpDes.rightup - pre_straight_speed_fpDes.rightup) * K1_Feed_Forward_RU + K_Feed_Forward * now_straight_speed_fpDes.rightup, -4000, 4000);
    feed_forward_current.rightdown = ClipFloat((now_straight_speed_fpDes.rightdown - pre_straight_speed_fpDes.rightdown) * K1_Feed_Forward_RD + K_Feed_Forward * now_straight_speed_fpDes.rightdown, -4000, 4000);
    feed_forward_current.leftdown = ClipFloat((now_straight_speed_fpDes.leftdown - pre_straight_speed_fpDes.leftdown) * K1_Feed_Forward_LD + K_Feed_Forward * now_straight_speed_fpDes.leftdown, -4000, 4000);

    pre_straight_speed_fpDes.leftup = now_straight_speed_fpDes.leftup;
    pre_straight_speed_fpDes.rightup = now_straight_speed_fpDes.rightup;
    pre_straight_speed_fpDes.rightdown = now_straight_speed_fpDes.rightdown;
    pre_straight_speed_fpDes.leftdown = now_straight_speed_fpDes.leftdown;
}

// by hzy
chassis_run_des steer_velt, steer_pos, steer_pos_pre;
chassis_run_des current_steer_velt, current_steer_pos;
chassis_run_des now_velt, pre_velt, now_pos, pre_pos;
void SpeedDistribute_Four_SteeringWheel(ST_Nav *p_nav)
{
    float fpQ;
    if (p_nav->nav_state == NAV_LOCAL_MANUAL || p_nav->nav_state == NAV_GLOBAL_MANUAL)
    { // 以车身坐标系操控
        fpQ = 0;
    }
    else
    {
        fpQ = ConvertAngle(stRobot.stPos.fpPosQ * RADIAN_10);
    }
    Concert_coorindnate(&p_nav->expect_robot_global_velt, &expect_robot_local_Velt, fpQ);

    if (fabs(expect_robot_local_Velt.fpW) < 0.5f)
        expect_robot_local_Velt.fpW = 0.f;
    // 左右侧电机转向正方向相反
    steer_velt.leftup = -powf(powf(expect_robot_local_Velt.fpX - R_ROBOT * expect_robot_local_Velt.fpW * cosf(PI / 4), 2) + powf(expect_robot_local_Velt.fpY - R_ROBOT * expect_robot_local_Velt.fpW * sinf(PI / 4), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
    steer_velt.rightup = powf(powf(expect_robot_local_Velt.fpX - R_ROBOT * expect_robot_local_Velt.fpW * cosf(PI / 4), 2) + powf(expect_robot_local_Velt.fpY + R_ROBOT * expect_robot_local_Velt.fpW * sinf(PI / 4), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
    steer_velt.rightdown = powf(powf(expect_robot_local_Velt.fpX + R_ROBOT * expect_robot_local_Velt.fpW * cosf(PI / 4), 2) + powf(expect_robot_local_Velt.fpY + R_ROBOT * expect_robot_local_Velt.fpW * sinf(PI / 4), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
    steer_velt.leftdown = -powf(powf(expect_robot_local_Velt.fpX + R_ROBOT * expect_robot_local_Velt.fpW * cosf(PI / 4), 2) + powf(expect_robot_local_Velt.fpY - R_ROBOT * expect_robot_local_Velt.fpW * sinf(PI / 4), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;

    if (p_nav->nav_state == NAV_LOCAL_MANUAL)
    {
        steer_pos.leftup = atan2f(expect_robot_local_Velt.fpX - R_ROBOT * expect_robot_local_Velt.fpW * cosf(PI / 4), expect_robot_local_Velt.fpY - R_ROBOT * expect_robot_local_Velt.fpW * sinf(PI / 4)) / PI * 180.f;
        steer_pos.leftdown = atan2f(expect_robot_local_Velt.fpX + R_ROBOT * expect_robot_local_Velt.fpW * cosf(PI / 4), expect_robot_local_Velt.fpY - R_ROBOT * expect_robot_local_Velt.fpW * sinf(PI / 4)) / PI * 180.f;
        steer_pos.rightup = atan2f(expect_robot_local_Velt.fpX - R_ROBOT * expect_robot_local_Velt.fpW * cosf(PI / 4), expect_robot_local_Velt.fpY + R_ROBOT * expect_robot_local_Velt.fpW * sinf(PI / 4)) / PI * 180.f;
        steer_pos.rightdown = atan2f(expect_robot_local_Velt.fpX + R_ROBOT * expect_robot_local_Velt.fpW * cosf(PI / 4), expect_robot_local_Velt.fpY + R_ROBOT * expect_robot_local_Velt.fpW * sinf(PI / 4)) / PI * 180.f;

        // 速度过小时，arctan无法准确反映方向，直接置零
        if (fabs(steer_velt.leftup) < 20.f)
            steer_pos.leftup = steer_pos_pre.leftup;
        if (fabs(steer_velt.leftdown) < 20.f)
            steer_pos.leftdown = steer_pos_pre.leftdown;
        if (fabs(steer_velt.rightup) < 20.f)
            steer_pos.rightup = steer_pos_pre.rightup;
        if (fabs(steer_velt.rightdown) < 20.f)
            steer_pos.rightdown = steer_pos_pre.rightdown;

        // 处理转向跳变问题
        if (fabs(steer_pos.rightdown - steer_pos_pre.rightdown) > 90.f)
        {
            if (steer_pos.rightdown > steer_pos_pre.rightdown)
                steer_pos.rightdown -= 180.f, steer_velt.rightdown = -steer_velt.rightdown;
            else
                steer_pos.rightdown += 180.f, steer_velt.rightdown = -steer_velt.rightdown;
        }
        if (fabs(steer_pos.rightup - steer_pos_pre.rightup) > 90.f)
        {
            if (steer_pos.rightup > steer_pos_pre.rightup)
                steer_pos.rightup -= 180.f, steer_velt.rightup = -steer_velt.rightup;
            else
                steer_pos.rightup += 180.f, steer_velt.rightup = -steer_velt.rightup;
        }
        if (fabs(steer_pos.leftdown - steer_pos_pre.leftdown) > 90.f)
        {
            if (steer_pos.leftdown > steer_pos_pre.leftdown)
                steer_pos.leftdown -= 180.f, steer_velt.leftdown = -steer_velt.leftdown;
            else
                steer_pos.leftdown += 180.f, steer_velt.leftdown = -steer_velt.leftdown;
        }
        if (fabs(steer_pos.leftup - steer_pos_pre.leftup) > 90.f)
        {
            if (steer_pos.leftup > steer_pos_pre.leftup)
                steer_pos.leftup -= 180.f, steer_velt.leftup = -steer_velt.leftup;
            else
                steer_pos.leftup += 180.f, steer_velt.leftup = -steer_velt.leftup;
        }

        steer_pos_pre.leftup = steer_pos.leftup;
        steer_pos_pre.leftdown = steer_pos.leftdown;
        steer_pos_pre.rightup = steer_pos.rightup;
        steer_pos_pre.rightdown = steer_pos.rightdown;

        chassis_run.leftup.fpDes = steer_velt.leftup;
        chassis_run.leftdown.fpDes = steer_velt.leftdown;
        chassis_run.rightdown.fpDes = steer_velt.rightdown;
        chassis_run.rightup.fpDes = steer_velt.rightup;
    }

    // +机械零点
    chassis_steer_angle.leftup_td.aim = steer_pos.leftup + leftup_init_angle;
    chassis_steer_angle.leftdown_td.aim = steer_pos.leftdown + leftdown_init_angle;
    chassis_steer_angle.rightdown_td.aim = steer_pos.rightdown + rightdown_init_angle;
    chassis_steer_angle.rightup_td.aim = steer_pos.rightup + rightup_init_angle;

    //steer_chassis_feed_forward();    // 仅驱动电机
    chassis_friction_compensation(); // 仅驱动电机
}

void steer_chassis_feed_forward()
{     
    now_velt.leftup = steer_velt.leftup;
    now_velt.rightup = steer_velt.rightup;
    now_velt.rightdown = steer_velt.rightdown;
    now_velt.leftdown = steer_velt.leftdown;

    current_steer_velt.leftup = ClipFloat((now_velt.leftup - pre_velt.leftup) * 4000 + 3 * now_velt.leftup, -4000, 4000);
    current_steer_velt.rightup = ClipFloat((now_velt.rightup - pre_velt.rightup) * 4000 + 3 * now_velt.rightup, -4000, 4000);
    current_steer_velt.rightdown = ClipFloat((now_velt.rightdown - pre_velt.rightdown) * 4000 + 3 * now_velt.rightdown, -4000, 4000);
    current_steer_velt.leftdown = ClipFloat((now_velt.leftdown - pre_velt.leftdown) * 4000 + 3 * now_velt.leftdown, -4000, 4000);

    pre_velt.leftup = now_velt.leftup;
    pre_velt.rightup = now_velt.rightup;
    pre_velt.rightdown = now_velt.rightdown;
    pre_velt.leftdown = now_velt.leftdown;
}

// float normalize_angle(float angle_deg)
// {
//     angle_deg = fmodf(angle_deg, 360.0f);
//     if (angle_deg < 0)
//     {
//         angle_deg += 360.0f;
//     }
//     // 处理接近360度的情况，将其转换为0度
//     if (fabsf(angle_deg - 360.0f) < 1.0f)
//     {
//         angle_deg = 0.0f;
//     }
//     // 映射到[-180, 180)
//     if (angle_deg >= 180.0f)
//     {
//         angle_deg -= 360.0f;
//     }
//     // 处理接近-180度的情况，将其转换为180度
//     if (fabsf(angle_deg + 180.0f) < 1.0f)
//     {
//         angle_deg = 180.0f;
//     }
//     return angle_deg;
// }
