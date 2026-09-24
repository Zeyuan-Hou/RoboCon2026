#include "chassis.h"

void SpeedDistribute_Four_SteeringWheel(ST_Nav *p_nav)
{
    float fpQ;
    if (p_nav->nav_state == RC_LOCAL || p_nav->nav_state == SEMIAUTO_UP_DOWN_STAIRS)
    { // 以车身坐标系操控
        fpQ = PI / 2.f;
    }else{
        // 以全场坐标系操控
        fpQ = ConvertAngle(stRobot.stPos.fpPosQ * RADIAN_10);
    }
    Convert_velt(&p_nav->expect_robot_global_velt, &expect_robot_local_Velt, fpQ);

    // 角速度死区
    if (fabs(expect_robot_local_Velt.fpW) < 0.1f) // rad/s
        expect_robot_local_Velt.fpW = 0.f;

    // 左右侧电机转向正方向相反！！
    steer_velt.leftup = -powf(powf(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), 2) + powf(expect_robot_local_Velt.fpY - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
    steer_velt.rightup = powf(powf(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), 2) + powf(expect_robot_local_Velt.fpY + UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE), 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
    steer_velt.down = -powf(powf(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW, 2) + powf(expect_robot_local_Velt.fpY, 2), 0.5) / R_WHEEL * RUN_GEAR_RATIO;
    
    steer_pos.leftup = atan2f(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), expect_robot_local_Velt.fpY - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE)) / PI * 180.f;
    steer_pos.rightup = atan2f(expect_robot_local_Velt.fpX - UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * sinf(UP_ANGLE), expect_robot_local_Velt.fpY + UP_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW * cosf(UP_ANGLE)) / PI * 180.f;
    steer_pos.down = atan2f(expect_robot_local_Velt.fpX + DOWN_WHEEL_TO_ROBOT * expect_robot_local_Velt.fpW, expect_robot_local_Velt.fpY) / PI * 180.f;

    // 速度过小时，arctan分式计算误差大，无法准确反映方向，保持上次位置
    if (fabsf(steer_velt.leftup) < 5.f)
        steer_pos.leftup = steer_pos_pre.leftup;
    if (fabsf(steer_velt.rightup) < 5.f)
        steer_pos.rightup = steer_pos_pre.rightup;
    if (fabsf(steer_velt.down) < 5.f)
        steer_pos.down = steer_pos_pre.down;

    swerve_optimize(steer_pos_pre.down, &steer_pos.down, &steer_velt.down);
    swerve_optimize(steer_pos_pre.rightup, &steer_pos.rightup, &steer_velt.rightup);
    swerve_optimize(steer_pos_pre.leftup, &steer_pos.leftup, &steer_velt.leftup);

    steer_pos_pre.leftup = steer_pos.leftup;
    steer_pos_pre.rightup = steer_pos.rightup;
    steer_pos_pre.down = steer_pos.down;

    chassis_run2.leftup.fpDes = steer_velt.leftup;
    chassis_run2.rightup.fpDes = steer_velt.rightup;
    chassis_run2.down.fpDes = steer_velt.down;

    // +机械零点
    leftup_turn_motor.Input = steer_pos.leftup * TURN_GEAR_RATIO + leftup_init_angle;
    rightup_turn_motor.Input = steer_pos.rightup * TURN_GEAR_RATIO + rightup_init_angle;
    down_turn_motor.Input = steer_pos.down * TURN_GEAR_RATIO + down_init_angle;
}

void Drive_Chassis()
{
    if (nav.nav_state != CHASSIS_OFF)
    {
        PI_Feedforward_Calc(&chassis_run2.down, chassis_run2.down.fpDes, chassis_run2.down.fpFB);
        PI_Feedforward_Calc(&chassis_run2.rightup, chassis_run2.rightup.fpDes, chassis_run2.rightup.fpFB);
        PI_Feedforward_Calc(&chassis_run2.leftup, chassis_run2.leftup.fpDes, chassis_run2.leftup.fpFB);

        DJI_ControlLoop(&down_turn_motor);
        DJI_ControlLoop(&rightup_turn_motor);
        DJI_ControlLoop(&leftup_turn_motor);
    }

    // 当且仅当小脚正在上台阶的第二阶段时，foot_motor才有输出，防止foot_motor在其他阶段干扰舵轮底盘控制
    if (nav.nav_state == SEMIAUTO_UP_DOWN_STAIRS && (nav.auto_path.up_down_state == UP_APPROACH_2 || nav.auto_path.up_down_state == UP_APPROACH_4 || nav.auto_path.up_down_state == UP_UPSTAIRS))
    {
        DJI_ControlLoop(&foot_motor);
    }
    else
    {
        foot_motor.motor_current = 0;
    }

    if (chassis_run2.feed_forward_state == WITH_FORWARD)
    {
        friction_compensation();
        cur_steer_velt.down = ClipFloat(chassis_run2.down.fpU + friction_feedforward.down, -15000.f, 15000.f);
        cur_steer_velt.leftup = ClipFloat(chassis_run2.leftup.fpU + friction_feedforward.leftup, -15000.f, 15000.f);
        cur_steer_velt.rightup = ClipFloat(chassis_run2.rightup.fpU + friction_feedforward.rightup, -15000.f, 15000.f);
    }
    else
    {
        cur_steer_velt.down = ClipFloat(chassis_run2.down.fpU, -15000.f, 15000.f);
        cur_steer_velt.leftup = ClipFloat(chassis_run2.leftup.fpU, -15000.f, 15000.f);
        cur_steer_velt.rightup = ClipFloat(chassis_run2.rightup.fpU, -15000.f, 15000.f);
    }

    CAN_SendCurrent(&hcan1, 0X200, down_turn_motor.motor_current, cur_steer_velt.down, foot_motor.motor_current, 0);
    CAN_SendCurrent(&hcan2, 0x200, leftup_turn_motor.motor_current, cur_steer_velt.leftup, rightup_turn_motor.motor_current, cur_steer_velt.rightup);
}

// 斜坡的重力补偿
void friction_compensation()
{
    friction_feedforward.leftup = ClipFloat(fric_k.leftup * steer_velt.leftup, -2000.f, 2000.f);
    friction_feedforward.rightup = ClipFloat(fric_k.rightup * steer_velt.rightup, -2000.f, 2000.f);
    friction_feedforward.down = ClipFloat(fric_k.down * steer_velt.down, -2000.f, 2000.f);
}

// 返回相对ref最近的target等效角（可能超出[-180,180]，用于连续命令）
static float angle_nearest_to(float target_deg, float ref_deg)
{
    float delta = fmodf(target_deg - ref_deg, 360.0f);
    if (delta > 180.0f)
        delta -= 360.0f;
    if (delta < -180.0f)
        delta += 360.0f;
    return ref_deg + delta;
}

// 舵轮转向优化，保证每次转向电机的转角至多为90°
static void swerve_optimize(float prev_deg, float *target_deg, float *wheel_vel)
{
    float cand = angle_nearest_to(*target_deg, prev_deg);
    float delta = cand - prev_deg;
    if (fabsf(delta) > 90.0f)
    {
        cand = angle_nearest_to(*target_deg + 180.0f, prev_deg);
        *wheel_vel = -*wheel_vel;
    }
    *target_deg = cand; // 可能超出[-180,180]，但保证与prev的数值差≤90°
}
