#include "chassis.h"

float global_remote_yaw = 0;
uint8_t global_remote_flag = 1;
ST_TD td_global_remote_yaw = {
    .h = 0.05f,
    .r = 80.f,
    .T = 0.001f};

float fixed_vel = 0;
float global_remote_yaw_ff_k = 0.9f;

void Chassis_Run(void)
{
    chassis_LED(Chassis.err);
		chassis_protection();
    switch (Chassis.state)
    {
    case CHASSIS_STANDBY:
        chassis_clear();
        break;
    case CHASSIS_INIT:
        chassis_init();
        break;
    case CHASSIS_LOCAL_REMOTE:
        Chassis.local_vel = Remotevel;
        SteerWheels_distribute(&Chassis.wheels, &Chassis.local_vel);
        chassis_pid_calc();
        break;
    case CHASSIS_GLOBAL_REMOTE:
//        GlobalVel_To_Local(&Chassis.local_vel, &Remotevel, location.fpPosQ);
//        if (fabsf(Remotevel.fpW) < 0.1f)
//        {
//            if (global_remote_flag == 1 && fabsf(wheeltobody_vel.fpW) < 0.5f)
//            {
//                chassis_remote_yaw_clear();
//                global_remote_flag = 0;
//                td_global_remote_yaw.x1 = location.fpPosQ;
//                td_global_remote_yaw.x2 = 0;
//            }
//
//            float vel_dir = atan2f(Chassis.local_vel.fpVy, Chassis.local_vel.fpVx) * RAD_TO_DEG - 90;
//            float vel = hypotf(Chassis.local_vel.fpVx, Chassis.local_vel.fpVy);
//            if (global_remote_flag == 0)
//            {
//                td_global_remote_yaw.aim = location.fpPosQ;
//                CalTD(&td_global_remote_yaw);
//                PID_Calc_Angle_withoutDiff(&pid_remote_yaw, global_remote_yaw, location.fpPosQ, td_global_remote_yaw.x2);
//                Chassis.local_vel.fpW = pid_remote_yaw.fpU;
//            }
//            SteerFixed(&Chassis.wheels, vel_dir, vel, Chassis.local_vel.fpW);
//        }
//        else
//        {
//            global_remote_flag = 1;
//            chassis_remote_yaw_clear();
//            SteerWheels_distribute(&Chassis.wheels, &Chassis.local_vel);
//        }
        global_remote_yaw += Remotevel.fpW * 0.001f;
        PID_Calc_Angle(&pid_remote_yaw, global_remote_yaw, location.fpPosQ);
        GlobalVel_To_Local(&Chassis.local_vel, &Remotevel, location.fpPosQ);
        Chassis.local_vel.fpW = pid_remote_yaw.fpU + Chassis.local_vel.fpW * global_remote_yaw_ff_k;
        SteerWheels_distribute(&Chassis.wheels, &Chassis.local_vel);
        chassis_pid_calc();
        break;
    case CHASSIS_LOCK:
        SteerLock(&Chassis.wheels, 45, -45, -45, 45);
        chassis_pid_calc();
        break;
    case CHASSIS_NAV:
        GlobalVel_To_Local(&Chassis.local_vel, &Nav.global_vel, location.fpPosQ);
        SteerWheels_distribute(&Chassis.wheels, &Chassis.local_vel);
        chassis_pid_calc();
        break;
    case CHASSIS_FIXED: // 导航结束专用
		if(Chassis.cnt < Chassis.fixed_cnt){
			fixed_vel = Chassis.fixed_vel; 
            if(Chassis.cnt > 900 && Nav.mac_flag == 1)
                data_to_upper.mac_flag = 2;
		}
        else
        {
            fixed_vel = 0;
			Nav.progress = 2;
        }
        SteerFixed(&Chassis.wheels, Chassis.fixed_dir, fixed_vel, 0);
        chassis_pid_calc();
        Chassis.cnt++;
        break;
    case CHASSIS_TEST:
        chassis_pid_calc();
        break;
    default:
        break;
    }
}

void Chassis_Change(CHASSIS_STATE newstate)
{
    if (Chassis.state == newstate)
        return;
    Chassis.state = newstate;
    switch (Chassis.state)
    {
		case CHASSIS_INIT:
		    if(Chassis.err != CHASSIS_INIT_FAIL)
						Chassis.err = NO_ERROR;
				break;
    case CHASSIS_LOCAL_REMOTE:
        Path_Reset();
        chassis_clear();
        Nav.cur_spot = 0;
        break;
    case CHASSIS_GLOBAL_REMOTE:
        Path_Reset();
        chassis_clear();
        chassis_remote_yaw_clear();
        Nav.cur_spot = 0;
        break;
    case CHASSIS_LOCK:
        chassis_clear();
        break;
    case CHASSIS_NAV:
        chassis_clear();
        break;
    case CHASSIS_FIXED:
        Chassis.cnt = 0;
        break;
    default:
        break;
    }
}

void chassis_clear(void)
{
    Chassis.wheels.leftdown.driver_output = 1;
    Chassis.wheels.rightdown.driver_output = 1;
    Chassis.wheels.leftup.driver_output = 1;
    Chassis.wheels.rightup.driver_output = 1;
    Chassis.wheels.leftdown.steer_output = 0;
    Chassis.wheels.rightdown.steer_output = 0;
    Chassis.wheels.leftup.steer_output = 0;
    Chassis.wheels.rightup.steer_output = 0;
    pid_leftdown_driver.fpSumE = 0;
    pid_rightdown_driver.fpSumE = 0;
    pid_leftup_driver.fpSumE = 0;
    pid_rightup_driver.fpSumE = 0;
    pid_leftdown_steer.outer.fpSumE = 0;
    pid_rightdown_steer.outer.fpSumE = 0;
    pid_leftup_steer.outer.fpSumE = 0;
    pid_rightup_steer.outer.fpSumE = 0;
    pid_leftdown_steer.inner.fpSumE = 0;
    pid_rightdown_steer.inner.fpSumE = 0;
    pid_leftup_steer.inner.fpSumE = 0;
    pid_rightup_steer.inner.fpSumE = 0;
    pid_leftdown_driver.fpPreE = 0;
    pid_rightdown_driver.fpPreE = 0;
    pid_leftup_driver.fpPreE = 0;
    pid_rightup_driver.fpPreE = 0;
    pid_leftdown_steer.outer.fpPreE = 0;
    pid_rightdown_steer.outer.fpPreE = 0;
    pid_leftup_steer.outer.fpPreE = 0;
    pid_rightup_steer.outer.fpPreE = 0;
    pid_leftdown_steer.inner.fpPreE = 0;
    pid_rightdown_steer.inner.fpPreE = 0;
    pid_leftup_steer.inner.fpPreE = 0;
    pid_rightup_steer.inner.fpPreE = 0;
}

void chassis_remote_yaw_clear(void)
{
    global_remote_yaw = location.fpPosQ;
    pid_remote_yaw.fpSumE = 0;
    pid_remote_yaw.fpPreE = 0;
    pid_remote_yaw.fpU = 0;
}

uint32_t err_output_cnt = 0;
ST_VEL pre_remotevel;
uint32_t err_remote_cnt = 0;
uint32_t err_location_cnt = 0;
uint32_t err_nav_cnt = 0;
uint32_t err_vel_cnt = 0;
void chassis_protection(void)
{
	
	    // 保护
    if (Chassis.err != NO_ERROR)
    {
        Chassis_Change(CHASSIS_STANDBY);
    }
		else {//防止错误信息被覆盖
			
    // 过流保护
    uint8_t protect = 0;
    protect |= fabsf(Chassis.wheels.leftdown.driver_output) > 19999;
    protect |= fabsf(Chassis.wheels.rightdown.driver_output) > 19999;
    protect |= fabsf(Chassis.wheels.leftup.driver_output) > 19999;
    protect |= fabsf(Chassis.wheels.rightup.driver_output) > 19999;
    //    protect |= fabsf(Chassis.wheels.rightup.steer_output) > 8500;
    //    protect |= fabsf(Chassis.wheels.leftup.steer_output) > 8500;
    //    protect |= fabsf(Chassis.wheels.rightdown.steer_output) > 8500;
    //    protect |= fabsf(Chassis.wheels.leftdown.steer_output) > 8500;
    if (protect)
    {
        err_output_cnt++;
        if (err_output_cnt > 1000)
        {
            Chassis.err = CHASSIS_OUTPUT_OVERSIZE;
        }
    }
    else
    {
        err_output_cnt = 0;
    }

    // 过速保护
    protect = 0;
    protect |= fabsf(Chassis.wheels.leftdown.driver.speed) > 158;
    protect |= fabsf(Chassis.wheels.leftup.driver.speed) > 158;
    protect |= fabsf(Chassis.wheels.rightdown.driver.speed) > 158;
    protect |= fabsf(Chassis.wheels.rightup.driver.speed) > 158;
    if (protect)
    {
				err_vel_cnt++;
				if(err_vel_cnt > 50)
						Chassis.err = CHASSIS_VELOCITY_OVERSIZE;
    }
		else{
				err_vel_cnt = 0;
		}

    // 初始化失败
    if (Chassis.state == CHASSIS_INIT)
    {
        protect = 0;
        protect |= fabsf(pid_leftdown_steer.outer_des - pid_leftdown_steer.outer_fb) > 90;
        protect |= fabsf(pid_leftup_steer.outer_des - pid_leftup_steer.outer_fb) > 90;
        protect |= fabsf(pid_rightdown_steer.outer_des - pid_rightdown_steer.outer_fb) > 90;
        protect |= fabsf(pid_rightup_steer.outer_des - pid_rightup_steer.outer_fb) > 90;
        if (protect)
            Chassis.err = CHASSIS_INIT_FAIL;
    }
    else if (Chassis.state != CHASSIS_STANDBY)
    {
        if (Chassis.init_flag == 0)
            Chassis.err = CHASSIS_INIT_DISCPLT;
    }

    // 导航卡死, 位置异常, 雷达断联
    if (Chassis.state == CHASSIS_NAV)
    {
        protect = 0;
        if (Nav.state == NAV_PATH && Path_Points.point_pos > 0)
        {
            PATH_WAYPOINT *cur_point = &Path_Points.point[Path_Points.point_pos];
            PATH_WAYPOINT *last_point = &Path_Points.point[Path_Points.point_pos - 1];
            if (cur_point->time > 1e-3f)
            {
                if (hypotf(cur_point->pos.fpPosX - last_point->pos.fpPosX, cur_point->pos.fpPosY - last_point->pos.fpPosY) / cur_point->time > 4500)
                    Chassis.err = NAV_STUCK;
            }
        }

        if (hypotf(pid_x.fpE, pid_y.fpE) > 2000)
        {
            err_nav_cnt++;
            if (err_nav_cnt > 50)
            {
                Chassis.err = NAV_STUCK;
            }
        }
        else
        {
            err_nav_cnt = 0;
        }

        protect = 0;
        protect |= location.fpPosX < -100;
        protect |= location.fpPosX > 6100;
        protect |= location.fpPosY < -100;
        protect |= location.fpPosY > 12200;
        protect |= fabsf(location.fpPosX - Vision.pos.fpPosX) > 1000;
        protect |= fabsf(location.fpPosY - Vision.pos.fpPosY) > 1000;
        if (protect)
        {
            err_location_cnt++;
            if (err_location_cnt > 50)
            {
                Chassis.err = LOCATION_EXCEPTION;
            }
        }
        else
        {
            err_location_cnt = 0;
        }

        if (sys_mnt.fps.radar_rx < 10)
            Chassis.err = RADAR_LOSS;
				
				if(g_stuck_cnt > 10)
						Chassis.err = GYRO_STUCK;
    }

    // 遥控异常
    if (Chassis.state == CHASSIS_LOCAL_REMOTE || Chassis.state == CHASSIS_GLOBAL_REMOTE)
    {
        protect = 0;
        protect |= fabsf(Remotevel.fpVx) > 5000;
        protect |= fabsf(Remotevel.fpVy) > 5000;
        protect |= fabsf(Remotevel.fpW) > 200;
        protect |= fabsf(Remotevel.fpVx) > 200 && Remotevel.fpVx == pre_remotevel.fpVx;
        protect |= fabsf(Remotevel.fpVy) > 200 && Remotevel.fpVy == pre_remotevel.fpVy;
        protect |= fabsf(Remotevel.fpW) > 10 && Remotevel.fpW == pre_remotevel.fpW;
        if (protect)
        {
            err_remote_cnt++;
//            if (err_remote_cnt > 500)
//                Chassis.err = REMOTE_EXCEPTION;
        }
        else
            err_remote_cnt = 0;
    }
    pre_remotevel = Remotevel;
	}
}

uint16_t LED_phase = 0;

// num是闪烁的次数
void chassis_LED(Error_t err_type)
{
    LED_phase++;
    uint16_t err_num = (uint16_t)err_type;
    if (LED_phase > err_num * 400 + 1000)
        LED_phase = 0;
    else if (LED_phase >= err_num * 400)
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
    else if (LED_phase % 400 < 200)
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
    else if (LED_phase % 400 >= 200)
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
}

uint32_t init_angle = 0;

void chassis_init(void)
{
    init_angle++;
    float des_angle = (float)init_angle / 20.f;
    if (QD.ts_leftup || Chassis.wheels.leftup.steer_init_flag)
    {
        if (!Chassis.wheels.leftup.steer_init_flag)
            Chassis.wheels.leftup.steer_init_angle = Chassis.wheels.leftup.steer.angle - 90; // ��init_angleΪ��ǰ����λ��
        Chassis.wheels.leftup.steer_init_flag = 1;
        Chassis.wheels.leftup.steer_output = 0;
    }
    else
    {
        PID_Cascade_Calc(&pid_leftup_steer, des_angle, Chassis.wheels.leftup.steer.angle, Chassis.wheels.leftup.steer.speed);
        Chassis.wheels.leftup.steer_output = pid_leftup_steer.output;
    }
    if (QD.ts_rightdown || Chassis.wheels.rightdown.steer_init_flag)
    {
        if (!Chassis.wheels.rightdown.steer_init_flag)
            Chassis.wheels.rightdown.steer_init_angle = Chassis.wheels.rightdown.steer.angle + 90;
        Chassis.wheels.rightdown.steer_init_flag = 1;
        Chassis.wheels.rightdown.steer_output = 0;
    }
    else
    {
        PID_Cascade_Calc(&pid_rightdown_steer, des_angle, Chassis.wheels.rightdown.steer.angle, Chassis.wheels.rightdown.steer.speed);
        Chassis.wheels.rightdown.steer_output = pid_rightdown_steer.output;
    }
    if (QD.ts_leftdown || Chassis.wheels.leftdown.steer_init_flag)
    {
        if (!Chassis.wheels.leftdown.steer_init_flag)
            Chassis.wheels.leftdown.steer_init_angle = Chassis.wheels.leftdown.steer.angle - 90;
        Chassis.wheels.leftdown.steer_init_flag = 1;
        Chassis.wheels.leftdown.steer_output = 0;
    }
    else
    {
        PID_Cascade_Calc(&pid_leftdown_steer, des_angle, Chassis.wheels.leftdown.steer.angle, Chassis.wheels.leftdown.steer.speed);
        Chassis.wheels.leftdown.steer_output = pid_leftdown_steer.output;
    }
    if (QD.ts_rightup || Chassis.wheels.rightup.steer_init_flag)
    {
        if (!Chassis.wheels.rightup.steer_init_flag)
            Chassis.wheels.rightup.steer_init_angle = Chassis.wheels.rightup.steer.angle + 90;
        Chassis.wheels.rightup.steer_init_flag = 1;
        Chassis.wheels.rightup.steer_output = 0;
    }
    else
    {
        PID_Cascade_Calc(&pid_rightup_steer, des_angle, Chassis.wheels.rightup.steer.angle, Chassis.wheels.rightup.steer.speed);
        Chassis.wheels.rightup.steer_output = pid_rightup_steer.output;
    }
    if (Chassis.wheels.leftup.steer_init_flag && Chassis.wheels.rightup.steer_init_flag && Chassis.wheels.leftdown.steer_init_flag && Chassis.wheels.rightdown.steer_init_flag)
    {
        Chassis.init_flag = 1;
        Chassis_Change(CHASSIS_STANDBY);
    }
}

uint8_t ff_flag;

float test_kp1 = 300;
float test_kp2 = 520;
float ff_v_k1 = 25;
float ff_v_k2 = 20;
float ff_w_k1 = 90;
float ff_w_k2 = 25;

float min_pid_Des = 0.1f;
float min_pid_E = 1.f;

void chassis_pid_calc(void)
{
    intg_clear_with_brake(min_pid_Des, min_pid_E);

    PID_Cascade_Calc(&pid_leftdown_steer, Chassis.wheels.leftdown.pos, Chassis.wheels.leftdown.steer.angle - Chassis.wheels.leftdown.steer_init_angle, Chassis.wheels.leftdown.steer.speed);
    PID_Cascade_Calc(&pid_rightdown_steer, Chassis.wheels.rightdown.pos, Chassis.wheels.rightdown.steer.angle - Chassis.wheels.rightdown.steer_init_angle, Chassis.wheels.rightdown.steer.speed);
    PID_Cascade_Calc(&pid_leftup_steer, Chassis.wheels.leftup.pos, Chassis.wheels.leftup.steer.angle - Chassis.wheels.leftup.steer_init_angle, Chassis.wheels.leftup.steer.speed);
    PID_Cascade_Calc(&pid_rightup_steer, Chassis.wheels.rightup.pos, Chassis.wheels.rightup.steer.angle - Chassis.wheels.rightup.steer_init_angle, Chassis.wheels.rightup.steer.speed);

    float ld_cos = cosf(pid_leftdown_steer.outer.fpE * DEG_TO_RAD);
    float lu_cos = cosf(pid_leftup_steer.outer.fpE * DEG_TO_RAD);
    float rd_cos = cosf(pid_rightdown_steer.outer.fpE * DEG_TO_RAD);
    float ru_cos = cosf(pid_rightup_steer.outer.fpE * DEG_TO_RAD);

    PID_Calc(&pid_leftdown_driver, Chassis.wheels.leftdown.vel * ld_cos, Chassis.wheels.leftdown.driver.speed);
    PID_Calc(&pid_rightdown_driver, Chassis.wheels.rightdown.vel * rd_cos, Chassis.wheels.rightdown.driver.speed);
    PID_Calc(&pid_leftup_driver, Chassis.wheels.leftup.vel * lu_cos, Chassis.wheels.leftup.driver.speed);
    PID_Calc(&pid_rightup_driver, Chassis.wheels.rightup.vel * ru_cos, Chassis.wheels.rightup.driver.speed);

	pid_change_with_vel(&pid_leftdown_driver, test_kp1, test_kp2);
	pid_change_with_vel(&pid_leftup_driver, test_kp1, test_kp2);
	pid_change_with_vel(&pid_rightdown_driver, test_kp1, test_kp2);
	pid_change_with_vel(&pid_rightup_driver, test_kp1, test_kp2);
	SteerWheels_FfCalc(&Chassis.wheels, &Chassis.local_vel, ff_v_k1, ff_v_k2, ff_w_k1, ff_w_k2);
    Chassis.wheels.leftdown.steer_output = pid_leftdown_steer.output;
    Chassis.wheels.rightdown.steer_output = pid_rightdown_steer.output;
    Chassis.wheels.leftup.steer_output = pid_leftup_steer.output;
    Chassis.wheels.rightup.steer_output = pid_rightup_steer.output;
    Chassis.wheels.leftdown.driver_output = clipfloat(pid_leftdown_driver.fpU + Chassis.wheels.leftdown.ff, -20000, 20000);
    Chassis.wheels.rightdown.driver_output = clipfloat(pid_rightdown_driver.fpU + Chassis.wheels.rightdown.ff, -20000, 20000);
    Chassis.wheels.leftup.driver_output = clipfloat(pid_leftup_driver.fpU + Chassis.wheels.leftup.ff, -20000, 20000);
    Chassis.wheels.rightup.driver_output = clipfloat(pid_rightup_driver.fpU + Chassis.wheels.rightdown.ff, -20000, 20000);
}

float lock_ff0 = 1150;

float test_ki = 0.45f;
float test_ff_k2 = 52;

void para_change_withNav(void)
{
    if (Nav.state == NAV_LOCK)
    {
        ff_leftup_driver.k2 = 0;
        ff_leftdown_driver.k2 = 0;
        ff_rightup_driver.k2 = 0;
        ff_rightdown_driver.k2 = 0;

        pid_leftdown_driver.fpSumEMax = 1000;
        pid_leftdown_driver.fpKi = 0.f;
        pid_leftup_driver.fpSumEMax = 1000;
        pid_leftup_driver.fpKi = 0.f;
        pid_rightdown_driver.fpSumEMax = 1000;
        pid_rightdown_driver.fpKi = 0.f;
        pid_rightup_driver.fpSumEMax = 1000;
        pid_rightup_driver.fpKi = 0.f;
    }
    else if (Nav.state == NAV_PATH)
    {
        ff_leftup_driver.k0 = 0;
        ff_leftdown_driver.k0 = 0;
        ff_rightup_driver.k0 = 0;
        ff_rightdown_driver.k0 = 0;

        ff_leftup_driver.k2 = test_ff_k2;
        ff_leftdown_driver.k2 = test_ff_k2;
        ff_rightup_driver.k2 = test_ff_k2;
        ff_rightdown_driver.k2 = test_ff_k2;

        pid_leftdown_driver.fpSumEMax = 1000;
        pid_leftdown_driver.fpKi = 0.8f;
        pid_leftup_driver.fpSumEMax = 1000;
        pid_leftup_driver.fpKi = 0.8f;
        pid_rightdown_driver.fpSumEMax = 1000;
        pid_rightdown_driver.fpKi = 0.8f;
        pid_rightup_driver.fpSumEMax = 1000;
        pid_rightup_driver.fpKi = 0.8f;
    }
    else
    {
        ff_leftup_driver.k0 = 0;
        ff_leftdown_driver.k0 = 0;
        ff_rightup_driver.k0 = 0;
        ff_rightdown_driver.k0 = 0;

        ff_leftup_driver.k2 = 35;
        ff_leftdown_driver.k2 = 35;
        ff_rightup_driver.k2 = 35;
        ff_rightdown_driver.k2 = 35;

    }
		
		

}

float v_start = 4.8;
float v_end = 20;
void pid_change_with_vel(ST_PID *pid, float kp1, float kp2){
		float v_target = fmaxf(fabsf(pid->fpDes), fabsf(pid->fpFB));
    float x = (fabsf(pid->fpDes) - v_start)/(v_end - v_start);
    pid->fpKp = sin_interp_fast(kp1, kp2, x);
}

void intg_clear_with_brake(float min_des, float min_e){
    if(fabsf(pid_leftup_driver.fpDes) < min_des && fabsf(pid_leftup_driver.fpE) < min_e){
        pid_leftup_driver.fpSumE = 0;
    }
    if(fabsf(pid_leftdown_driver.fpDes) < min_des  && fabsf(pid_leftdown_driver.fpE) < min_e){
        pid_leftdown_driver.fpSumE = 0;
    }
    if(fabsf(pid_rightup_driver.fpDes) < min_des && fabsf(pid_rightup_driver.fpE) < min_e){
        pid_rightup_driver.fpSumE = 0;
    }
    if(fabsf(pid_rightdown_driver.fpDes) < min_des && fabsf(pid_rightdown_driver.fpE) < min_e){
        pid_rightdown_driver.fpSumE = 0;
    }
}

