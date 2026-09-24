#include "navigation.h"

uint16_t test_spot;
uint16_t test_spot1 = 0x2002;
uint16_t test_spot2 = 0x2010;
uint16_t test_spot3 = 0x0000;
uint8_t mac_test_flag = 0;
uint8_t noLock_mode = 0;

void Nav_Run(void)
{
    switch (Nav.state)
    {
    case NAV_STANDBY:
        nav_clear();
        Nav.global_vel = (ST_VEL){0};

        // mac相关
        if (Nav.mac_flag == 1)
        {
            if ((Chassis.state == CHASSIS_FIXED && Chassis.cnt > Chassis.fixed_cnt) ||
                (Chassis.state == CHASSIS_LOCK && data_from_upper.upper_cplt == 1) || mac_test_flag == 1)
            {
                Nav.state = NAV_PATH;
                Nav.init_flag = 0;
                mac_test_flag = 0;
                data_to_upper.mac_flag = 0;
            }
        }

        break;
    case NAV_PATH:
        if (Nav.init_flag == 0)
        {
            Nav_generatePath();
            Nav.init_flag = 1;
        }
        Path_Spline(&Nav.object);
        nav_pid_calc();
        Nav.progress = Path_Points.progress;
        if (Path_isEnd())
        {
            if (Nav.cur_spot == 0x2050)
            { // 特殊情况
                Nav.state = NAV_PATH;
                Nav.init_flag = 0;
            }
            else
            {
                Nav.state = NAV_LOCK;
                nav_clear();
                pid_leftdown_driver.fpSumE = 0;
                pid_leftup_driver.fpSumE = 0;
                pid_rightup_driver.fpSumE = 0;
                pid_rightdown_driver.fpSumE = 0;
            }

            Nav.progress = 1;
            if (Nav.mac_flag == 1)
                data_to_upper.mac_flag = 1;

            Nav.object.pos = Nav.final_pos;
            Nav.object.vel = (ST_VEL){0};
            Path_Reset();
        }
        break;
    case NAV_LOCK:
				if(noLock_mode == 1){
						Nav.object.pos.fpPosX += Remotevel.fpVx * 0.001f;
						Nav.object.pos.fpPosY += Remotevel.fpVy * 0.001f;
						Nav.object.pos.fpPosQ = norm_angle(Nav.object.pos.fpPosQ + Remotevel.fpW * 0.001f);
				}
		
        nav_pid_calc();
        if (nav_lockcheck() == 1 && noLock_mode == 0)
        {
            Nav.state = NAV_STANDBY;
            if (Nav.mac_flag == 1)
            {
                Nav.progress = 2;
                if (mac_step <= 3)
                {
                    uint8_t dir_num = 0;
                    if (mac_step >= MAC_spotnum)
                        dir_num = MAC_spotnum - 1;
                    else
                        dir_num = mac_step - 1;

                    if (mac_dir[dir_num] == 1)
                    {
                        Chassis_Change(CHASSIS_FIXED);
                        Chassis.fixed_cnt = 2000;
                        Chassis.fixed_dir = 90;
                        Chassis.fixed_vel = 300;
                    }
                    else if (mac_dir[dir_num] == -1)
                    {
                        Chassis_Change(CHASSIS_FIXED);
                        Chassis.fixed_cnt = 2000;
                        Chassis.fixed_dir = -90;
                        Chassis.fixed_vel = 300;
                    }
                    else
                    {
                        Chassis_Change(CHASSIS_LOCK);
                        data_to_upper.mac_flag = 2;
                    }
                }
                else
                {
                    Chassis_Change(CHASSIS_LOCK);
                }

                if (mac_step > 4)
                    Nav.mac_flag = 0;
            }
            else
            {
                if (Nav.cur_spot == 0x1005)
                {
                    Nav.progress = 2;
                    Chassis_Change(CHASSIS_FIXED);
                    Chassis.fixed_cnt = 1000;
                    Chassis.fixed_dir = 90;
                    Chassis.fixed_vel = 200;
                }
                else
                {
                    Nav.progress = 2;
                    Chassis_Change(CHASSIS_LOCK);
                }
            }
        };
        break;
    case NAV_TEST:
        Nav_Start(test_spot);
        break;
    case NAV_MAC_TEST:
        Nav_MAC_Start(test_spot1, test_spot2, test_spot3);
        break;
    default:
        break;
    }
}

void nav_clear(void)
{
    pid_x.fpPreE = 0;
    pid_x.fpSumE = 0;
    pid_x.fpU = 0;
    pid_y.fpPreE = 0;
    pid_y.fpSumE = 0;
    pid_y.fpU = 0;
    pid_yaw.fpPreE = 0;
    pid_yaw.fpSumE = 0;
    pid_yaw.fpU = 0;
}

void Nav_Start(uint16_t spot)
{
    if (Nav.cur_spot == spot || spot == 0 || (!Path_isEnd()))
        return;
    Nav.cur_spot = spot;
    Nav.state = NAV_PATH;
    nav_clear();
    Nav.init_flag = 0;
}

void Nav_generatePath(void)
{
    if (Nav.mac_flag == 1)
    {
        MAC_Route(data_from_upper.nav_target);
        if (mac_step > 3)
        {
            Nav.cur_spot = 0x3003;
        }
        else if (mac_step > MAC_spotnum)
        {
            Nav.cur_spot = MAC_spot[MAC_spotnum - 1];
        }
        else if (mac_step <= MAC_spotnum)
        {
            Nav.cur_spot = MAC_spot[mac_step - 1];
        }
    }
    else
    {
        route_choose(Nav.cur_spot);
    }

    Path_Init(&location, &Nav.final_pos);
    Nav.state = NAV_PATH;
    Nav.object.pos = Path_Points.point[0].pos;
    Chassis_Change(CHASSIS_NAV);
}

void Nav_MAC_Start(uint16_t spot1, uint16_t spot2, uint16_t spot3)
{
    if (Nav.mac_flag != 0 || spot1 == 0 || (!Path_isEnd()))
        return;
    Nav.mac_flag = 1; // 边走边吸刚开始的标志位
    MAC_Route_Init(spot1, spot2, spot3);
    Nav.state = NAV_PATH;
    nav_clear();
    Nav.init_flag = 0;
}

void Nav_Start_withoutLock(uint16_t spot){
    if (Nav.cur_spot == spot || spot == 0 || (!Path_isEnd()))
        return;
    Nav.cur_spot = spot;
    Nav.state = NAV_PATH;
    nav_clear();
    Nav.init_flag = 0;
}

float nav_ff_k1 = 0.9f; // 1.f;
float nav_ff_k0 = 0.f;
void nav_pid_calc(void)
{ // 计算位置pid
    nav_pid_adjust();
    if (Nav.state == NAV_PATH)
    {
        PID_Calc(&pid_x, Nav.object.pos.fpPosX, location.fpPosX);
        PID_Calc(&pid_y, Nav.object.pos.fpPosY, location.fpPosY);
        PID_Calc_Angle(&pid_yaw, Nav.object.pos.fpPosQ, location.fpPosQ);
        if (location.fpPosX > 4700 && location.fpPosY > 9300 && location.fpPosY < 10800)
        {
            Nav.global_vel.fpVx = (pid_x.fpU + Nav.object.vel.fpVx * nav_ff_k);
            Nav.global_vel.fpVy = (pid_y.fpU + Nav.object.vel.fpVy * nav_ff_k) / 0.96592f;
            Nav.global_vel.fpW = (pid_yaw.fpU + Nav.object.vel.fpW * nav_ff_k);
        }
        else
        {
            Nav.global_vel.fpVx = pid_x.fpU + Nav.object.vel.fpVx * nav_ff_k;
            Nav.global_vel.fpVy = pid_y.fpU + Nav.object.vel.fpVy * nav_ff_k;
            Nav.global_vel.fpW = pid_yaw.fpU + Nav.object.vel.fpW * nav_ff_k;
        }
    }
    else if (Nav.state == NAV_LOCK)
    {
        PID_Calc(&pid_x, Nav.object.pos.fpPosX, location.fpPosX);
        PID_Calc(&pid_y, Nav.object.pos.fpPosY, location.fpPosY);
        PID_Calc_Angle(&pid_yaw, Nav.object.pos.fpPosQ, location.fpPosQ);
        float vel = hypotf(pid_x.fpU, pid_y.fpU);
        Nav.global_vel.fpVx = pid_x.fpU + (pid_x.fpU / vel) * nav_ff_k0;
        Nav.global_vel.fpVy = pid_y.fpU + (pid_y.fpU / vel) * nav_ff_k0;
        Nav.global_vel.fpW = pid_yaw.fpU;
    }
}

uint32_t nav_lock_cnt = 0;
uint32_t nav_lock_cnt_max = 500;

uint8_t nav_lockcheck(void)
{
    //    float dist = hypotf(location.fpPosX - Nav.final_pos.fpPosX, location.fpPosY - Nav.final_pos.fpPosY);
    float dist_x = fabsf(location.fpPosX - Nav.final_pos.fpPosX);
    float dist_y = fabsf(location.fpPosY - Nav.final_pos.fpPosY); 
    float dist = hypotf(location.fpPosX - Nav.final_pos.fpPosX, location.fpPosY - Nav.final_pos.fpPosY);
    float delta = fabsf(norm_angle(location.fpPosQ - Nav.final_pos.fpPosQ));
	
		float dist_vis_x = fabsf(Vision.pos.fpPosX - Nav.final_pos.fpPosX);
		float dist_vis_y = fabsf(Vision.pos.fpPosY - Nav.final_pos.fpPosY);
		float delta_vis = fabsf(norm_angle(Vision.pos.fpPosQ - Nav.final_pos.fpPosQ));

    if (Nav.cur_spot == 0x1005 || Nav.cur_spot == 0x1006)
    {
        if (dist_x < 15.f && dist_y < 15.f && delta < 1.f)
            return 1;
    }
    else if (Nav.cur_spot >> 12 == 0x01 && (Nav.cur_spot & 0x0f) < 5)
    {
        if (dist_x < 10.f && dist_y < 10.f && delta < 0.1f)
            nav_lock_cnt++;
				else 
						nav_lock_cnt = 0;
				
								if(nav_lock_cnt > 300){
						nav_lock_cnt = 0;
						return 1;
				}
				
    }
    else if (Nav.cur_spot >> 12 == 0x02)
    {
        uint8_t row = Nav.cur_spot & 0xf0;
        uint8_t col = Nav.cur_spot & 0x0f;
        if (row == 0 || row == 5)
        {
            if (dist_vis_x < 12.f && dist_vis_y < 16.f && delta_vis < 1.f)
                nav_lock_cnt++;
						else
								nav_lock_cnt = 0;
        }
        else if (col == 0 || col == 4)
        {
            if (dist_vis_x < 16.f && dist_vis_y < 12.f && delta_vis < 1.f)
                nav_lock_cnt++;
						else 
								nav_lock_cnt = 0;
        }
        else
        {
            if (dist_vis_x < 16.f && dist_vis_y < 16.f && delta_vis < 1.f)
                nav_lock_cnt++;
						else
								nav_lock_cnt = 0;
        }
				
				if(nav_lock_cnt > nav_lock_cnt_max){
						nav_lock_cnt = 0;
						return 1;
				}
						
    }
    else
    {
        if (dist_x < 5.f && dist_y < 5.f && delta < 1.f)
            return 1;
    }
    return 0;
}

float pid_path_kp_v = 4.2f; // 13.f;
float pid_path_kp_w = 3.f;
float pid_lock_kp_v = 6.f; // 2.5f;//2.1f;
float pid_lock_kp_w = 6.f;
float t_remain_1 = 0.39f;
float t_remain_2 = 0.f;

void nav_pid_adjust(void)
{
    if (Nav.state == NAV_PATH)
    {
        float t_remain = Path_Points.t_sum + Path_Points.t_start - HAL_GetTick() / 1000.f;
        if (t_remain > t_remain_1)
        {
            pid_x.fpKp = pid_path_kp_v;
            pid_y.fpKp = pid_path_kp_v;
            pid_yaw.fpKp = pid_path_kp_w;
            nav_ff_k = nav_ff_k1;
        }
        else if (t_remain > t_remain_2)
        {
            float smooth_factor = (1 - cosf(PI * (t_remain - t_remain_2) / (t_remain_1 - t_remain_2))) / 2; // 从1到0的平滑函数
            float kp_v = pid_lock_kp_v + (pid_path_kp_v - pid_lock_kp_v) * smooth_factor;
            float kp_w = pid_lock_kp_w + (pid_path_kp_w - pid_lock_kp_w) * smooth_factor;
            pid_x.fpKp = kp_v;
            pid_y.fpKp = kp_v;
            pid_yaw.fpKp = kp_w;

            nav_ff_k = nav_ff_k1 * smooth_factor;
        }
        else
        {
            pid_x.fpKp = pid_lock_kp_v;
            pid_y.fpKp = pid_lock_kp_v;
            pid_yaw.fpKp = pid_lock_kp_w;
            nav_ff_k = 0;
        }
    }
    else if (Nav.state == NAV_LOCK)
    {
			if(noLock_mode == 0){
        pid_x.fpKp = pid_lock_kp_v;
        pid_y.fpKp = pid_lock_kp_v;
			}
			else{
        pid_x.fpKp = 6;
        pid_y.fpKp = 6;
			}
        pid_yaw.fpKp = pid_lock_kp_w;
        pid_x.fpKi = 0;
        pid_y.fpKi = 0;
        pid_yaw.fpKi = 0;
        pid_x.fpKd = 0;
        pid_y.fpKd = 0;
        pid_yaw.fpKd = 0;
        pid_x.fpSumE = 0;
        pid_y.fpSumE = 0;
        pid_yaw.fpSumE = 0;
    }
}
