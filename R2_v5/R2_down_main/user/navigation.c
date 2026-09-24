#include "navigation.h"


void Cubic_Curve_Set(Cube_Line* cube, float p0, float v0, float p1, float v1, int32_t tim_total)
{
    const float v0_ms = v0 / 1000.f;
    const float v1_ms = v1 / 1000.f;
    const float t = (float)tim_total;
    cube->k0 = p0;
    cube->k1 = v0_ms;
    cube->k2 = (3.f * (p1 - p0) - (2.f * v0_ms + v1_ms) * t) / (t * t);
    cube->k3 = (-2.f * (p1 - p0) + (v0_ms + v1_ms) * t) / (t * t * t);
}

void Cubic_Curve_Calc(float* aim_p, float* aim_v, Cube_Line cube, int32_t tim, int32_t tim_total)
{
    if (tim > tim_total) tim = tim_total;
    if (tim < 0) tim = 0;
    const float t = (float)tim;
    *aim_p = cube.k3 * t * t * t + cube.k2 * t * t + cube.k1 * t + cube.k0;
    *aim_v = (3.f * cube.k3 * t * t + 2.f * cube.k2 * t + cube.k1) * 1000.f;
}

float pos_x, pos_y, pos_w, vel_x, vel_y, vel_w,vel_xpre;
void Point_to_Point(PATH_POINT *p)
{
	static uint8_t flag_W = 0;
	
	// 1、给一小段路径的目标位置赋值
	// 首次进入一小段路径，计算三次多项式系数
	if (p->flag_cube_set == 0)
	{
		Cubic_Curve_Set(&cube_x, p->point[p->point_inx].fpX, p->velt[p->point_inx].fpX, p->point[p->point_inx + 1].fpX, p->velt[p->point_inx + 1].fpX, p->time[p->point_inx]);
		Cubic_Curve_Set(&cube_y, p->point[p->point_inx].fpY, p->velt[p->point_inx].fpY, p->point[p->point_inx + 1].fpY, p->velt[p->point_inx + 1].fpY, p->time[p->point_inx]);
		// 递推使每一次转角小于等于pi
		p->point[p->point_inx + 1].fpW = p->point[p->point_inx].fpW + ConvertAngle(p->point[p->point_inx + 1].fpW - p->point[p->point_inx].fpW);
		Cubic_Curve_Set(&cube_w, p->point[p->point_inx].fpW, p->velt[p->point_inx].fpW, p->point[p->point_inx + 1].fpW, p->velt[p->point_inx + 1].fpW, p->time[p->point_inx]);
		//加一个判断，如果一段路径W几乎不需要变，那么调大pid_w锁YAW，否则调小防止超调和震荡
		if(fabs(p->point[p->point_inx].fpW-p->point[p->point_inx + 1].fpW)<=0.5){flag_W = 0;}else{flag_W = 1;}
		p->point_tim = 0;
		p->flag_cube_set = 1;
	}
	// 由三次多项式计算实时目标位置和速度
	
	vel_xpre=vel_x;// 只有X方向，如果雷达坐标变了或者加其他方向记得改 
	
	Cubic_Curve_Calc(&pos_x, &vel_x, cube_x, p->point_tim, p->time[p->point_inx]);
	Cubic_Curve_Calc(&pos_y, &vel_y, cube_y, p->point_tim, p->time[p->point_inx]);
	Cubic_Curve_Calc(&pos_w, &vel_w, cube_w, p->point_tim, p->time[p->point_inx]);
	
	if(fabs(vel_xpre)-fabs(vel_x)>2)//
	{
			test_forward=0;
	}
	else 
	{
			test_forward=1;
	}
	
	nav.auto_path.pos_pid.pid_x.fpDes = pos_x;
	nav.auto_path.pos_pid.pid_y.fpDes = pos_y;
	nav.auto_path.pos_pid.pid_w.fpDes = pos_w;
	
	// 2、判断是否结束路径
	// 非最后一小段路径， 仅用时间判断是否结束，保障每一段路径导航的连贯性
	if (p->point_inx < p->point_num - 1)
	{
		if (p->point_tim > p->time[p->point_inx] )
		{
			p->point_inx++;
			p->flag_cube_set = 0; 
			nav.auto_path.pos_pid.pid_x.fpSumE = 0;
			nav.auto_path.pos_pid.pid_y.fpSumE = 0;
			nav.auto_path.pos_pid.pid_w.fpSumE = 0;
		}
	}
	// 最后一段路径，仅用位置反馈判断是否结束，保障组合路径导航的精确性
	else if (p->point_inx == p->point_num - 1)
	{
		float delta, dis_x, dis_y;
		dis_x = fabs(nav.auto_path.pos_pid.pid_x.fpFB - p->point[p->point_inx + 1].fpX);
		dis_y = fabs(nav.auto_path.pos_pid.pid_y.fpFB - p->point[p->point_inx + 1].fpY);
		delta = fabs(ConvertAngle(nav.auto_path.pos_pid.pid_w.fpFB - p->point[p->point_inx + 1].fpW));
		switch(nav_reach_state)
		{
			case 0://精细
				allow_nav.allow_x = 8.0f;
				allow_nav.allow_y = 8.0f;
				allow_nav.allow_rad = 0.03f;
				break;
			
			case 1://大范围容许，用于上坡,以及进入一个区域
				allow_nav.allow_x = 80.0f;
				allow_nav.allow_y = 80.0f;
				allow_nav.allow_rad = 0.1f;
				break;
			
			case 2://二区台阶归中，只对yaw角严格
				allow_nav.allow_x = 30.0f;
				allow_nav.allow_y = 30.0f;
				allow_nav.allow_rad = 0.03f;							
				break;
			
			case 3: //一区对接，不需要很严格，因为结束后会继续锁位置微调
				allow_nav.allow_x = 25.0f;
				allow_nav.allow_y = 25.0f;
				allow_nav.allow_rad = 0.07f;					
			
				break;
			case 7: //三区导航结束
				allow_nav.allow_x = 10.0f;
				allow_nav.allow_y = 10.0f;
				allow_nav.allow_rad = 0.015f;					
			
				break;
			
			case 4: // 时间判断结束，用于上坡
				if (p->point_tim > p->time[p->point_inx]||nav.auto_path.pos_pid.pid_x.fpFB<p->point[p->point_inx + 1].fpX)////记得改！！！
				{
					//p->flag_cube_set = 0;
					nav.auto_path.pos_pid.pid_x.fpSumE = 0;
					nav.auto_path.pos_pid.pid_y.fpSumE = 0;
					nav.auto_path.pos_pid.pid_w.fpSumE = 0;
					nav.auto_path.pos_pid.pid_x.fpU= 0;
					nav.auto_path.pos_pid.pid_y.fpU= 0;
					nav.auto_path.pos_pid.pid_w.fpU= 0;
					if(!flag_W){nav.expect_robot_global_velt.fpW=0;}
					nav.nav_state = NAV_RAMP;
					flag_lock=1;
				}
				break;
				
			case 5: //一区夹头基本归正就切	
				allow_nav.allow_x = 300.0f;
				allow_nav.allow_y = 300.0f;
				allow_nav.allow_rad = 0.3f;				
			break;
		}
		
				if (dis_x < allow_nav.allow_x && dis_y < allow_nav.allow_y && delta < allow_nav.allow_rad&& nav_reach_state!=4) // 单位是mm、弧度
		{
			// 给NAV_LOCK的目标位置赋值
			nav.auto_path.pos_pid.pid_x.fpDes = p->point[p->point_inx + 1].fpX;
			nav.auto_path.pos_pid.pid_y.fpDes = p->point[p->point_inx + 1].fpY;
			nav.auto_path.pos_pid.pid_w.fpDes = p->point[p->point_inx + 1].fpW;
			flag_lock = 1;
			nav.nav_state = NAV_LOCK;
		}
	}

	// 3、计算位置pid输出并赋值全局速度
	PID_Calc(&nav.auto_path.pos_pid.pid_x, nav.auto_path.pos_pid.pid_x.fpDes, nav.auto_path.pos_pid.pid_x.fpFB);
	PID_Calc(&nav.auto_path.pos_pid.pid_y, nav.auto_path.pos_pid.pid_y.fpDes, nav.auto_path.pos_pid.pid_y.fpFB);
	PID_Calc_Angle(&nav.auto_path.pos_pid.pid_w,nav.auto_path.pos_pid.pid_w.fpDes,nav.auto_path.pos_pid.pid_w.fpFB);
	
	// 全局速度 前馈系数自适应
	float K_V,K_VY,K_VW,progress;
	if(p->point_inx == p->point_num - 1){
			progress  = (float)p->point_tim / (float)p->time[p->point_inx];
			if (progress > 1.0f) progress = 1.0f;
			
			if(nav.auto_path.number_point ==28)
			{
						K_V = 0.6*(0.6f + 0.4f * cosf(PI * progress));
			}
			else if(nav.auto_path.number_point ==27)
			{
					K_V = 0.6*(0.6f + 0.4f * cosf(PI * progress));
			}
			
			else if(nav.auto_path.number_point ==24||nav.auto_path.number_point ==25||nav.auto_path.number_point ==26)
			{
					 K_V = 0.9*(0.9f - 0.1f * cosf(PI * progress));
					
			}
			else 
			{
					K_V = 0.75f + 0.25f * cosf(PI * progress);
			}
			K_VY=K_V;
			K_VW=K_V;
	}
	else if(nav.auto_path.number_point!=7&&nav.auto_path.number_point!=8&&nav.auto_path.number_point!=32&&nav.auto_path.number_point!=35)
	{
			K_V=0.9f;
			K_VY=0.8*(0.95f + 0.05f * cosf(PI * progress));
			K_VW=0.9*(0.95f + 0.05f * cosf(PI * progress));
	}
	else
	{
		K_V = 0.75f + 0.25f * cosf(PI * progress);
		K_VY=K_V;
		K_VW=K_V;
		
	}
		
	K_VEL_X = K_V;
	K_VEL_Y =	K_VY;
	K_VEL_W =	K_VW;
	
	
	if(p->point_inx == p->point_num - 1 && p->point_tim > p->time[p->point_inx]){nav.auto_path.pos_pid.pid_x.fpKp = 1.74f;nav.auto_path.pos_pid.pid_y.fpKp =1.74f;nav.auto_path.pos_pid.pid_w.fpKp = 2.5f;}
	else {nav.auto_path.pos_pid.pid_x.fpKp = 1.5f;nav.auto_path.pos_pid.pid_y.fpKp = 1.5;if(!flag_W){nav.auto_path.pos_pid.pid_w.fpKp = 3.0f;}else{nav.auto_path.pos_pid.pid_w.fpKp = 2.3f;}}

		
		
	nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.pid_x.fpU + K_VEL_X * vel_x;
	nav.expect_robot_global_velt.fpY = nav.auto_path.pos_pid.pid_y.fpU + K_VEL_Y * vel_y;
	nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.pid_w.fpU +  K_VEL_W * vel_w;


}
void SET_NAV_PATH_PERMUTATION(void) // 唯一接口，别的方式开启路径可能会由于run_time、rotation_time、P_Num、W_Num和flag_rotation不清零而出错
{
	memset(&Path_Point, 0, sizeof(PATH_POINT));
	Path_Point.flag_point_to_point = 1;
	nav.auto_path.pos_pid.pid_x.fpSumE = 0;
	nav.auto_path.pos_pid.pid_y.fpSumE = 0;
	nav.auto_path.pos_pid.pid_w.fpSumE = 0;
	
	chassis_run.wheel_1.fpSumE = 0;
	chassis_run.wheel_2.fpSumE = 0;
	chassis_run.wheel_3.fpSumE = 0;
	chassis_run.wheel_4.fpSumE = 0;
	
}


