#include "auto_path.h"

// 在chassis.c中已将此状态归为局部坐标系控制并忽略yaw，故直接给全局速度赋值
void UP_DOWN_Velt_Set(ST_Nav *pNav)
{
	switch (pNav->auto_path.up_down_state)
	{
	case UP_DOWN_STATE_OFF:
		pNav->expect_robot_global_velt.fpX = 0.f;
		pNav->expect_robot_global_velt.fpY = 0.f;
		pNav->expect_robot_global_velt.fpW = 0.f;
		chassis_run2.leftup.fpUKi = 0.f;
		chassis_run2.rightup.fpUKi = 0.f;
		chassis_run2.down.fpUKi = 0.f;
		leftup_turn_motor.pid_inner.fpSumE = 0.f;
		leftup_turn_motor.pid_outer.fpSumE = 0.f;
		rightup_turn_motor.pid_inner.fpSumE = 0.f;
		rightup_turn_motor.pid_outer.fpSumE = 0.f;
		down_turn_motor.pid_inner.fpSumE = 0.f;
		down_turn_motor.pid_outer.fpSumE = 0.f;
		pNav->auto_path.run_time_flag = 0;
		pNav->auto_path.run_time = 0;
		break;

	case UP_APPROACH_1:
		if (!foot_down_G_feedforward_flag && !foot_up_G_feedforward_flag)
		{
			pNav->expect_robot_global_velt.fpX = 0.f;
			pNav->expect_robot_global_velt.fpY = -1500.f;
			pNav->expect_robot_global_velt.fpW = 0.f;
		}
		else
		{
			pNav->expect_robot_global_velt.fpX = 0.f;
			pNav->expect_robot_global_velt.fpY = -1500.f;
			pNav->expect_robot_global_velt.fpW = 0.f;
		}
		break;

	case UP_APPROACH_2:
		pNav->expect_robot_global_velt.fpX = 0.f;
		pNav->expect_robot_global_velt.fpY = 0.f;
		pNav->expect_robot_global_velt.fpW = 0.f;
		foot_motor.ControlLoop_State = SPEED_LOOP;
		foot_motor.Input_v = 240.f;
		break;

	case UP_APPROACH_3:
		if (!foot_down_G_feedforward_flag && !foot_up_G_feedforward_flag)
		{
			pNav->expect_robot_global_velt.fpX = 0.f;
			pNav->expect_robot_global_velt.fpY = -400.f;
			pNav->expect_robot_global_velt.fpW = 0.f;
		}
		else
		{
			pNav->expect_robot_global_velt.fpX = 0.f;
			pNav->expect_robot_global_velt.fpY = -400.f;
			pNav->expect_robot_global_velt.fpW = 0.f;
		}
		break;

	case UP_APPROACH_4:
		pNav->expect_robot_global_velt.fpX = 0.f;
		pNav->expect_robot_global_velt.fpY = -1000.f;
		pNav->expect_robot_global_velt.fpW = 0.f;
		foot_motor.ControlLoop_State = SPEED_LOOP;
		foot_motor.Input_v = 55.f;
		break;

	case UP_UPSTAIRS:
		pNav->expect_robot_global_velt.fpX = 0.f;
		pNav->expect_robot_global_velt.fpY = -4000.f;
		pNav->expect_robot_global_velt.fpW = 0.f;
		foot_motor.Input_v = 240.f;
		break;

	case DOWN_APPROACH_AND_DOWNSTAIRS:
		pNav->expect_robot_global_velt.fpX = 0.f;
		pNav->expect_robot_global_velt.fpY = 900.f;
		pNav->expect_robot_global_velt.fpW = 0.f;
		break;

	default:
		break;
	}
}

void SET_NAV_PATH_PERMUTATION(void) // 唯一接口，别的方式开启路径可能会由于run_time、rotation_time、P_Num、W_Num和flag_rotation不清零而出错
{
	memset(&Path_Point, 0, sizeof(PATH_POINT));
	Path_Point.flag_point_to_point = 1;
	nav.auto_path.pos_pid.x.fpSumE = 0;
	nav.auto_path.pos_pid.y.fpSumE = 0;
	nav.auto_path.pos_pid.w.fpSumE = 0;
}


void path_point_choose(ST_Nav *p_nav)
{
	Path_Point.point[0].fpX = stRobot.stPos.fpPosX; // mm
	Path_Point.point[0].fpY = stRobot.stPos.fpPosY; // mm
	Path_Point.point[0].fpW = stRobot.stPos.fpPosQ / 10.f; // 0.1度转为度
	Path_Point.velt[0].fpX = 0;
	Path_Point.velt[0].fpY = 0;
	Path_Point.velt[0].fpW = 0;
	Path_Point.acc[0].fpX = 0;
	Path_Point.acc[0].fpY = 0;
	Path_Point.acc[0].fpW = 0;

	switch (p_nav->auto_path.number_point)
	{
	case 1:
		Path_Point.point[1].fpX = 1000.f;
		Path_Point.point[1].fpY = 0.f;
		Path_Point.point[1].fpW = 0.f;
		Path_Point.velt[1].fpX = 0;
		Path_Point.velt[1].fpY = 0;
		Path_Point.velt[1].fpW = 0;
		Path_Point.acc[1].fpX = 0;
		Path_Point.acc[1].fpY = 0;
		Path_Point.acc[1].fpW = 0;
		Path_Point.time[0] = 3000;
		Path_Point.type[0] = QUINTIC_LINE;
		Path_Point.extra_ff[0] = 1.26;

		Path_Point.point[2].fpX = 1000.f;
		Path_Point.point[2].fpY = 1000.f;
		Path_Point.point[2].fpW = 90.f;
		Path_Point.velt[2].fpX = 0;
		Path_Point.velt[2].fpY = 0;
		Path_Point.velt[2].fpW = 0;
		Path_Point.acc[2].fpX = 0;
		Path_Point.acc[2].fpY = 0;
		Path_Point.acc[2].fpW = 0;
		Path_Point.time[1] = 3000;
		Path_Point.extra_ff[1] = 0.88;
		Path_Point.type[1] = QUINTIC_LINE;
		Path_Point.point_num = 2;
		break;

	default:
		break;
	}
}

void Point_to_Point(PATH_POINT *p)
{
	// 1、给一小段路径的目标位置赋值
	// 首次进入一小段路径，计算三次多项式系数
	float pos_x, pos_y, pos_w, vel_x, vel_y, vel_w, acc_x, acc_y, acc_w;
	switch (p->type[p->point_inx])
	{
	case QUINTIC_LINE:
		if (p->flag_path_set == 0)
		{
			Quintic_Curve_Set(&quintic_x, p->point[p->point_inx].fpX, p->velt[p->point_inx].fpX, p->acc[p->point_inx].fpX, p->point[p->point_inx + 1].fpX, p->velt[p->point_inx + 1].fpX, p->acc[p->point_inx + 1].fpX, p->time[p->point_inx]);
			Quintic_Curve_Set(&quintic_y, p->point[p->point_inx].fpY, p->velt[p->point_inx].fpY, p->acc[p->point_inx].fpY, p->point[p->point_inx + 1].fpY, p->velt[p->point_inx + 1].fpY, p->acc[p->point_inx + 1].fpY, p->time[p->point_inx]);
			p->point[p->point_inx + 1].fpW = p->point[p->point_inx].fpW + Angle_Limit(p->point[p->point_inx + 1].fpW - p->point[p->point_inx].fpW);
			Quintic_Curve_Set(&quintic_w, p->point[p->point_inx].fpW, p->velt[p->point_inx].fpW, p->acc[p->point_inx].fpW, p->point[p->point_inx + 1].fpW, p->velt[p->point_inx + 1].fpW, p->acc[p->point_inx + 1].fpW, p->time[p->point_inx]);
			p->point_tim = 0;
			p->flag_path_set = 1;
		}

		Quintic_Curve_Calc(&pos_x, &vel_x, &acc_x, quintic_x, p->point_tim, p->time[p->point_inx]);
		Quintic_Curve_Calc(&pos_y, &vel_y, &acc_y, quintic_y, p->point_tim, p->time[p->point_inx]);
		Quintic_Curve_Calc(&pos_w, &vel_w, &acc_w, quintic_w, p->point_tim, p->time[p->point_inx]);
		nav.auto_path.pos_pid.x.fpDes = pos_x;
		nav.auto_path.pos_pid.y.fpDes = pos_y;
		nav.auto_path.pos_pid.w.fpDes = pos_w;
		break;

	case BEZIER:
		if (p->flag_path_set == 0)
		{
			p->point[p->point_inx + 1].fpW = p->point[p->point_inx].fpW + Angle_Limit(p->point[p->point_inx + 1].fpW - p->point[p->point_inx].fpW);
			Bezier_3rd_Set(&bezier, &p->point[p->point_inx], &p->velt[p->point_inx], &p->point[p->point_inx + 1], &p->velt[p->point_inx + 1], p->time[p->point_inx]);
			p->point_tim = 0;
			p->flag_path_set = 1;
		}

		Bezier_3rd_Calc(bezier, &pos_x, &pos_y, &pos_w, &vel_x, &vel_y, &vel_w, p->point_tim, p->time[p->point_inx]);
		acc_x = 0, acc_y = 0, acc_w = 0;
		nav.auto_path.pos_pid.x.fpDes = pos_x;
		nav.auto_path.pos_pid.y.fpDes = pos_y;
		nav.auto_path.pos_pid.w.fpDes = pos_w;
		break;

	default:
		break;
	}
	
	// 2、判断是否结束路径
	// 非最后一小段路径， 仅用时间判断是否结束，保障每一段路径导航的连贯性
	if (p->point_inx < p->point_num - 1)
	{
		// 预加载下一段路径的前馈系数
		if (p->point_tim >= p->time[p->point_inx] - 300 && p->point_tim < p->time[p->point_inx])
		{
			float t = (p->point_tim - (p->time[p->point_inx] - 300)) / 300.0f;
			path_ff.k1_x = (1 - t) * Path_Point.extra_ff[p->point_inx] + t * Path_Point.extra_ff[p->point_inx + 1];
			path_ff.k1_y = (1 - t) * Path_Point.extra_ff[p->point_inx] + t * Path_Point.extra_ff[p->point_inx + 1];
			path_ff.k1_w = (1 - t) * Path_Point.extra_ff[p->point_inx] + t * Path_Point.extra_ff[p->point_inx + 1];
		}
		// 判断是否结束当前小段路径
		if (p->point_tim >= p->time[p->point_inx])
		{
			p->point_inx++;
			p->flag_path_set = 0; 
			nav.auto_path.pos_pid.x.fpSumE = 0;
			nav.auto_path.pos_pid.y.fpSumE = 0;
			nav.auto_path.pos_pid.w.fpSumE = 0;
			path_ff.k1_x = Path_Point.extra_ff[p->point_inx]; // p->point_inx已经自增了，指向下一段路径的索引
			path_ff.k1_y = Path_Point.extra_ff[p->point_inx];
			path_ff.k1_w = Path_Point.extra_ff[p->point_inx];
		}
	}
	// 最后一段路径，仅用位置反馈判断是否结束，保障组合路径导航的精确性
	else if (p->point_inx == p->point_num - 1)
	{
		float delta, dis_x, dis_y;
		dis_x = fabs(nav.auto_path.pos_pid.x.fpFB - p->point[p->point_inx + 1].fpX);
		dis_y = fabs(nav.auto_path.pos_pid.y.fpFB - p->point[p->point_inx + 1].fpY);
		delta = fabs(Angle_Limit(p->point[p->point_inx + 1].fpW - nav.auto_path.pos_pid.w.fpFB));
		if (dis_x < 10.f && dis_y < 10.f && delta < 1.f) // 单位是mm、°
		{
			// 给NAV_LOCK的目标位置赋值
			nav.auto_path.pos_pid.x.fpDes = p->point[p->point_inx + 1].fpX;
			nav.auto_path.pos_pid.y.fpDes = p->point[p->point_inx + 1].fpY;
			nav.auto_path.pos_pid.w.fpDes = p->point[p->point_inx + 1].fpW;
			flag_lock = 1;
			nav.nav_state = NAV_LOCK;
		}
	}

	// 3、计算位置pid输出并赋值全局速度
	PID_Calc(&nav.auto_path.pos_pid.x, nav.auto_path.pos_pid.x.fpDes, nav.auto_path.pos_pid.x.fpFB);
	PID_Calc(&nav.auto_path.pos_pid.y, nav.auto_path.pos_pid.y.fpDes, nav.auto_path.pos_pid.y.fpFB);
	PID_Calc_Angle(&nav.auto_path.pos_pid.w, nav.auto_path.pos_pid.w.fpDes, nav.auto_path.pos_pid.w.fpFB);
	// 全局速度
	nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.x.fpU + path_ff.k1_x * vel_x + path_ff.k2_x * acc_x;
	nav.expect_robot_global_velt.fpY = nav.auto_path.pos_pid.y.fpU + path_ff.k1_y * vel_y + path_ff.k2_y * acc_y;
	nav.expect_robot_global_velt.fpW = (nav.auto_path.pos_pid.w.fpU + path_ff.k1_w * vel_w + path_ff.k2_w * acc_w) / 180.f * PI;
}