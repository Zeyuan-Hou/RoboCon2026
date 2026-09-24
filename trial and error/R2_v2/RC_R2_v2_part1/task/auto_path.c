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
	nav.auto_path.run_time = 0;
	nav.auto_path.rotation_time = 0;
	nav.auto_path.run_Sumtime = 0;
	memset(&Path_Point, 0, sizeof(PATH_POINT));
	nav.auto_path.pos_pid.x.fpSumE = 0;
	nav.auto_path.pos_pid.y.fpSumE = 0;
	nav.auto_path.pos_pid.w.fpSumE = 0;
}


void path_point_choose(ST_Nav *p_nav)
{

	switch (p_nav->auto_path.number_point)
	{
	case 1:
		Path_Point.point[0].fpX = -2970.f;
		Path_Point.point[0].fpY = -590.f;
		Path_Point.point[0].fpW = 90.2f;
		Path_Point.point_num = 1;
		break;

	case 14:
		Path_Point.point[0].fpX = -2965.f;
		Path_Point.point[0].fpY = -1950.f;
		Path_Point.point[0].fpW = 270.2f;
		Path_Point.point[1].fpX = -2965.f;
		Path_Point.point[1].fpY = -2550.f;
		Path_Point.point[1].fpW = 270.2f;
		Path_Point.point_num = 2;
		break;

	case 47:
		Path_Point.point[0].fpX = -3010.f;
		Path_Point.point[0].fpY = -3330.f;
		Path_Point.point[0].fpW = 90.2f;
		Path_Point.point_num = 1;
		break;

	case 70:
		Path_Point.point[0].fpX = -3025.f;
		Path_Point.point[0].fpY = -4340.f;
		Path_Point.point[0].fpW = 270.2f;
		Path_Point.point[1].fpX = -3025.f;
		Path_Point.point[1].fpY = -4940.f;
		Path_Point.point[1].fpW = 270.2f;
		Path_Point.point_num = 2;
		break;

	case 100:
		Path_Point.point[0].fpX = -3075.f;
		Path_Point.point[0].fpY = -6155.f;
		Path_Point.point[0].fpW = 270.2f;
		Path_Point.point_num = 1;
		break;

	case 10:
		Path_Point.point[0].fpX = -3040.f;
		Path_Point.point[0].fpY = -7035.f;
		Path_Point.point[0].fpW = 271.f;
		Path_Point.point_num = 1;
		break;

	case 107:
		Path_Point.point[0].fpX = -3040.f;
		Path_Point.point[0].fpY = -5810.f;
		Path_Point.point[0].fpW = 270.2f;
		Path_Point.point_num = 1;
		break;

	case 74:
		Path_Point.point[0].fpX = -3015.f;
		Path_Point.point[0].fpY = -4770.f;
		Path_Point.point[0].fpW = 90.2f;
		Path_Point.point[1].fpX = -3015.f;
		Path_Point.point[1].fpY = -4170.f;
		Path_Point.point[1].fpW = 90.2f;
		Path_Point.point_num = 2;
		break;

	case 41:
		Path_Point.point[0].fpX = -2965.f;
		Path_Point.point[0].fpY = -3400.f;
		Path_Point.point[0].fpW = 270.2f;
		Path_Point.point_num = 1;
		break;

	case 255:
		Path_Point.point[0].fpX = -2965.f;
		Path_Point.point[0].fpY = -2270.f;
		Path_Point.point[0].fpW = 90.2f;
		Path_Point.point[1].fpX = -2965.f;
		Path_Point.point[1].fpY = -1770.f;
		Path_Point.point[1].fpW = 90.2f;
		Path_Point.point_num = 2;
		break;

	case 2:
		Path_Point.point[0].fpX = -1720.f;
		Path_Point.point[0].fpY = -890.f;
		Path_Point.point[0].fpW = 90.2f;
		Path_Point.point_num = 1;
		break;

	case 25:
		Path_Point.point[0].fpX = -1800.f;
		Path_Point.point[0].fpY = -2135.f;
		Path_Point.point[0].fpW = 90.2f;
		Path_Point.point_num = 1;
		break;

	case 58:
		Path_Point.point[0].fpX = -1815.f;
		Path_Point.point[0].fpY = -3300.f;
		Path_Point.point[0].fpW = 91.2f;
		Path_Point.point_num = 1;
		break;

	case 81:
		Path_Point.point[0].fpX = -1835.f;
		Path_Point.point[0].fpY = -4400.f;
		Path_Point.point[0].fpW = 270.4f;
		Path_Point.point[1].fpX = -1835.f;
		Path_Point.point[1].fpY = -4980.f;
		Path_Point.point[1].fpW = 270.4f;
		Path_Point.point_num = 2;
		break;

	case 110:
		Path_Point.point[0].fpX = -1850.f;
		Path_Point.point[0].fpY = -6200.f;
		Path_Point.point[0].fpW = 270.4f;
		Path_Point.point_num = 1;
		break;

	case 11:
		Path_Point.point[0].fpX = -1810.f;
		Path_Point.point[0].fpY = -7195.f;
		Path_Point.point[0].fpW = 271.4f;
		Path_Point.point_num = 1;
		break;

	case 118:
		Path_Point.point[0].fpX = -1830.f;
		Path_Point.point[0].fpY = -5800.f;
		Path_Point.point[0].fpW = 270.4f;
		Path_Point.point_num = 1;
		break;

	case 85:
		Path_Point.point[0].fpX = -1822.f;
		Path_Point.point[0].fpY = -4800.f;
		Path_Point.point[0].fpW = 90.2f;
		Path_Point.point[1].fpX = -1822.f;
		Path_Point.point[1].fpY = -4225.f;
		Path_Point.point[1].fpW = 90.2f;
		Path_Point.point_num = 2;
		break;

	case 52:
		Path_Point.point[0].fpX = -1790.f;
		Path_Point.point[0].fpY = -2995.f;
		Path_Point.point[0].fpW = 90.2f;
		Path_Point.point_num = 1;
		break;

	case 20:
		Path_Point.point[0].fpX = -1770.f;
		Path_Point.point[0].fpY = -1800.f;
		Path_Point.point[0].fpW = 90.2f;
		Path_Point.point_num = 1;
		break;

	default:
		break;
	}
}

float dis, delta, dis_x, dis_y;
void Point_to_Point(PATH_POINT *p_point)
{
	// 1、给一小段路径的目标位置赋值
	if (p_point->flag_only_position[p_point->point_inx] == 1)
	{
		nav.auto_path.pos_pid.x.fpDes = p_point->point[p_point->point_inx].fpX;
		nav.auto_path.pos_pid.y.fpDes = p_point->point[p_point->point_inx].fpY;
		nav.auto_path.pos_pid.w.fpDes = p_point->point[p_point->point_inx].fpW;
	}
	else if (p_point->flag_only_position[p_point->point_inx] == 2)
	{
		nav.auto_path.pos_pid.x.fpDes = p_point->point[p_point->point_inx].fpX;
		nav.auto_path.pos_pid.y.fpDes = nav.auto_path.pos_pid.y.fpFB;
		nav.auto_path.pos_pid.w.fpDes = nav.auto_path.pos_pid.w.fpFB; // 只走x
	}
	else if (p_point->flag_only_position[p_point->point_inx] == 3)
	{
		nav.auto_path.pos_pid.x.fpDes = nav.auto_path.pos_pid.x.fpFB;
		nav.auto_path.pos_pid.y.fpDes = p_point->point[p_point->point_inx].fpY;
		nav.auto_path.pos_pid.w.fpDes = nav.auto_path.pos_pid.w.fpFB; // 只走y
	}
	else if (p_point->flag_only_yaw[p_point->point_inx] == 1)
	{
		nav.auto_path.pos_pid.x.fpDes = nav.auto_path.pos_pid.x.fpFB;
		nav.auto_path.pos_pid.y.fpDes = nav.auto_path.pos_pid.y.fpFB;
		nav.auto_path.pos_pid.w.fpDes = p_point->point[p_point->point_inx].fpW; // 只转向
	}
	else if (!p_point->flag_only_position[p_point->point_inx] && !p_point->flag_only_yaw[p_point->point_inx])
	{
		nav.auto_path.pos_pid.x.fpDes = p_point->point[p_point->point_inx].fpX;
		nav.auto_path.pos_pid.y.fpDes = p_point->point[p_point->point_inx].fpY;
		nav.auto_path.pos_pid.w.fpDes = p_point->point[p_point->point_inx].fpW; // 走位置和转向
	}
	
	// 2、判断是否到达一小段路径的目标位置
	dis_x = fabs(nav.auto_path.pos_pid.x.fpFB - nav.auto_path.pos_pid.x.fpDes);
	dis_y = fabs(nav.auto_path.pos_pid.y.fpFB - nav.auto_path.pos_pid.y.fpDes);
	dis = sqrt(dis_x * dis_x + dis_y * dis_y);
	delta = fabs(nav.auto_path.pos_pid.w.fpFB - nav.auto_path.pos_pid.w.fpDes);

	if (dis_x < 20.f && dis_y < 10.f && delta < 1.f && !p_point->flag_point_end)
	{
		if (p_point->point_inx < p_point->point_num - 1) // 非最后一段路径，直接在NAV_POINT_TO_POINT状态下切换到下一段路径
		{
			p_point->point_inx++;
			nav.auto_path.pos_pid.x.fpSumE = 0;
			nav.auto_path.pos_pid.y.fpSumE = 0;
			nav.auto_path.pos_pid.w.fpSumE = 0;
		}
		else if (p_point->point_inx == p_point->point_num - 1) // 最后一段路径
		{
			p_point->flag_point_end = 1;
			p_point->point_tim = 0;
		}
	}

	if (p_point->flag_point_end && p_point->point_tim > 300) // 最后一段路径，给0.3s精调，然后切换到NAV_LOCK状态
	{
		flag_lock = 1;
		nav.nav_state = NAV_LOCK;
		nav.auto_path.pos_pid.x.fpSumE = 0;
		nav.auto_path.pos_pid.y.fpSumE = 0;
		nav.auto_path.pos_pid.w.fpSumE = 0;
		nav.auto_path.pos_pid.x.fpU = 0;
		nav.auto_path.pos_pid.y.fpU = 0;
		nav.auto_path.pos_pid.w.fpU = 0;
		Path_Point.flag_point_end = 0;
		Path_Point.point_inx = 0;
	}

	// PID_Fuzzy_Calc和TD_Calc互斥，不能同时使用，追求速度用PID_Fuzzy_Calc，追求平滑用TD_Calc
	PID_Fuzzy_Calc(&nav.auto_path.pos_pid.x, nav.auto_path.pos_pid.x.fpDes, nav.auto_path.pos_pid.x.fpFB);
	PID_Fuzzy_Calc(&nav.auto_path.pos_pid.y, nav.auto_path.pos_pid.y.fpDes, nav.auto_path.pos_pid.y.fpFB);
	// 可能需要转角优化
	PID_Fuzzy_Calc(&nav.auto_path.pos_pid.w, nav.auto_path.pos_pid.w.fpDes, nav.auto_path.pos_pid.w.fpFB);

	nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.x.fpU;
	nav.expect_robot_global_velt.fpY = nav.auto_path.pos_pid.y.fpU;
	nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU / 180.f * PI;
}