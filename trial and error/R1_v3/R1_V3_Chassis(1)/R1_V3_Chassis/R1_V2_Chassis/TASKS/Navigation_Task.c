#include "Navigation_Task.h"

void navigation(void)
{
	//	catch_continue();
	switch (nav.nav_state)
	{
	case NAV_INIT: // 0
		chassis_run.pid_state = VELT_LOOP;
		chassis_run.feed_forward_state = WITHOUT_FORWARD;

		//		chassis_run.rightup.fpDes = 0;
		//		chassis_run.leftup.fpDes = 0;
		//		chassis_run.leftdown.fpDes = 0;
		//		chassis_run.rightdown.fpDes = 0;

		break;

	case NAV_OFF: // 1
		chassis_run.pid_state = OPEN_LOOP;
		chassis_run.feed_forward_state = WITHOUT_FORWARD;

		chassis_run.rightup.fpU = 0;
		chassis_run.leftup.fpU = 0;
		chassis_run.leftdown.fpU = 0;
		chassis_run.rightdown.fpU = 0;
		break;

	case NAV_LOCK: // 2
		if (flag_lock == 1)
		{
			nav.auto_path.pos_pid.x.fpDes = nav.auto_path.pos_pid.x.fpFB;
			nav.auto_path.pos_pid.y.fpDes = nav.auto_path.pos_pid.y.fpFB;
			flag_lock = 0;
		}

		CalculateVelocities(&Js_Value, &nav, 500, 0, 500, 0, 500, 45); // 鎽囨潌涓嶆帶X,Y绉诲姩

		Nav_PID_Adjust();

		PID_Calc_New(&nav.auto_path.pos_pid.x);
		PID_Calc_New(&nav.auto_path.pos_pid.y);

		nav.expect_robot_global_velt.fpX = (float)0.3 * nav.auto_path.pos_pid.x.fpU;
		nav.expect_robot_global_velt.fpY = (float)0.3 * nav.auto_path.pos_pid.y.fpU;
		nav.expect_robot_global_velt.fpW = nav.auto_path.basic_velt.fpW;

		SpeedDistribute_Four_OmnidriectionalWhile(&nav);
		break;

	case NAV_GLOBAL_MANUAL: // 3
		chassis_run.pid_state = VELT_LOOP;
		chassis_run.feed_forward_state = WITHOUT_FORWARD;

		CalculateVelocities(&Js_Value, &nav, 300, 3500, 300, 3500, 500, 150);
		// 这里算出来单位都是mm，这里手柄分配最大速度是400mm/s，转速30°/s   //估计主要做微调用

		ramp_signal(&nav.expect_robot_global_velt.fpX, nav.auto_path.basic_velt.fpVx, LineAccelStep);
		ramp_signal(&nav.expect_robot_global_velt.fpY, nav.auto_path.basic_velt.fpVy, LineAccelStep);

		if (flag_global_manual == 1)
		{
			nav.auto_path.pos_pid.w.fpDes = fpSumPosQ / 10; // 记录此时的车身旋转累计角度值（累计角度值就可以无限转了）

			//			nav.auto_path.pos_pid.w.fpDes = stRobot.stPos.fpPosQ / 10.f;
			flag_global_manual = 0;
		}

		if (fabs(nav.auto_path.basic_velt.fpW) < 10)
		{
			nav.auto_path.basic_velt.fpW = 0;
		}
		if (fabs(nav.auto_path.basic_velt.fpW) >= 10)
		{
			nav.auto_path.pos_pid.w.fpDes = fpSumPosQ / 10;
			//						nav.auto_path.pos_pid.w.fpDes = stRobot.stPos.fpPosQ / 10.f;
		}

		nav.auto_path.pos_pid.w.fpFB = fpSumPosQ / 10;

		Nav_PID_Adjust();

		PID_Calc_New(&nav.auto_path.pos_pid.w);

		if (fabs(nav.auto_path.basic_velt.fpW) < 10)
		{
			nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU /*+ nav.auto_path.basic_velt.fpW*/;
		}
		else if (fabs(nav.auto_path.basic_velt.fpW) >= 10)
		{
			nav.expect_robot_global_velt.fpW = nav.auto_path.basic_velt.fpW;
		}

		nav.expect_robot_global_velt.fpX = nav.auto_path.basic_velt.fpVx;
		nav.expect_robot_global_velt.fpY = nav.auto_path.basic_velt.fpVy;

		SpeedDistribute_Four_OmnidriectionalWhile(&nav);
		break;

	case NAV_AREA_1: // 一区   4
		Nav_Area_1_Task();

		break;

	case NAV_AREA_2: // 5
		chassis_run.pid_state = VELT_LOOP;
		chassis_run.feed_forward_state = WITHOUT_FORWARD;

		Nav_Area_2_Task(&nav);

		Nav_PID_Adjust();

		PID_Calc_New(&nav.auto_path.pos_pid.x);
		PID_Calc_New(&nav.auto_path.pos_pid.y);
		PID_Calc_New(&nav.auto_path.pos_pid.w);

		//		nav.expect_robot_global_velt.fpX = (float)0.5*nav.auto_path.pos_pid.x.fpU + (float)0.5*nav.auto_path.basic_velt.fpVx;
		//		nav.expect_robot_global_velt.fpY = (float)0.3*nav.auto_path.pos_pid.y.fpU + (float)0.7*nav.auto_path.basic_velt.fpVy;
		//	  nav.expect_robot_global_velt.fpW = (float)0.5*nav.auto_path.pos_pid.w.fpU + (float)0.5*nav.auto_path.basic_velt.fpW;
		nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.x.fpU + nav.auto_path.basic_velt.fpVx;
		nav.expect_robot_global_velt.fpY = nav.auto_path.pos_pid.y.fpU + nav.auto_path.basic_velt.fpVy;
		nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU + nav.auto_path.basic_velt.fpW;

		SpeedDistribute_Four_OmnidriectionalWhile(&nav);
		break;

	case NAV_AREA_3: // 6
		Nav_Area_3_Task();
		break;

	case NAV_AREA_3_RESET: // 7
		Nav_Area_3_Reset_Task();
		break;
	case NAV_MANUAL: // 3

		//
		//		chassis_run.pid_state = VELT_LOOP;
		//		chassis_run.feed_forward_state = WITHOUT_FORWARD;

		//		CalculateVelocities(&Js_Value, &nav, 500, 1200, 500, 1200, 500, 90);
		//  	//这里算出来单位都是mm，这里手柄分配最大速度是400mm/s，转速30°/s   //估计主要做微调用

		//		ramp_signal(&nav.expect_robot_global_velt.fpX, nav.auto_path.basic_velt.fpVx, LineAccelStep);
		//		ramp_signal(&nav.expect_robot_global_velt.fpY, nav.auto_path.basic_velt.fpVy, LineAccelStep);

		//
		//		if (flag_global_manual == 1)
		//		{
		//		nav.auto_path.pos_pid.w.fpDes = fpSumPosQ / 10; //记录此时的车身旋转累计角度值（累计角度值就可以无限转了）
		//
		////			nav.auto_path.pos_pid.w.fpDes = stRobot.stPos.fpPosQ / 10.f;
		//			flag_global_manual = 0;
		//		}
		//
		//
		//		if (fabs(nav.auto_path.basic_velt.fpW) < 10)
		//		{
		//			nav.auto_path.basic_velt.fpW = 0;
		//		}
		//		if (fabs(nav.auto_path.basic_velt.fpW) >= 10)
		//		{
		//			nav.auto_path.pos_pid.w.fpDes = fpSumPosQ / 10;
		////						nav.auto_path.pos_pid.w.fpDes = stRobot.stPos.fpPosQ / 10.f;
		//		}
		//
		//
		//
		//
		//		nav.auto_path.pos_pid.w.fpFB = fpSumPosQ / 10;
		//
		//		Nav_PID_Adjust();
		//
		//		PID_Calc_New(&nav.auto_path.pos_pid.w);
		//
		//
		//		if (fabs(nav.auto_path.basic_velt.fpW) < 10)
		//		{
		//			nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU /*+ nav.auto_path.basic_velt.fpW*/;
		//		}
		//		else if (fabs(nav.auto_path.basic_velt.fpW) >= 10)
		//		{
		//			nav.expect_robot_global_velt.fpW = nav.auto_path.basic_velt.fpW;
		//		}
		//
		nav.expect_robot_global_velt.fpX = nav.auto_path.basic_velt.fpVx;
		nav.expect_robot_global_velt.fpY = nav.auto_path.basic_velt.fpVy;
		PID_Calc_New(&nav.auto_path.pos_pid.w);
		nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU + nav.auto_path.basic_velt.fpW;
		//		nav.expect_robot_global_velt.fpW = 0;

		SpeedDistribute_Four_OmnidriectionalWhile(&nav);
		break;

	case NAV_AREA_3_SINGLE:
		Nav_Area_3_SINGLE_Task();
		break;

	default:
		break;
	}
}
