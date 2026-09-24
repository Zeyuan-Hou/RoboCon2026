#include "navigation_task.h"
float LineAccelStep = 10.f;
void navigation(void)
{
	switch(nav.nav_state)
	{
		case NAV_INIT:
			chassis_run.pid_state=VELT_LOOP ;
			chassis_run.feed_forward_state =WITHOUT_FORWARD;
			//PID速度期望为0
			chassis_run.rightup.fpDes = 0;
			chassis_run.leftup.fpDes = 0;
			chassis_run.leftdown.fpDes = 0;
			chassis_run.rightdown.fpDes = 0;
		
			break;
		
		case NAV_OFF:
			chassis_run.pid_state=OPEN_LOOP ;
			chassis_run.feed_forward_state =WITHOUT_FORWARD;
			//PID输出为0
			chassis_run.rightup.fpU = 0;
			chassis_run.leftup.fpU = 0;
			chassis_run.leftdown.fpU = 0;
			chassis_run.rightdown.fpU = 0;
		
			break;
		
		case NAV_LOCK:
			
			chassis_run.pid_state=VELT_LOOP ;
			chassis_run.feed_forward_state =WITHOUT_FORWARD;
			
			if(flag_lock == 1)//记录下刚按下时的坐标，后续fpDes不会再更新
			{
				nav.auto_path.pos_pid.x.fpDes =nav.auto_path.pos_pid.x.fpFB;
				nav.auto_path.pos_pid.y.fpDes =nav.auto_path.pos_pid.y.fpFB;
				nav.auto_path.pos_pid.w.fpDes =nav.auto_path.pos_pid.w.fpFB;
				
				flag_lock=0;
			}
			PID_Calc_NEW(&nav.auto_path.pos_pid.x);
  		PID_Calc_NEW(&nav.auto_path.pos_pid.y);
			PID_Calc_NEW(&nav.auto_path.pos_pid.w);
		
			nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.x.fpU ;
			nav.expect_robot_global_velt.fpY = nav.auto_path.pos_pid.y.fpU ;
			nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU ;
			
			SpeedDistribute_Four_OmnidriectionalWhile(&nav);
			
			break;
			
		case NAV_MANUAL:
			chassis_run.pid_state=VELT_LOOP ;
			chassis_run.feed_forward_state =WITHOUT_FORWARD;
			
			CalculateVelocities(&Js_Value,&nav,500,3500,500,3500,500,120);//这里算出来单位都是mm，这里手柄分配最大速度是3500mm/s，转速75°/s
			
			ramp_signal(&nav.expect_robot_global_velt.fpX,nav.auto_path.basic_velt.fpVx,LineAccelStep);
			ramp_signal(&nav.expect_robot_global_velt.fpY,nav.auto_path.basic_velt.fpVy,LineAccelStep);
		
//			Chassis_Global_Yaw_Pid.inner.fpDes = nav.auto_path.basic_velt.fpW;
//			PID_Calc_NEW(&Chassis_Global_Yaw_Pid.inner);
//			
//			nav.expect_robot_global_velt.fpW = Chassis_Global_Yaw_Pid.inner.fpU;
			nav.expect_robot_global_velt.fpW = nav.auto_path.basic_velt.fpW;
			
			SpeedDistribute_Four_OmnidriectionalWhile(&nav);//yaw轴传入的是角度
			
			break;
		
		case NAV_GLOBAL_MANUAL:
			chassis_run.pid_state=VELT_LOOP;
			chassis_run.feed_forward_state =WITHOUT_FORWARD;
			
			CalculateVelocities(&Js_Value,&nav,500,3500,500,3500,500,75);//这里算出来单位都是mm，这里手柄分配最大速度是3500mm/s，转速75°/s
			
			ramp_signal(&nav.expect_robot_global_velt.fpX,nav.auto_path.basic_velt.fpVx,LineAccelStep);
			ramp_signal(&nav.expect_robot_global_velt.fpY,nav.auto_path.basic_velt.fpVy,LineAccelStep);
		
//			if(flag_global_manual == 1)
//			{
//				Chassis_Global_Yaw_Pid.outer.fpDes = fpSumPosQ /10;//记录此时的车身旋转累计角度值（累计角度值就可以无限转了）
//				flag_global_manual = 0;
//			}
//			if(fabs((float)(Js_Value.usJsRight_X - RIGHT_JS_MID)) > 1000)
//			{
//				Chassis_Global_Yaw_Pid.outer.fpDes -= 0.09*Sgn((float)(Js_Value.usJsRight_X - RIGHT_JS_MID));
//				nav.auto_path.basic_velt.fpW = -90*Sgn((float)(Js_Value.usJsRight_X - RIGHT_JS_MID));
//			}
//			else 
//			{
//				nav.auto_path.basic_velt.fpW = 0;
//			}
//			PID_Calc_NEW(&Chassis_Global_Yaw_Pid.outer);
//			Chassis_Global_Yaw_Pid.inner.fpDes = nav.auto_path.basic_velt.fpW + Chassis_Global_Yaw_Pid.outer.fpU;
//			PID_Calc_NEW(&Chassis_Global_Yaw_Pid.inner);
//			
//			nav.expect_robot_global_velt.fpW = Chassis_Global_Yaw_Pid.inner.fpU;
			if(flag_global_manual == 1)
			{
				Chassis_Global_Yaw_Pid.fpDes = fpSumPosQ /10;//记录此时的车身旋转累计角度值（累计角度值就可以无限转了）
				flag_global_manual = 0;
			}
			if(fabs(nav.expect_robot_global_velt.fpW) > 30)
			{
				Chassis_Global_Yaw_Pid.fpDes += 0.12*Sgn(nav.expect_robot_global_velt.fpW);
//				nav.auto_path.basic_velt.fpW = 75*Sgn(nav.expect_robot_global_velt.fpW);//顺便给速度前馈，降低PID负担
			}
			else 
			{
				nav.auto_path.basic_velt.fpW = 0;
			}
			Chassis_Global_Yaw_Pid.fpFB = fpSumPosQ /10;
			PID_Calc_NEW(&Chassis_Global_Yaw_Pid);
			
			nav.expect_robot_global_velt.fpW = Chassis_Global_Yaw_Pid.fpU + nav.auto_path.basic_velt.fpW;
			
			SpeedDistribute_Four_OmnidriectionalWhile(&nav);
			break;
			
		case NAV_AIM_MANUAL:
			chassis_run.pid_state=VELT_LOOP;
			chassis_run.feed_forward_state =WITHOUT_FORWARD;
		
			CalculateVelocities(&Js_Value,&nav,500,3000,500,3000,500,75);//这里算出来单位都是mm，这里手柄分配最大速度是2300mm/s，转速75°/s
		
			ramp_signal(&nav.expect_robot_global_velt.fpX,nav.auto_path.basic_velt.fpVx,LineAccelStep);
			ramp_signal(&nav.expect_robot_global_velt.fpY,nav.auto_path.basic_velt.fpVy,LineAccelStep);
		
			PID_Calc(&Chassis_Aim_Yaw_Pid,0,Vision_Data.aim_chassis_yaw);
			nav.expect_robot_global_velt.fpW = Chassis_Aim_Yaw_Pid.fpU;
		
			SpeedDistribute_Four_OmnidriectionalWhile(&nav);	
			break;
		
		case NAV_AUTO_PATH:
			chassis_run.pid_state=VELT_LOOP;
			chassis_run.feed_forward_state =WITHOUT_FORWARD;
	
			path_choose(&nav);//选择哪个打点路径:这里主要使用精细校准模式
			
			PID_Calc_NEW(&nav.auto_path.pos_pid.x);
			PID_Calc_NEW(&nav.auto_path.pos_pid.y);
			PID_Calc_NEW(&nav.auto_path.pos_pid.w);
		
			nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.x.fpU + nav.auto_path.basic_velt.fpVx;
			nav.expect_robot_global_velt.fpY = nav.auto_path.pos_pid.y.fpU + nav.auto_path.basic_velt.fpVy;
			nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU + nav.auto_path.basic_velt.fpW;
		
			SpeedDistribute_Four_OmnidriectionalWhile(&nav);
			break;
		
		case NAV_PERMUTATION_PATH:
			chassis_run.pid_state=VELT_LOOP ;
			chassis_run.feed_forward_state =WITHOUT_FORWARD;
			
			if(flag_permutation_path==1)//保证只有在一段大路径的每一段小路径开头才进行选择路径
			{														//或者是两段大路径切换时才进行选择，防止多次进行重复打点
				path_permutation_choose(&nav);
				flag_permutation_path=0;
			}
			NavPosition(&nav,&Path_Permuta);
			
			if(flag_rotation==1)
			{
				NavRotation(&nav,&Path_Permuta);
			}
			
			CheckPathEnd(&nav,&Path_Permuta);
		
			PID_Calc_NEW(&nav.auto_path.pos_pid.x);
			PID_Calc_NEW(&nav.auto_path.pos_pid.y);
			PID_Calc_NEW(&nav.auto_path.pos_pid.w);
		
			nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.x.fpU + nav.auto_path.basic_velt.fpVx;
			nav.expect_robot_global_velt.fpY = nav.auto_path.pos_pid.y.fpU + nav.auto_path.basic_velt.fpVy;
			nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU + nav.auto_path.basic_velt.fpW;
		
			SpeedDistribute_Four_OmnidriectionalWhile(&nav);
			break;
			
		//以下为视觉发过来的数据
		case NAV_VISION_MANUAL:
			chassis_run.pid_state=VELT_LOOP ;
			chassis_run.feed_forward_state =WITHOUT_FORWARD;
			
			nav.expect_robot_global_velt.fpX = Vision_Data.joy_v_x;
			nav.expect_robot_global_velt.fpY = Vision_Data.joy_v_y;
			nav.expect_robot_global_velt.fpW = Vision_Data.joy_v_w;
		
			SpeedDistribute_Four_OmnidriectionalWhile(&nav);
			break;
		
		case NAV_VISION_AIM_MANUAL:
			chassis_run.pid_state=VELT_LOOP ;
			chassis_run.feed_forward_state =WITHOUT_FORWARD;
			
			nav.expect_robot_global_velt.fpX = Vision_Data.joy_v_x;
			nav.expect_robot_global_velt.fpY = Vision_Data.joy_v_y;
			
			PID_Calc(&Chassis_Aim_Yaw_Pid,0,Vision_Data.aim_chassis_yaw);
			nav.expect_robot_global_velt.fpW = Chassis_Aim_Yaw_Pid.fpU;
		
			SpeedDistribute_Four_OmnidriectionalWhile(&nav);
			break;
		
		case NAV_VISION_PATH:
			chassis_run.pid_state=VELT_LOOP ;
			chassis_run.feed_forward_state =WITHOUT_FORWARD;
			if(Vision_Data.nav_flag == 1)
			{
				nav.auto_path.basic_velt.fpVx = Vision_Data.nav_v_x;
				nav.auto_path.basic_velt.fpVy = Vision_Data.nav_v_y;
				
				nav.auto_path.pos_pid.x.fpDes  = Vision_Data.nav_pos_x;
				nav.auto_path.pos_pid.y.fpDes  = Vision_Data.nav_pos_y;
				nav.auto_path.pos_pid.w.fpDes  = Vision_Data.nav_pos_q;
			
				PID_Calc_NEW(&nav.auto_path.pos_pid.x);
				PID_Calc_NEW(&nav.auto_path.pos_pid.y);
				PID_Calc_NEW(&nav.auto_path.pos_pid.w);
				
				nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.x.fpU + nav.auto_path.basic_velt.fpVx;
				nav.expect_robot_global_velt.fpY = nav.auto_path.pos_pid.y.fpU + nav.auto_path.basic_velt.fpVy;
				nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU;
				
				SpeedDistribute_Four_OmnidriectionalWhile(&nav);
			}
			else 
			{
				chassis_run.rightup.fpDes = 0;
				chassis_run.leftup.fpDes = 0;
				chassis_run.leftdown.fpDes = 0;
				chassis_run.rightdown.fpDes = 0;
			}
			break;
		default:
				break;
	}
	
}


