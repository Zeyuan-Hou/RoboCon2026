#include "dunk.h"
/*********************************************************************************************************************
函数功能：整个扣篮过程

		0.DUNK_INIT(扣篮前的所有准备):机械臂电机使能，气泵进行吸球，跳跃电机读取零点后进入位置控制
		1.ARM_INIT(机械臂准备)：先抬到一定高度再进行跳跃，这里抬升的高度要调节
		2.DRIVE_LEG(蹬腿过程)：跳跃电机给一个巨大的位置期望，持续发满电流。同时进行机械臂上甩进行扣篮。
													机构快到顶端之后延时，让机构撞上去实现离地的效果。此时进行松球扣篮，并进入收腿状态
		3.DRAW_BACK_LEG(收腿过程)：此时机械臂失能防烧，这时候收腿一段时间后进入阻尼模式，开始落地
		4.LAND(落地过程)：底盘快到底部或者底盘卡在阻尼模式时跳跃电机失能，一切回归初始化
		5.DUNK_END(扣篮结束)：扣篮结束的标志位
		
		除此之外还有整个状态机的保护，如果运行时间不正常，则立即停止该过程，一切归零
***********************************************************************************************************************/
u8 start_from_dunk;
uint32_t Time_dunk;

u8 start_from_dunk_LAND;
uint32_t Time_dunk_LAND;
//用于比较高的跳跃
//void Dunk(void)
//{
//	if(start_from_dunk == 0)
//	{
//		Time_dunk = Current_Time;
//		start_from_dunk = 1;
//	}
//	if((Current_Time - Time_dunk)< 6000 && J60_Enable_State == ENABLE_STATE)//在正常时间内完成扣篮，正常运行
//	{
//		switch(Dunk_State)
//		{
//			case DUNK_INIT:
//				J60_Ctrl_Mode = CONTROL_MODE;//J60进入位控模式
//				Suction_State = CATCH_BALL;//确保球被吸着
//				if(jump_motor_pos_mode() == 1)//电机进行位控
//				{
//					Dunk_State = ARM_INIT;//跳到下一状态
//				}
//				break;
//			case ARM_INIT:
//				J60_target_pos = 180;//臂先抬起一定角度
//				if(fabs(J60_target_pos - J60_angle)<3)//臂抬到位之后进行跳跃
//				{
//					vTaskDelay(pdMS_TO_TICKS(1500));
//					Dunk_State = DRIVE_LEG;
//				}
//				break;
//			
//			case DRIVE_LEG:
//				jump_con_delta_pos = 400;
//				if(fabs(left_xc5000_motor.Real_Delta_Pos)>105
//				 ||fabs(right_xc5000_motor.Real_Delta_Pos)>105
//				 ||fabs(left_4219_motor.Real_Delta_Pos)>105
//				 ||fabs(right_4219_motor.Real_Delta_Pos)>105)//快到顶部了
//				{
//					vTaskDelay(pdMS_TO_TICKS(50));//延时让撞上去
//					Dunk_State = DRAW_BACK_LEG;//此时进入收腿状态
//				}
//				break;
//			
//			case DRAW_BACK_LEG:
//				jump_con_delta_pos = 20;//收一部分腿
//				J60_td.r = 3000;
//				J60_target_pos = 270;
//				if(fabs(left_xc5000_motor.Real_Delta_Pos - left_xc5000_motor.Target_Delta_Pos)<5
//				 ||fabs(right_xc5000_motor.Real_Delta_Pos - right_xc5000_motor.Target_Delta_Pos)<5
//				 ||fabs(left_4219_motor.Real_Delta_Pos - left_4219_motor.Target_Delta_Pos)<5
//				 ||fabs(right_4219_motor.Real_Delta_Pos - right_4219_motor.Target_Delta_Pos)<5)
//				{
//					J60_Ctrl_Mode = NO_TORQUE_MODE;//机械臂电机失能
////					Suction_State = LOOSE_BALL;//升到最高之后松球
//					jump_motor_damp_mode();
//					Dunk_State = LAND;//最后缓冲落下，扣篮结束
//				}
//				break;
//			
//			case LAND:
//				if(start_from_dunk_LAND == 0)
//				{
//					Time_dunk_LAND = Current_Time;
//					start_from_dunk_LAND = 1;
//				}
//				if(fabs(left_xc5000_motor.Real_Delta_Pos)<10
//				 ||fabs(right_xc5000_motor.Real_Delta_Pos)<10
//				 ||fabs(left_4219_motor.Real_Delta_Pos)<10
//				 ||fabs(right_4219_motor.Real_Delta_Pos)<10
//				 ||(Current_Time - Time_dunk_LAND)> 1000)//底盘快落地了或者被阻尼模式卡住太长时间了，跳跃电机直接失能，防止底盘卡在半空
//				{
//					left_xc5000_motor.Pid_State = OPEN_LOOP;//取消发送电流
//					right_xc5000_motor.Pid_State = OPEN_LOOP;
//					left_4219_motor.Pid_State = OPEN_LOOP;
//					right_4219_motor.Pid_State = OPEN_LOOP;
//					
//					Suction_State = ALL_CLOSE;//松球之后吸盘任务关闭
//					jump_con_delta_pos = 0;//恢复期望角度
//					J60_td.r = 600;
//					J60_target_pos = 0;//恢复期望角度
//					start_from_dunk_LAND = 0;
//					Dunk_State = DUNK_END;
//					
//					start_from_dunk = 0;
//				}
//				break;
//			
//			case DUNK_END:
//				break;
//			default :
//				break;
//		}
//	}
//	else//卡在某一部分时，自动退出该任务，所有电机进行失能处理
//	{
//		Suction_State = ALL_CLOSE;//吸盘任务关闭
//		J60_Ctrl_Mode = NO_TORQUE_MODE;//机械臂电机失能
//		
//		left_xc5000_motor.Pid_State = OPEN_LOOP;//取消发送电流
//		right_xc5000_motor.Pid_State = OPEN_LOOP;
//		left_4219_motor.Pid_State = OPEN_LOOP;
//		right_4219_motor.Pid_State = OPEN_LOOP;
//		
//		jump_con_delta_pos = 0;//恢复期望角度
//		J60_td.r = 600;
//		J60_target_pos = 0;//恢复期望角度
//		start_from_dunk_LAND = 0;
//		Dunk_State = DUNK_END;
//		
//		start_from_dunk = 0;
//	}
//}
//用于比较矮的跳跃
//void Dunk(void)
//{
//	if(start_from_dunk == 0)
//	{
//		Time_dunk = Current_Time;
//		start_from_dunk = 1;
//	}
//	if((Current_Time - Time_dunk)< 6000 && J60_Enable_State == ENABLE_STATE)//在正常时间内完成扣篮，正常运行
//	{
//		switch(Dunk_State)
//		{
//			case DUNK_INIT:
//				J60_Ctrl_Mode = CONTROL_MODE;//J60进入位控模式
//				Suction_State = CATCH_BALL;//确保球被吸着
//				if(jump_motor_pos_mode() == 1)//电机进行位控
//				{
//					Dunk_State = ARM_INIT;//跳到下一状态
//				}
//				break;
//			case ARM_INIT:
//				J60_target_pos = 180;//臂先抬起一定角度
//				if(fabs(J60_target_pos - J60_angle)<3)//臂抬到位之后进行跳跃
//				{
//					vTaskDelay(pdMS_TO_TICKS(2000));
//					Dunk_State = DRIVE_LEG;
//				}
//				break;
//			
//			case DRIVE_LEG:
//				jump_con_delta_pos = 400;
//				if(fabs(left_xc5000_motor.Real_Delta_Pos)>80
//				 ||fabs(right_xc5000_motor.Real_Delta_Pos)>80
//				 ||fabs(left_4219_motor.Real_Delta_Pos)>80
//				 ||fabs(right_4219_motor.Real_Delta_Pos)>80)
//				{
//					Dunk_State = DRAW_BACK_LEG;//此时进入收腿状态
//				}
//				break;
//			
//			case DRAW_BACK_LEG:
//				jump_con_delta_pos = 20;//收一部分腿
//				J60_td.r = 6000;
//				J60_target_pos = 265;
//				if(fabs(left_xc5000_motor.Real_Delta_Pos - left_xc5000_motor.Target_Delta_Pos)<5
//				 ||fabs(right_xc5000_motor.Real_Delta_Pos - right_xc5000_motor.Target_Delta_Pos)<5
//				 ||fabs(left_4219_motor.Real_Delta_Pos - left_4219_motor.Target_Delta_Pos)<5
//				 ||fabs(right_4219_motor.Real_Delta_Pos - right_4219_motor.Target_Delta_Pos)<5)
//				{
//					J60_Ctrl_Mode = NO_TORQUE_MODE;//机械臂电机失能
////					Suction_State = LOOSE_BALL;//升到最高之后松球
//					jump_motor_damp_mode();
//					Dunk_State = LAND;//最后缓冲落下，扣篮结束
//				}
//				break;
//			
//			case LAND:
//				if(start_from_dunk_LAND == 0)
//				{
//					Time_dunk_LAND = Current_Time;
//					start_from_dunk_LAND = 1;
//				}
//				if(fabs(left_xc5000_motor.Real_Delta_Pos)<10
//				 ||fabs(right_xc5000_motor.Real_Delta_Pos)<10
//				 ||fabs(left_4219_motor.Real_Delta_Pos)<10
//				 ||fabs(right_4219_motor.Real_Delta_Pos)<10
//				 ||(Current_Time - Time_dunk_LAND)> 1000)//底盘快落地了或者被阻尼模式卡住太长时间了，跳跃电机直接失能，防止底盘卡在半空
//				{
//					left_xc5000_motor.Pid_State = OPEN_LOOP;//取消发送电流
//					right_xc5000_motor.Pid_State = OPEN_LOOP;
//					left_4219_motor.Pid_State = OPEN_LOOP;
//					right_4219_motor.Pid_State = OPEN_LOOP;
//					
//					Suction_State = ALL_CLOSE;//松球之后吸盘任务关闭
//					jump_con_delta_pos = 0;//恢复期望角度
//					J60_td.r = 600;
//					J60_target_pos = 0;//恢复期望角度
//					start_from_dunk_LAND = 0;
//					Dunk_State = DUNK_END;
//					
//					start_from_dunk = 0;
//				}
//				break;
//			
//			case DUNK_END:
//				break;
//			default :
//				break;
//		}
//	}
//	else//卡在某一部分时，自动退出该任务，所有电机进行失能处理
//	{
////		Suction_State = ALL_CLOSE;//吸盘任务关闭
//		J60_Ctrl_Mode = NO_TORQUE_MODE;//机械臂电机失能
//		
//		left_xc5000_motor.Pid_State = OPEN_LOOP;//取消发送电流
//		right_xc5000_motor.Pid_State = OPEN_LOOP;
//		left_4219_motor.Pid_State = OPEN_LOOP;
//		right_4219_motor.Pid_State = OPEN_LOOP;
//		
//		jump_con_delta_pos = 0;//恢复期望角度
//		J60_td.r = 600;
//		J60_target_pos = 0;//恢复期望角度
//		start_from_dunk_LAND = 0;
//		Dunk_State = DUNK_END;
//		
//		start_from_dunk = 0;
//	}
//}
void Dunk(void)
{
	if(start_from_dunk == 0)
	{
		Time_dunk = Current_Time;
		start_from_dunk = 1;
	}
	if((Current_Time - Time_dunk)< 7000)// && J60_Enable_State == ENABLE_STATE)//在正常时间内完成扣篮，正常运行
	{
		switch(Dunk_State)
		{
			case DUNK_INIT:
				Motor_A1.Ctrl_Data.mode = ENABLE_MODE;//J60进入位控模式
				Suction_State = CATCH_BALL;//确保球被吸着
				if(jump_motor_pos_mode() == 1)//电机进行位控
				{
					Dunk_State = ARM_INIT;//跳到下一状态
				}
				break;
			case ARM_INIT:
				A1_target_pos = 170;//臂先抬起一定角度
				if(fabs(A1_target_pos - Motor_A1.Real_Delta_Deg)<3)//臂抬到位之后进行跳跃
				{
					vTaskDelay(pdMS_TO_TICKS(1000));
					Dunk_State = DRIVE_LEG;
				}
				break;
			
			case DRIVE_LEG:
				jump_con_delta_pos = 400;
				if(fabs(left_xc5000_motor.Real_Delta_Pos)>110
				 ||fabs(right_xc5000_motor.Real_Delta_Pos)>110
				 ||fabs(left_4219_motor.Real_Delta_Pos)>110
				 ||fabs(right_4219_motor.Real_Delta_Pos)>110)
				{
					vTaskDelay(pdMS_TO_TICKS(50));//延时让撞上去
					Dunk_State = DRAW_BACK_LEG;//此时进入收腿状态
				}
				break;
			
			case DRAW_BACK_LEG:
				jump_con_delta_pos = -100;//收一部分腿
				if(fabs(left_xc5000_motor.Real_Delta_Pos)<45
				 ||fabs(right_xc5000_motor.Real_Delta_Pos)<45
				 ||fabs(left_4219_motor.Real_Delta_Pos)<45
				 ||fabs(right_4219_motor.Real_Delta_Pos)<45)
				{
					Suction_State = LOOSE_BALL;//升到最高之后松球
					OPEN_VALVE(6);//拍球气缸进行平抛
					jump_motor_damp_mode();
					Dunk_State = LAND;//最后缓冲落下，扣篮结束
				}
				break;
			
			case LAND:
				if(start_from_dunk_LAND == 0)
				{
					Time_dunk_LAND = Current_Time;
					start_from_dunk_LAND = 1;
				}
				if(fabs(left_xc5000_motor.Real_Delta_Pos)<5
				 ||fabs(right_xc5000_motor.Real_Delta_Pos)<5
				 ||fabs(left_4219_motor.Real_Delta_Pos)<5
				 ||fabs(right_4219_motor.Real_Delta_Pos)<5
				 ||(Current_Time - Time_dunk_LAND)> 1000)//底盘快落地了或者被阻尼模式卡住太长时间了，跳跃电机直接失能，防止底盘卡在半空
				{	
					CLOSE_VALVE(6);//拍球气缸回收
					left_xc5000_motor.Pid_State = OPEN_LOOP;//取消发送电流
					right_xc5000_motor.Pid_State = OPEN_LOOP;
					left_4219_motor.Pid_State = OPEN_LOOP;
					right_4219_motor.Pid_State = OPEN_LOOP;
					
					Suction_State = ALL_CLOSE;//松球之后吸盘任务关闭
					jump_con_delta_pos = 0;//恢复期望角度
//					A1_target_pos = 0;//恢复期望角度
					start_from_dunk_LAND = 0;
					Dunk_State = DUNK_END;
					
					start_from_dunk = 0;
				}
				break;
			
			case DUNK_END:
				break;
			default :
				break;
		}
	}
	else//卡在某一部分时，自动退出该任务，所有电机进行失能处理
	{
		Suction_State = ALL_CLOSE;//吸盘任务关闭
		Motor_A1.Ctrl_Data.mode = DISABLE_MODE;//机械臂电机失能
		
		left_xc5000_motor.Pid_State = OPEN_LOOP;//取消发送电流
		right_xc5000_motor.Pid_State = OPEN_LOOP;
		left_4219_motor.Pid_State = OPEN_LOOP;
		right_4219_motor.Pid_State = OPEN_LOOP;
		
		jump_con_delta_pos = 0;//恢复期望角度
		A1_target_pos = 0;//恢复期望角度
		start_from_dunk_LAND = 0;
		Dunk_State = DUNK_END;
		
		start_from_dunk = 0;
	}
}
