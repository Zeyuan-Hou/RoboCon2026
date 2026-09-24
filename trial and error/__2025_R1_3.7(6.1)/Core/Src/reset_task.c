#include "reset_task.h"

void Reset_task(void)
{
	if(Reset_Task_State != LOCKED)
	{
		//底盘reset
		nav.nav_state = NAV_LOCK;//停下路径，锁到当前位置
		flag_lock = 1;
		flag_global_manual = 1;
		path_state = PATH_END;
		
		//接球动作reset
		Rec_Ball_State = J60_LIFT;//接球状态回归初始
		
		//运球动作reset
		Dribble_State = MOTOR_SPEED_UP;//运球状态回归初始
		start_from_drib = 0;//计时标志位归0
		Drib_Left_Targetv = 0;//两摩擦轮停下
		Drib_Right_Targetv = 0;
		CLOSE_VALVE_Plus(1);//夹爪夹住
		
		//放球动作reset
		Load_State = 0;//放球状态回归初始
		uart1_eft.num[3] = 0;//给上板的放球状态归0
		
		//手动模式reset
		Rec_Ball_flag = 0;//手操接球标志位归0，防止
		Drib_flag = 0;//手操运球标志位归0，防止之前运球还没结束，
		Load_flag = 0;//手操放球标志位归0
		Drib_Load = DRIB_LOAD_INIT;//运球+放球标志位初始化
		uart1_eft.num[4] = 0;//自瞄发射标志位归0
		
		//投篮挑战赛reset
		shoot_number = 0;
		Shoot_Challenge = SHOOT_PATH1;
		uart1_eft.num[4] = 0;
		
		//运球挑战赛reset
		flag_second_drib = 0;//运球次数标志位归0
		Dribble_Challenge = DRIB_EXIT;//运球挑战赛归0
		
		J60_target_pos = 80;//58;//J60移到运球位置（因为只有接球之后才会跑到运球的位置，除此之外没有赋过58这个值）
		
		if(fabs(Drib_Motor_Left.anglev)<100&&fabs(Drib_Motor_Right.anglev)<100&&(((J60_Enable_State==ENABLE_STATE)&&(fabs(J60_target_pos - J60_angle)< 5))||J60_Enable_State==DISABLE_STATE))//确认所有状态都reset之后把reset任务锁上，防止一些东西在持续赋值
		{
			Reset_Task_State = LOCKED;
		}
				
	}
}



