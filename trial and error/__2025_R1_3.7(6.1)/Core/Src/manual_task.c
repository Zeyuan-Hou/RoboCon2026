#include "manual_task.h"

void Manual_task(void)
{
	if(Manual_Task_State != LOCKED)
	{
		switch(Manual_Mode)
		{
			case NORMAL_MANUAL:
				nav.nav_state = NAV_MANUAL;
				break;
			case GLOBA_MANUAL:
				nav.nav_state = NAV_GLOBAL_MANUAL;
				break;
			case AIM_MANUAL:
				nav.nav_state = NAV_AIM_MANUAL;
				break;
			case VISION_NORMAL_MANUAL:
				nav.nav_state = NAV_VISION_MANUAL;
				break;
			case VISION_AIM_MANUAL:
				nav.nav_state = NAV_VISION_AIM_MANUAL;
				break;
			default:
				break;
		}
		//手操接球
		if(Rec_Ball_flag == 1)
		{
			Receive_ball();
			if(Rec_Ball_State == REC_END)
				Rec_Ball_flag = 0;
		}
		//手操运球
		if(Drib_flag == 1)
		{
			Dribble();
			if(Dribble_State == DRIBBLE_VIC || Dribble_State == DRIBBLE_LOSE)
				Drib_flag = 0;
		}
		//手操放球
		if(Load_flag == 1)
		{
			Load_ball();
			if(Load_State == LOAD_END)//放球结束
				Load_flag = 0;
		}
		
		//手操的自瞄发射
		if(uart1_efr.num[0] == 1)//接收到发射完毕
		{
			uart1_eft.num[4] = 0;
			uart1_eft.num[5] = 0;
		}
		
		//手操的运球+放球    有点用不上
//		Dribble_Load();
	}
}

