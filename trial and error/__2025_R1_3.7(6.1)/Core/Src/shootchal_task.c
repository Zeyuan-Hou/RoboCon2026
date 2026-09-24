#include "shootchal_task.h"


u8 shoot_number;
void reset_load()
{
	if(Js_Value.usJsKey==35||switch_on_2==1)
	{
		switch(shoot_number)
		{
			case 0:
				break;
			case 1:
				Shoot_Challenge = SHOOT_PATH2;
				break;
			case 2:
				Shoot_Challenge = SHOOT_PATH3;
				break;
			case 3:
				Shoot_Challenge = SHOOT_PATH4;
				break;
			case 4:
				Shoot_Challenge = SHOOT_PATH5;
				break;
			case 5:
				Shoot_Challenge = SHOOT_PATH6;
				break;
			case 6:
				Shoot_Challenge = SHOOT_PATH7;
				break;
			case 7://第七次发射完跑到固定点位
				Shoot_Challenge = SHOOT_PATH_FIXED;
				break;
			case 8://之后一直在跑固定点位
				Shoot_Challenge = SHOOT_PATH_FIXED;
				break;
			default:
				break;
		}
	}
}

void Shoot_challenge(void)
{
	if(ShootChal_Task_State != LOCKED)
	{
		switch(Shoot_Challenge)
		{
			case SHOOT_PATH1:
				if(path_state == PATH_INIT || path_state == PATH_END)
				{
					point_end.x = -1500;//-2200;
					point_end.y = 1500;//2400;
					SET_NAV_PATH_AUTO(1);
//					SET_NAV_PATH_AUTO(3);
					
					Shoot_Challenge = ROTATION_READY;
				}
				break;
			case SHOOT_PATH2:
				if(path_state == PATH_END)
				{
					point_end.x = -350;
					point_end.y = 1700;
					SET_NAV_PATH_AUTO(1);
//					SET_NAV_PATH_AUTO(3);
					
					Shoot_Challenge = ROTATION_READY;
				}
				break;
			case SHOOT_PATH3:
				if(path_state == PATH_END)
				{
					point_end.x = 100;
					point_end.y = 1700;
					SET_NAV_PATH_AUTO(1);
//					SET_NAV_PATH_AUTO(3);
					
					Shoot_Challenge = ROTATION_READY;
				}
				break;
			case SHOOT_PATH4:
				if(path_state == PATH_END)
				{
					point_end.x = 100;
					point_end.y = 700;
					SET_NAV_PATH_AUTO(1);
//					SET_NAV_PATH_AUTO(3);
					
					Shoot_Challenge = ROTATION_READY;
				}
				break;
			case SHOOT_PATH5:
				if(path_state == PATH_END)
				{
					point_end.x = 1200;
					point_end.y = 1800;
					SET_NAV_PATH_AUTO(1);
//					SET_NAV_PATH_AUTO(3);
					
					Shoot_Challenge = ROTATION_READY;
				}
				break;
			case SHOOT_PATH6:
				if(path_state == PATH_END)
				{
					point_end.x = -1000;
					point_end.y = 240;
					SET_NAV_PATH_AUTO(1);
//					SET_NAV_PATH_AUTO(3);
					
					Shoot_Challenge = ROTATION_READY;
				}
				break;
			case SHOOT_PATH7:
				if(path_state == PATH_END)
				{
					point_end.x = 0;
					point_end.y = 0;
					SET_NAV_PATH_AUTO(1);
//					SET_NAV_PATH_AUTO(3);
					
					Shoot_Challenge = ROTATION_READY;
				}
				break;
			case SHOOT_PATH_FIXED:
				if(path_state == PATH_END)
				{
					point_end.x = 100;
					point_end.y = 1700;
					SET_NAV_PATH_AUTO(1);
//					SET_NAV_PATH_AUTO(3);
					
					Shoot_Challenge = ROTATION_READY;
				}
				break;
			case ROTATION_READY:
				if(path_state == PATH_END)
				{
					if(fabs(nav.auto_path.pos_pid.w.fpDes - nav.auto_path.pos_pid.w.fpFB) < 2)//到点后如果yaw轴跟上了就通知上板发送
							Shoot_Challenge = SHOOT_READY;
				}
				break;
			case SHOOT_READY:
				if(path_state == PATH_END)
				{
					uart1_eft.num[4] = 1;//已经到达目标点位和角度，提示发射
					if(uart1_efr.num[0] == 1)//接收到发射完毕
					{
						uart1_eft.num[4] = 0;//发射完了，归零
						if(shoot_number <= 7)//直到在固定点位发射就不再增加了，最后一直跑到固定点位
						{
							shoot_number++;
						}
						point_end.x = 0;
						point_end.y = 0;
						SET_NAV_PATH_AUTO(1);
						
						Shoot_Challenge = RESET_LOAD;
					}
				}
				break;
			case RESET_LOAD:
				if(path_state == PATH_END)
				{
					reset_load();
				}
				break;
		}
	}
}



