#include "drib_task.h"

/*********************************************************************************************************
函数名称：void Dribble_challenge(void)
函数功能：完成运球挑战赛的七段路径
备注:
		初始状态INIT不会动，直接放freertos里，要开始的话修改Dribble_Challenge=DRIB_PATH1即可
		
		
**********************************************************************************************************/ 
u8 flag_second_drib;
DRIB_RESET Drib_Reset;
void Dribble_challenge(void)
{
	if(DribChal_Task_State != LOCKED)
	{
		switch(Dribble_Challenge)
		{
			case DRIB_EXIT:
				if(path_state == PATH_END || path_state == PATH_INIT)
				{
					point_end.x = -1400;
					point_end.y = 0;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);
					
					Dribble_Challenge = DRIB_PATH1;
				}
			case DRIB_PATH1:
				if(path_state == PATH_END || path_state == PATH_INIT)
				{
					Dribble();//在出发点运球,运球结束之后 
					if(Dribble_State == DRIBBLE_VIC && flag_second_drib == 0 && switch_on_2 == 1)//第一次运球成功，下一次运球
					{
						vTaskDelay(pdMS_TO_TICKS(1000));
						Dribble_State = MOTOR_SPEED_UP;
						flag_second_drib++;
					}
					if(Dribble_State == DRIBBLE_LOSE && flag_second_drib == 0)//第一次运球失误，进入重启区，接到球后自动回来继续运球
					{
						Dribble_Challenge = DRIB_RESET1_1;
						Dribble_State = MOTOR_SPEED_UP;
						Drib_Reset = RESET_GO;
						Drib_Chal_Record = DRIB_PATH1;
						flag_second_drib++;
					}
					if(Dribble_State == DRIBBLE_VIC && flag_second_drib == 1)//第二次运球成功，到下一个点
					{
						point_end.x = -700;
						point_end.y = 700;
						point_end.q = 0;
						SET_NAV_PATH_AUTO(1);
						
						vTaskDelay(pdMS_TO_TICKS(100));
						Dribble_Challenge = DRIB_PATH2;
						Dribble_State = MOTOR_SPEED_UP;
						flag_second_drib = 0;
					}
					if(Dribble_State == DRIBBLE_LOSE && flag_second_drib == 1)//第二次运球失误，进入重启区，接到球后自动到下一个点
					{
						Dribble_Challenge = DRIB_RESET1_2;
						Dribble_State = MOTOR_SPEED_UP;
						Drib_Reset = RESET_GO;
						Drib_Chal_Record = DRIB_PATH2;
						flag_second_drib = 0;
					}
				}
				break;
				
			case DRIB_PATH2:
				if(path_state == PATH_END || path_state == PATH_INIT)
				{
					Dribble();//在出发点运球,运球结束之后 
					if(Dribble_State == DRIBBLE_VIC && flag_second_drib == 0 && switch_on_2 == 1)//第一次运球成功，下一次运球
					{
						vTaskDelay(pdMS_TO_TICKS(1000));
						Dribble_State = MOTOR_SPEED_UP;
						flag_second_drib++;
					}
					if(Dribble_State == DRIBBLE_LOSE && flag_second_drib == 0)//第一次运球失误，进入重启区，接到球后自动回来继续运球
					{
						Dribble_Challenge = DRIB_RESET2_1;
						Dribble_State = MOTOR_SPEED_UP;
						Drib_Reset = RESET_GO;
						Drib_Chal_Record = DRIB_PATH2;
						flag_second_drib++;
					}
					if(Dribble_State == DRIBBLE_VIC && flag_second_drib == 1)//第二次运球成功，到下一个点
					{
						point_end.x = -700;
						point_end.y = 2400;
						point_end.q = 0;
						SET_NAV_PATH_AUTO(1);
						
						vTaskDelay(pdMS_TO_TICKS(100));
						Dribble_Challenge = DRIB_PATH3;
						Dribble_State = MOTOR_SPEED_UP;
						flag_second_drib = 0;
					}
					if(Dribble_State == DRIBBLE_LOSE && flag_second_drib == 1)//第二次运球失误，进入重启区，接到球后自动到下一个点
					{
						Dribble_Challenge = DRIB_RESET2_2;
						Dribble_State = MOTOR_SPEED_UP;
						Drib_Reset = RESET_GO;
						Drib_Chal_Record = DRIB_PATH3;
						flag_second_drib = 0;
					}
				}
				break;
				
			case DRIB_PATH3:
				if(path_state == PATH_END || path_state == PATH_INIT)
				{
					Dribble();//在出发点运球,运球结束之后 
					if(Dribble_State == DRIBBLE_VIC && flag_second_drib == 0 && switch_on_2 == 1)//第一次运球成功，下一次运球
					{
						vTaskDelay(pdMS_TO_TICKS(1000));
						Dribble_State = MOTOR_SPEED_UP;
						flag_second_drib++;
					}
					if(Dribble_State == DRIBBLE_LOSE && flag_second_drib == 0)//第一次运球失误，进入重启区，接到球后自动回来继续运球
					{
						Dribble_Challenge = DRIB_RESET3_1;
						Dribble_State = MOTOR_SPEED_UP;
						Drib_Reset = RESET_GO;
						Drib_Chal_Record = DRIB_PATH3;
						flag_second_drib++;
					}
					if(Dribble_State == DRIBBLE_VIC && flag_second_drib == 1)//第二次运球成功，到下一个点
					{
						point_end.x = -2600;
						point_end.y = 2400;
						point_end.q = 0;
						SET_NAV_PATH_AUTO(1);
						
						vTaskDelay(pdMS_TO_TICKS(100));
						Dribble_Challenge = DRIB_PATH4;
						Dribble_State = MOTOR_SPEED_UP;
						flag_second_drib = 0;
					}
					if(Dribble_State == DRIBBLE_LOSE && flag_second_drib == 1)//第二次运球失误，进入重启区，接到球后自动到下一个点
					{
						Dribble_Challenge = DRIB_RESET3_2;
						Dribble_State = MOTOR_SPEED_UP;
						Drib_Reset = RESET_GO;
						Drib_Chal_Record = DRIB_PATH4;
						flag_second_drib = 0;
					}
				}
				break;
				
			case DRIB_PATH4:
				if(path_state == PATH_END || path_state == PATH_INIT)
				{
					Dribble();//在出发点运球,运球结束之后  
					if(Dribble_State == DRIBBLE_VIC && flag_second_drib == 0 && switch_on_2 == 1)//第一次运球成功，下一次运球
					{
						vTaskDelay(pdMS_TO_TICKS(1000));
						Dribble_State = MOTOR_SPEED_UP;
						flag_second_drib++;
					}
					if(Dribble_State == DRIBBLE_LOSE && flag_second_drib == 0)//第一次运球失误，进入重启区，接到球后自动回来继续运球
					{
						Dribble_Challenge = DRIB_RESET4_1;
						Dribble_State = MOTOR_SPEED_UP;
						Drib_Reset = RESET_GO;
						Drib_Chal_Record = DRIB_PATH4;
						flag_second_drib++;
					}
					if(Dribble_State == DRIBBLE_VIC && flag_second_drib == 1)//第二次运球成功，到下一个点
					{
						point_end.x = -4500;
						point_end.y = 2400;
						point_end.q = 0;
						SET_NAV_PATH_AUTO(1);
						
						vTaskDelay(pdMS_TO_TICKS(100));
						Dribble_Challenge = DRIB_PATH5;
						Dribble_State = MOTOR_SPEED_UP;
						flag_second_drib = 0;
					}
					if(Dribble_State == DRIBBLE_LOSE && flag_second_drib == 1)//第二次运球失误，进入重启区，接到球后自动到下一个点
					{
						Dribble_Challenge = DRIB_RESET4_2;
						Dribble_State = MOTOR_SPEED_UP;
						Drib_Reset = RESET_GO;
						Drib_Chal_Record = DRIB_PATH5;
						flag_second_drib = 0;
					}
				}
				break;
			
			case DRIB_PATH5:
				if(path_state == PATH_END || path_state == PATH_INIT)
				{
					Dribble();//在出发点运球,运球结束之后  
					if(Dribble_State == DRIBBLE_VIC && flag_second_drib == 0 && switch_on_2 == 1)//第一次运球成功，下一次运球
					{
						vTaskDelay(pdMS_TO_TICKS(1000));
						Dribble_State = MOTOR_SPEED_UP;
						flag_second_drib++;
					}
					if(Dribble_State == DRIBBLE_LOSE && flag_second_drib == 0)//第一次运球失误，进入重启区，接到球后自动回来继续运球
					{
						Dribble_Challenge = DRIB_RESET5_1;
						Dribble_State = MOTOR_SPEED_UP;
						Drib_Reset = RESET_GO;
						Drib_Chal_Record = DRIB_PATH5;
						flag_second_drib++;
					}
					if(Dribble_State == DRIBBLE_VIC && flag_second_drib == 1)//第二次运球成功，到下一个点
					{
						point_end.x = -4600;
						point_end.y = 500;
						point_end.q = 0;
						SET_NAV_PATH_AUTO(1);
						
						vTaskDelay(pdMS_TO_TICKS(100));
						Dribble_Challenge = DRIB_PATH6;
						Dribble_State = MOTOR_SPEED_UP;
						flag_second_drib = 0;
					}
					if(Dribble_State == DRIBBLE_LOSE && flag_second_drib == 1)//第二次运球失误，进入重启区，接到球后自动到下一个点
					{
						Dribble_Challenge = DRIB_RESET5_2;
						Dribble_State = MOTOR_SPEED_UP;
						Drib_Reset = RESET_GO;
						Drib_Chal_Record = DRIB_PATH6;
						flag_second_drib = 0;
					}
				}
				break;
			
			case DRIB_PATH6:
				if(path_state == PATH_END || path_state == PATH_INIT)
				{
					Dribble();//在出发点运球,运球结束之后  
					if(Dribble_State == DRIBBLE_VIC && flag_second_drib == 0 && switch_on_2 == 1)//第一次运球成功，下一次运球
					{
						vTaskDelay(pdMS_TO_TICKS(1000));
						Dribble_State = MOTOR_SPEED_UP;
						flag_second_drib++;
					}
					if(Dribble_State == DRIBBLE_LOSE && flag_second_drib == 0)//第一次运球失误，进入重启区，接到球后自动回来继续运球
					{
						Dribble_Challenge = DRIB_RESET6_1;
						Dribble_State = MOTOR_SPEED_UP;
						Drib_Reset = RESET_GO;
						Drib_Chal_Record = DRIB_PATH6;
						flag_second_drib++;
					}
					if(Dribble_State == DRIBBLE_VIC && flag_second_drib == 1)//第二次运球成功，到下一个点
					{
						point_end.x = -3900;
						point_end.y = 200;
						point_end.q = 0;
						SET_NAV_PATH_AUTO(1);
						
						vTaskDelay(pdMS_TO_TICKS(100));
						Dribble_Challenge = DRIB_PATH7;
						Dribble_State = MOTOR_SPEED_UP;
						flag_second_drib = 0;
					}
					if(Dribble_State == DRIBBLE_LOSE && flag_second_drib == 1)//第二次运球失误，进入重启区，接到球后自动到下一个点
					{
						Dribble_Challenge = DRIB_RESET6_2;
						Dribble_State = MOTOR_SPEED_UP;
						Drib_Reset = RESET_GO;
						Drib_Chal_Record = DRIB_PATH7;
						flag_second_drib = 0;
					}
				}
				break;
			
			case DRIB_PATH7:
				if(path_state == PATH_END || path_state == PATH_INIT)
				{
					Dribble();//在出发点运球,运球结束之后  
					if(Dribble_State == DRIBBLE_VIC && flag_second_drib == 0 && switch_on_2 == 1)//第一次运球成功，下一次运球
					{
						vTaskDelay(pdMS_TO_TICKS(1000));
						Dribble_State = MOTOR_SPEED_UP;
						flag_second_drib++;
					}
					if(Dribble_State == DRIBBLE_LOSE && flag_second_drib == 0)//第一次运球失误，进入重启区，接到球后自动回来继续运球
					{
						Dribble_Challenge = DRIB_RESET7;
						Dribble_State = MOTOR_SPEED_UP;
						Drib_Reset = RESET_GO;
						Drib_Chal_Record = DRIB_PATH7;
						flag_second_drib++;
					}
					if((Dribble_State == DRIBBLE_VIC||Dribble_State == DRIBBLE_LOSE)&& flag_second_drib == 1)//第十四次运球，无论运成还是失败都直接到终点
					{
						point_end.x = -1400;
						point_end.y = 0;//-300;
						point_end.q = 0;
						SET_NAV_PATH_AUTO(1);
						
						vTaskDelay(pdMS_TO_TICKS(100));
						
						//全部拍完，所有状态归位
						DribChal_Task_State = LOCKED;//任务锁上
						Dribble_Challenge = DRIB_EXIT;//运球挑战赛初始化
						
						Dribble_State = MOTOR_SPEED_UP;//运球过程初始化
						flag_second_drib = 0;//运球次数归0
					}
				}
				break;
			
			case DRIB_RESET1_1:
				if(Drib_Reset == RESET_GO)
				{
					point_end.x = 0;
					point_end.y = 0;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);//会把path_state置为ON_GOING，所以两个if不会同时进入
					Drib_Reset = RESET_BACK;
				}
				if(path_state == PATH_END && Drib_Reset == RESET_BACK && switch_on_2 == 1)
				{
					vTaskDelay(pdMS_TO_TICKS(100));
					point_end.x = -1400;
					point_end.y = 0;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);
					Dribble_Challenge = Drib_Chal_Record;
				}
				break;
				
			case DRIB_RESET1_2:
				if(Drib_Reset == RESET_GO)
				{
					point_end.x = 0;
					point_end.y = 0;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);//会把path_state置为ON_GOING，所以两个if不会同时进入
					Drib_Reset = RESET_BACK;
				}
				if(path_state == PATH_END && Drib_Reset == RESET_BACK && switch_on_2 == 1)
				{
					vTaskDelay(pdMS_TO_TICKS(100));
					point_end.x = -700;
					point_end.y = 700;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);
					Dribble_Challenge = Drib_Chal_Record;
				}
				break;
				
			case DRIB_RESET2_1:
				if(Drib_Reset == RESET_GO)
				{
					point_end.x = -200;
					point_end.y = 700;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);//会把path_state置为ON_GOING，所以两个if不会同时进入
					Drib_Reset = RESET_BACK;
				}
				if(path_state == PATH_END && Drib_Reset == RESET_BACK && switch_on_2 == 1)
				{
					vTaskDelay(pdMS_TO_TICKS(100));
					point_end.x = -700;
					point_end.y = 700;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);
					Dribble_Challenge = Drib_Chal_Record;
				}
				break;
				
			case DRIB_RESET2_2:
				if(Drib_Reset == RESET_GO)
				{
					point_end.x = -200;
					point_end.y = 700;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);//会把path_state置为ON_GOING，所以两个if不会同时进入
					Drib_Reset = RESET_BACK;
				}
				if(path_state == PATH_END && Drib_Reset == RESET_BACK && switch_on_2 == 1)
				{
					vTaskDelay(pdMS_TO_TICKS(100));
					point_end.x = -700;
					point_end.y = 2400;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);
					Dribble_Challenge = Drib_Chal_Record;
				}
				break;
				
			case DRIB_RESET3_1:
				if(Drib_Reset == RESET_GO)
				{
					point_end.x = -200;
					point_end.y = 2400;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);//会把path_state置为ON_GOING，所以两个if不会同时进入
					Drib_Reset = RESET_BACK;
				}
				if(path_state == PATH_END && Drib_Reset == RESET_BACK && switch_on_2 == 1)
				{
					vTaskDelay(pdMS_TO_TICKS(100));
					point_end.x = -700;
					point_end.y = 2400;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);
					Dribble_Challenge = Drib_Chal_Record;
				}
				break;
				
			case DRIB_RESET3_2:
				if(Drib_Reset == RESET_GO)
				{
					point_end.x = -200;
					point_end.y = 2400;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);//会把path_state置为ON_GOING，所以两个if不会同时进入
					Drib_Reset = RESET_BACK;
				}
				if(path_state == PATH_END && Drib_Reset == RESET_BACK && switch_on_2 == 1)
				{
					vTaskDelay(pdMS_TO_TICKS(100));
					point_end.x = -2600;
					point_end.y = 2400;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);
					Dribble_Challenge = Drib_Chal_Record;
				}
				break;
				
			case DRIB_RESET4_1:
				if(Drib_Reset == RESET_GO)
				{
					point_end.x = -2600;
					point_end.y = 2900;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);//会把path_state置为ON_GOING，所以两个if不会同时进入
					Drib_Reset = RESET_BACK;
				}
				if(path_state == PATH_END && Drib_Reset == RESET_BACK && switch_on_2 == 1)
				{
					vTaskDelay(pdMS_TO_TICKS(100));
					point_end.x = -2600;
					point_end.y = 2400;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);
					Dribble_Challenge = Drib_Chal_Record;
				}
				break;
				
			case DRIB_RESET4_2:
				if(Drib_Reset == RESET_GO)
				{
					point_end.x = -2600;
					point_end.y = 2900;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);//会把path_state置为ON_GOING，所以两个if不会同时进入
					Drib_Reset = RESET_BACK;
				}
				if(path_state == PATH_END && Drib_Reset == RESET_BACK && switch_on_2 == 1)
				{
					vTaskDelay(pdMS_TO_TICKS(100));
					point_end.x = -4500;
					point_end.y = 2400;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);
					Dribble_Challenge = Drib_Chal_Record;
				}
				break;
				
			case DRIB_RESET5_1:
				if(Drib_Reset == RESET_GO)
				{
					point_end.x = -5000;
					point_end.y = 2400;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);//会把path_state置为ON_GOING，所以两个if不会同时进入
					Drib_Reset = RESET_BACK;
				}
				if(path_state == PATH_END && Drib_Reset == RESET_BACK && switch_on_2 == 1)
				{
					vTaskDelay(pdMS_TO_TICKS(100));
					point_end.x = -4500;
					point_end.y = 2400;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);
					Dribble_Challenge = Drib_Chal_Record;
				}
				break;
				
			case DRIB_RESET5_2:
				if(Drib_Reset == RESET_GO)
				{
					point_end.x = -5000;
					point_end.y = 2400;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);//会把path_state置为ON_GOING，所以两个if不会同时进入
					Drib_Reset = RESET_BACK;
				}
				if(path_state == PATH_END && Drib_Reset == RESET_BACK && switch_on_2 == 1)
				{
					vTaskDelay(pdMS_TO_TICKS(100));
					point_end.x = -4600;
					point_end.y = 500;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);
					Dribble_Challenge = Drib_Chal_Record;
				}
				break;
				
			case DRIB_RESET6_1:
				if(Drib_Reset == RESET_GO)
				{
					point_end.x = -5300;
					point_end.y = 500;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);//会把path_state置为ON_GOING，所以两个if不会同时进入
					Drib_Reset = RESET_BACK;
				}
				if(path_state == PATH_END && Drib_Reset == RESET_BACK && switch_on_2 == 1)
				{
					vTaskDelay(pdMS_TO_TICKS(100));
					point_end.x = -4600;
					point_end.y = 500;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);
					Dribble_Challenge = Drib_Chal_Record;
				}
				break;
				
			case DRIB_RESET6_2:
				if(Drib_Reset == RESET_GO)
				{
					point_end.x = -5300;
					point_end.y = 500;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);//会把path_state置为ON_GOING，所以两个if不会同时进入
					Drib_Reset = RESET_BACK;
				}
				if(path_state == PATH_END && Drib_Reset == RESET_BACK && switch_on_2 == 1)
				{
					vTaskDelay(pdMS_TO_TICKS(100));
					point_end.x = -3900;
					point_end.y = -200;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);
					Dribble_Challenge = Drib_Chal_Record;
				}
				break;
				
			case DRIB_RESET7:
				if(Drib_Reset == RESET_GO)
				{
					point_end.x = -5300;
					point_end.y = -200;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);//会把path_state置为ON_GOING，所以两个if不会同时进入
					Drib_Reset = RESET_BACK;
				}
				if(path_state == PATH_END && Drib_Reset == RESET_BACK && switch_on_2 == 1)
				{
					vTaskDelay(pdMS_TO_TICKS(100));
					point_end.x = -3900;
					point_end.y = -200;
					point_end.q = 0;
					SET_NAV_PATH_AUTO(1);
					Dribble_Challenge = Drib_Chal_Record;
				}
				break;
				
			default:
				break;
		}
	}
}

