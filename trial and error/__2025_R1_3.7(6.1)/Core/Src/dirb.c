#include "drib.h"
/**********************************************************************************************************************************************************
版权声明：HITCRT(哈工大竞技机器人协会)
文件名：dribble_task.c
最近修改日期：2025.2.15
版本：1.0
----------------------------------------------------------------------------------------------------------------------------------------------------------
模块描述：
函数列表：

----------------------------------------------------------------------------------------------------------------------------------------------------------
修订记录：
	 作者        	时间            版本     	说明
	 XSH       2025.2.19        	1.1      
**********************************************************************************************************************************************************/

/*********************************************************************************************************
函数名称：void Dribble(void)
函数功能：自动执行整个运球的过程
备注:
		只要一调用便会自动进行运球过程，因此不要直接写到freertos里，要运球时调用即可
		
Tip：	用前用后要将Dribble_State复位，否则可能不会执行，
			复位语句不要和Dribble()放一块，否则会一直在MOTOR_SPEED_UP里面！！！！！！！！！！！
		
		0.MOTOR_SPEED_UP：让摩擦轮加速，把夹球夹上
		1.CYLINDER_LOOSE：直到加速完成再进行，松开夹球气缸，让球落下
		2.MOTOR_REVERSE：当摩擦轮大幅减速时，也就是球碰到摩擦轮了，隔100ms让球顺利落下后让摩擦轮反转
		3.MOTOR_READY:电机反转加速完再切换状态。
				（此状态是为了分开2和4，放到MOTOR_REVERSE里会由于电机速度而多次触发延时，放到CYLINDER_CLAMP里两条判断语句又会相互冲突，
					不判断电机速度是否达到也会出问题）
		4.CYLINDER_CLAMP:当摩擦轮反转也大幅减速时，也就是球弹起到摩擦轮了，隔100ms夹紧气缸，夹住篮球，
											再让摩擦轮停下，状态机结束
		5.DRIBBLE_VIC：让系统知道运球结束了，可以进行其他的动作了
		
		综旨：有些指令位的语句最好只进行一次就进行状态机切换，防止多次触发某条语句（例如延时语句，不可多次触发）

**********************************************************************************************************/ 
//u8 start_from_drib;
//uint32_t TimeDrib;
//float Drib_Left_Inc= 4800;//5200;//7000;//5000;//7000;//5000;
//float Drib_Right_Inc= -4800;//-6600;//-4800;//-3800;//-6600;//-5000;
//float Drib_Left_Rev= -7500;//-4000;//-2000;//-5000;//-4000;//-5000;
//float Drib_Right_Rev= 7150;//3900;//1900;//4700;//3800;//4800;
//float time_delay = 200;
//void Dribble(void)//整个运球过程
//{
//	switch(Dribble_State)
//	{
//		case MOTOR_SPEED_UP:
//			
//			Drib_Left_Targetv = Drib_Left_Inc;
//			Drib_Right_Targetv = Drib_Right_Inc;
//			Drib_Left_Pid.fpSumE = 0;
//			Drib_Right_Pid.fpSumE = 0;
//			Rec_Ball_State = REC_END;//运球开始了，就必定是接球成功了，这里锁上接球是为了运球的时候不会被接球状态位影响舵机角度。
//			CLOSE_VALVE_Plus(1);//保证开始运球时夹球要夹上

//			Dribble_State = CYLINDER_LOOSE;
//			break;
//			
//		case CYLINDER_LOOSE:
//			if(start_from_drib == 0)
//			{
//				TimeDrib = Current_Time;
//				start_from_drib = 1;
//			}
//			if((Current_Time - TimeDrib)< 1200)
//			{
//				if(fabs(Drib_Motor_Left.anglev-Drib_Left_Inc)<100 && fabs(Drib_Motor_Right.anglev-Drib_Right_Inc)<100 && (Current_Time - TimeDrib)> 100)
//				{
//					OPEN_VALVE_Plus(1);//夹球气缸松开，让球落到摩擦轮上
//			
//					Dribble_State = MOTOR_REVERSE;
//					start_from_drib = 0;
//				}
//			}
//			else //电机被球卡住了，松开球，运球失败
//			{
//				OPEN_VALVE_Plus(1);//松开篮球
//				Drib_Left_Targetv = 1000;
//				Drib_Right_Targetv = -1000;
//				
//				vTaskDelay(pdMS_TO_TICKS(1000));//延1s，让球下来
//				
//				Dribble_State = DRIBBLE_STICK;
//				start_from_drib = 0;
//			}
//			break;
//		
//		case MOTOR_REVERSE:
//			if((fabs(Drib_Motor_Left.anglev-Drib_Left_Inc)>400 || fabs(Drib_Motor_Right.anglev-Drib_Right_Inc)>400)&&switch_on_2 == 0)
//			{
//				vTaskDelay(pdMS_TO_TICKS(50));
//				Drib_Left_Targetv = Drib_Left_Rev;
//				Drib_Right_Targetv = Drib_Right_Rev;
//				
//				J60_target_pos = 70;
//				
//				Dribble_State = MOTOR_READY;
//			}
//			break;
//			
//		case MOTOR_READY:
//			if(fabs(Drib_Motor_Left.anglev-Drib_Left_Rev)<3500 && fabs(Drib_Motor_Right.anglev-Drib_Right_Rev)<3500)
//			{
//				Dribble_State = CYLINDER_CLAMP;
//			}
//			break;
//			
//			
//		case CYLINDER_CLAMP:
//			if(start_from_drib == 0)
//			{
//				TimeDrib = Current_Time;
//				start_from_drib = 1;
//			}
//			if((Current_Time - TimeDrib)< 1400)
//			{
//				if(switch_on_2==1) //&& ((Drib_Left_Pid.fpE-Drib_Left_Pid.fpPreE)> 0 ) && ((Drib_Right_Pid.fpE-Drib_Right_Pid.fpPreE)< 0 ))//开关检测到后，并且两个电机都处于加速状态，说明阻力没了，球上去了，此时闭上夹爪
//				{
//					vTaskDelay(pdMS_TO_TICKS(time_delay));
//					CLOSE_VALVE_Plus(1);//夹球气缸夹住篮球
//					Drib_Left_Targetv = 0;
//					Drib_Right_Targetv = 0;
//					
//					J60_target_pos = 80;
//					
//					Dribble_State = DRIBBLE_VIC;
//					start_from_drib = 0;
//				}
//			}
//			else //运球失败了
//			{
//				CLOSE_VALVE_Plus(1);//夹球气缸合上
//				Drib_Left_Targetv = 0;
//				Drib_Right_Targetv = 0;
//				
//				J60_target_pos = 80;
//				
//				Dribble_State = DRIBBLE_LOSE;
//				start_from_drib = 0;
//			}
//			break;
//				
//		case DRIBBLE_VIC:
//			break;
//		
//		case DRIBBLE_STICK:
//			CLOSE_VALVE_Plus(1);//夹球气缸夹住篮球
//			Drib_Left_Targetv = 0;
//			Drib_Right_Targetv = 0;
//		
//			J60_target_pos = 80;
//			
//			Dribble_State = DRIBBLE_LOSE;
//			break;
//		
//		case DRIBBLE_LOSE:
//			break;
//		
//		default:
//			break;
//	}
//}
u8 start_from_drib;
uint32_t TimeDrib;
float Drib_Left_Inc= 5200;//5200;//7000;//5000;//7000;//5000;
float Drib_Right_Inc= -5200;//-6600;//-4800;//-3800;//-6600;//-5000;
float Drib_Left_Rev= -7500;//-4000;//-2000;//-5000;//-4000;//-5000;
float Drib_Right_Rev= 7150;//3900;//1900;//4700;//3800;//4800;
float time_delay = 100;
void Dribble(void)//整个运球过程
{
	switch(Dribble_State)
	{
		case MOTOR_SPEED_UP:
			
			Drib_Left_Targetv = Drib_Left_Inc;
			Drib_Right_Targetv = Drib_Right_Inc;
			Drib_Left_Pid.fpSumE = 0;
			Drib_Right_Pid.fpSumE = 0;
			Rec_Ball_State = REC_END;//运球开始了，就必定是接球成功了，这里锁上接球是为了运球的时候不会被接球状态位影响舵机角度。
			CLOSE_VALVE_Plus(1);//保证开始运球时夹球要夹上

			Dribble_State = CYLINDER_LOOSE;
			break;
			
		case CYLINDER_LOOSE:
			if(start_from_drib == 0)
			{
				TimeDrib = Current_Time;
				start_from_drib = 1;
			}
			if((Current_Time - TimeDrib)< 1200)
			{
				if(fabs(Drib_Motor_Left.anglev-Drib_Left_Inc)<100 && fabs(Drib_Motor_Right.anglev-Drib_Right_Inc)<100 && (Current_Time - TimeDrib)> 100)
				{
					OPEN_VALVE_Plus(1);//夹球气缸松开，让球落到摩擦轮上
			
					Dribble_State = MOTOR_REVERSE;
					start_from_drib = 0;
				}
			}
			else //电机被球卡住了，松开球，运球失败
			{
				OPEN_VALVE_Plus(1);//松开篮球
				Drib_Left_Targetv = 1000;
				Drib_Right_Targetv = -1000;
				
				vTaskDelay(pdMS_TO_TICKS(1000));//延1s，让球下来
				
				Dribble_State = DRIBBLE_STICK;
				start_from_drib = 0;
			}
			break;
		
		case MOTOR_REVERSE:
			if((fabs(Drib_Motor_Left.anglev-Drib_Left_Inc)>400 || fabs(Drib_Motor_Right.anglev-Drib_Right_Inc)>400)&&switch_on_2 == 0)
			{
				vTaskDelay(pdMS_TO_TICKS(50));
				Drib_Left_Targetv = Drib_Left_Rev;
				Drib_Right_Targetv = Drib_Right_Rev;
				
				J60_target_pos = 70;
				
				Dribble_State = MOTOR_READY;
			}
			break;
			
		case MOTOR_READY:
			if(fabs(Drib_Motor_Left.anglev-Drib_Left_Rev)<3500 && fabs(Drib_Motor_Right.anglev-Drib_Right_Rev)<3500)
			{
				Dribble_State = CYLINDER_CLAMP;
			}
			break;
			
			
		case CYLINDER_CLAMP:
			if(start_from_drib == 0)
			{
				TimeDrib = Current_Time;
				start_from_drib = 1;
			}
			if((Current_Time - TimeDrib)< 1400)
			{
				if(switch_on_2 == 1)//((Drib_Left_Pid.fpE-Drib_Left_Pid.fpPreE)> 0 ) && ((Drib_Right_Pid.fpE-Drib_Right_Pid.fpPreE)< 0 ))//开关检测到后，并且两个电机都处于加速状态，说明阻力没了，球上去了，此时闭上夹爪
				{
					vTaskDelay(pdMS_TO_TICKS(time_delay));
					CLOSE_VALVE_Plus(1);//夹球气缸夹住篮球
					Drib_Left_Targetv = 0;
					Drib_Right_Targetv = 0;
					
					J60_target_pos = 80;
					
					Dribble_State = DRIBBLE_VIC;
					start_from_drib = 0;
				}
			}
			else //运球失败了
			{
				CLOSE_VALVE_Plus(1);//夹球气缸合上
				Drib_Left_Targetv = 0;
				Drib_Right_Targetv = 0;
				
				J60_target_pos = 80;
				
				Dribble_State = DRIBBLE_LOSE;
				start_from_drib = 0;
			}
			break;
				
		case DRIBBLE_VIC:
			break;
		
		case DRIBBLE_STICK:
			CLOSE_VALVE_Plus(1);//夹球气缸夹住篮球
			Drib_Left_Targetv = 0;
			Drib_Right_Targetv = 0;
		
			J60_target_pos = 80;
			
			Dribble_State = DRIBBLE_LOSE;
			break;
		
		case DRIBBLE_LOSE:
			break;
		
		default:
			break;
	}
}
/*********************************************************************************************************
函数名称：void Load_ball(void)
函数功能：把篮球放到抽屉
备注:
		只要一调用便会自动进行放球过程，因此不要直接写到freertos里，要放球时调用即可

Tip：	用前用后要将Load_State复位，否则可能不会执行，
			复位语句不要和Load_Basketball()放一块，否则会一直在第一句里面！！！！！！！！！！！

		0：发送信息，电机加速，降下升降
		1：电机加速完毕且上板准备完毕之后松开夹球
		2：摩擦轮检测到球落下后，延时一段让球落下
**********************************************************************************************************/ 
void Load_ball(void)//把球放置到抽屉里
{
	switch(Load_State)
	{
		case 0:
			uart1_eft.num[3] = 1;//通知上板要进行接球了
		
			Drib_Left_Targetv = Load_Left;
			Drib_Right_Targetv = Load_Right;
			
			J60_target_pos = 30;
		
			Load_State++;
			break;
		
		case 1:
			if(fabs(Drib_Motor_Left.anglev-Load_Left)<200 && fabs(Drib_Motor_Right.anglev-Load_Right)<200 && fabs(J60_target_pos - J60_angle) < 5)//&& uart1_efr.num[0] == 1 && fabs(J60_target_pos - J60_angle) < 5)//摩擦轮加速完毕，J60下来了，并且抽屉已经到达指定位置
			{
				OPEN_VALVE_Plus(1);
				
				Load_State++;
			}
			break;
			
		case 2:
			if(fabs(Drib_Motor_Left.anglev-Load_Left)>300 || fabs(Drib_Motor_Right.anglev-Load_Right)>300)
			{
				vTaskDelay(pdMS_TO_TICKS(1000));
				Drib_Left_Targetv = 0;
				Drib_Right_Targetv = 0;
				
				CLOSE_VALVE_Plus(1);
				
				uart1_eft.num[3] = 2;//表示球已经掉落到抽屉里，放球过程已完毕
				
				Load_State++;
			}
			break;
			
		case LOAD_END:
			break;
		default :
			break;
	}
}

/*********************************************************************************************************
函数名称：void Dribble_Load(void)
函数功能：完成一次运球加放球
备注:
		初始状态INIT不会动，直接放freertos里，要开始的话修改Drib_Load=DRIB_INIT即可
		
		0.DRIB_LOAD_INIT：空状态，放在freertos里面随时准备调用
		1.DRIB_INIT：给运球复位，给放球标志位复位，跳到下一个状态，放这里是为了防止运球复位和Dribble()耦合
		2.DRIB：运球过程，运球结束时，给放球状态初始化，同时进入放球状态
		3.LOAD：放球过程，放球结束后，运球加放球过程结束
		4.DRIB_LOAD_END：空状态
		
		到达DRIB_LOAD_END之后运球加放球结束，再想开始运球加放球，还是修改Drib_Load=DRIB_INIT即可
**********************************************************************************************************/
void Dribble_Load(void)
{
	switch(Drib_Load)
	{
		case DRIB_LOAD_INIT:
			break;
		case DRIB_INIT:
			Dribble_State = MOTOR_SPEED_UP;
			uart1_eft.num[3] = 0;//进入装球状态后需要给置0否则会直接向上板表示放球完毕
			Drib_Load = DRIB;
			break;
		case DRIB:
			Dribble();
			if(Dribble_State == DRIBBLE_VIC)
			{
				Load_State = 0;//给放球状态初始化
				Drib_Load = LOAD;
			}
			break;
		case LOAD:
			Load_ball();
			if(Load_State == LOAD_END)
			{
				Drib_Load = DRIB_LOAD_END;
			}
			break;
		case DRIB_LOAD_END:
			break;
		default:
			break;
	}
}

