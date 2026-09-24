#include "Nav_Area_3.h"



void Nav_Area_3_Task(void)//竞技赛的正常上三区
{
//        chassis_run.pid_state = VELT_LOOP;
//		chassis_run.feed_forward_state = WITHOUT_FORWARD;

		if (get_fb == 0)
		{			
			Q = point_end.q;//记录上坡前目标角度，后续转正
			get_fb = 1;
		}
		
		
		switch (state3)
		{
		case 0:
			if (uphill == 0)
			{
				Nav_Uphill(&nav, &speed);
			}
			break;
		case 1:
			Nav_Rotation(&nav, Q);
			break;
		case 2:
			uart1_tx_buffer[3]=1;
			point_end.x = -100;
			point_end.y = 1532;
			point_end.q = Q;
			NavLineMoveWithHeading(&nav);
			break;
		case 3://放第一个下层方块
			
		  switch(Region3_Spot)
			{
				case 1:		
			point_end.x = -314+50+20;
			point_end.y = 4553-200; //放块
			point_end.q = Q;
			NavLineMoveWithHeading(&nav);
			break;
				case 2:		
			point_end.x = -805;
			point_end.y = 4553-200; //放块
			point_end.q = Q;
			NavLineMoveWithHeading(&nav);
			break;											
				case 3:		
			point_end.x = -1394+50+20;
			point_end.y = 4553-200; //放块
			point_end.q = Q;
			NavLineMoveWithHeading(&nav);
			break;
				
				default:
         break;
			}
			
			break;
				case 4://躲到一边去等R2过来
			point_end.x = -805;
			point_end.y = 3553-200; 
			point_end.q = Q-90;
			NavLineMoveWithHeading(&nav);				
        break;

				
				case 5://取地下方块
			point_end.x = -605;
			point_end.y = 3553-200; 
			point_end.q = Q-90;
			NavLineMoveWithHeading(&nav);									
					break;
			
			
			
		}

		if (uphilling == 0)
		{ 
			Nav_PID_Adjust();
			
			PID_Calc_New(&nav.auto_path.pos_pid.x);
			PID_Calc_New(&nav.auto_path.pos_pid.y);
			PID_Calc_New(&nav.auto_path.pos_pid.w);
    
		nav.expect_robot_global_velt.fpX = (float)0.3*nav.auto_path.pos_pid.x.fpU + (float)0.7*nav.auto_path.basic_velt.fpVx;
		nav.expect_robot_global_velt.fpY = (float)0.3*nav.auto_path.pos_pid.y.fpU + (float)0.7*nav.auto_path.basic_velt.fpVy;
		nav.expect_robot_global_velt.fpW = (float)0.3*nav.auto_path.pos_pid.w.fpU + (float)0.7*nav.auto_path.basic_velt.fpW;
		}
		else//也就是正在上坡
		{ 
			PID_Calc_New(&nav.auto_path.pos_pid.w);


      

			
			nav.expect_robot_global_velt.fpX = nav.auto_path.basic_velt.fpVx;
			nav.expect_robot_global_velt.fpY = nav.auto_path.basic_velt.fpVy;
			nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU + nav.auto_path.basic_velt.fpW;
			

		}
		
		SpeedDistribute_Four_OmnidriectionalWhile(&nav);
    }




    void Nav_Area_3_Reset_Task(void)//在三区上的重试（用于竞技赛） 
		{			
			
			
			
/******************************************************************************竞技赛重试***********************************************/			
//        chassis_run.pid_state = VELT_LOOP;
//		chassis_run.feed_forward_state = WITHOUT_FORWARD;

		switch (state3_RESET)//三区重试的状态，路径可以再次修改，因为没有必要再刷新初始位置了
		{
		case 0:
			point_end.x = -100;
			point_end.y = 1532;
			point_end.q = 0;
			NavLineMoveWithHeading(&nav);
			break;
		case 1:
			
		
		  switch(Region3_Spot)
			{
				case 1:		
			point_end.x = -314+70;
			point_end.y = 4553-200; //放块
			point_end.q = 90;
			NavLineMoveWithHeading(&nav);
			break;
				
				case 2:		
			point_end.x = -854+70;
			point_end.y = 4553-200; //放块
			point_end.q = 90;
			NavLineMoveWithHeading(&nav);
			break;
				
				case 3:		
			point_end.x = -1394+70;
			point_end.y = 4553-200; //放块
			point_end.q = 90;
			NavLineMoveWithHeading(&nav);
			break;
				
				default:
         break;
			}					
	   break;
				
		default:
			break;
		}
	    Nav_PID_Adjust();

			PID_Calc_New(&nav.auto_path.pos_pid.x);
			PID_Calc_New(&nav.auto_path.pos_pid.y);
			PID_Calc_New(&nav.auto_path.pos_pid.w);

		nav.expect_robot_global_velt.fpX = (float)0.3*nav.auto_path.pos_pid.x.fpU + (float)0.7*nav.auto_path.basic_velt.fpVx;
		nav.expect_robot_global_velt.fpY = (float)0.3*nav.auto_path.pos_pid.y.fpU + (float)0.7*nav.auto_path.basic_velt.fpVy;
		nav.expect_robot_global_velt.fpW = (float)0.3*nav.auto_path.pos_pid.w.fpU + (float)0.7*nav.auto_path.basic_velt.fpW;

		
		
		SpeedDistribute_Four_OmnidriectionalWhile(&nav);

	}
/******************************************************************************竞技赛重试***********************************************/		











void Nav_Area_3_SINGLE_Task(void)//单项赛//和正常三区逻辑很相似
{
/******************************************************************************单项赛正常上坡***********************************************/
//    chassis_run.pid_state = VELT_LOOP;
//		chassis_run.feed_forward_state = WITHOUT_FORWARD;
//	
	   Q=90;
		
		
		switch (state3_SINGLE)
		{
		case 0:
			if (uphill == 0)
			{
				Nav_Uphill(&nav, &speed);
			}
			break;
		case 1:
			Nav_Rotation(&nav, Q);
			break;
		case 2:
			uart1_tx_buffer[3]=1;
			point_end.x = -100;
			point_end.y = 1532;
			point_end.q = Q;
			NavLineMoveWithHeading(&nav);
			break;
		case 3://放第一个下层方块
			
		  switch(Region3_Spot)
			{
				case 1:		
			point_end.x = -314+50+20;
			point_end.y = 4553-200; //放块
			point_end.q = Q;
			NavLineMoveWithHeading(&nav);
			break;
				case 2:		
			point_end.x = -805;
			point_end.y = 4553-200; //放块
			point_end.q = Q;
			NavLineMoveWithHeading(&nav);
			break;											
				case 3:		
			point_end.x = -1394+50+20;
			point_end.y = 4553-200; //放块
			point_end.q = Q;
			NavLineMoveWithHeading(&nav);
			break;
				
				default:
         break;
			}
			
			break;
				case 4://躲到一边去等R2过来
			point_end.x = -1394+50+20;
			point_end.y = 4553-200; 
			point_end.q = Q-90;
			NavLineMoveWithHeading(&nav);				
        break;

				
				case 5://取地下方块
			point_end.x = -1394+50+20;
			point_end.y = 4553-200; 
			point_end.q = Q-90;
			NavLineMoveWithHeading(&nav);									
					break;
			
			
			
		}

		if (uphilling == 0)
		{ 
			Nav_PID_Adjust();
			
			PID_Calc_New(&nav.auto_path.pos_pid.x);
			PID_Calc_New(&nav.auto_path.pos_pid.y);
			PID_Calc_New(&nav.auto_path.pos_pid.w);
    
		nav.expect_robot_global_velt.fpX = (float)0.3*nav.auto_path.pos_pid.x.fpU + (float)0.7*nav.auto_path.basic_velt.fpVx;
		nav.expect_robot_global_velt.fpY = (float)0.3*nav.auto_path.pos_pid.y.fpU + (float)0.7*nav.auto_path.basic_velt.fpVy;
		nav.expect_robot_global_velt.fpW = (float)0.3*nav.auto_path.pos_pid.w.fpU + (float)0.7*nav.auto_path.basic_velt.fpW;
		}
		else//也就是正在上坡
		{ 
			PID_Calc_New(&nav.auto_path.pos_pid.w);


      

			
			nav.expect_robot_global_velt.fpX = nav.auto_path.basic_velt.fpVx;
			nav.expect_robot_global_velt.fpY = nav.auto_path.basic_velt.fpVy;
			nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU + nav.auto_path.basic_velt.fpW;
			
      //nav.expect_robot_global_velt.fpW = 0;
			//nav.expect_robot_global_velt.fpW = nav.auto_path.basic_velt.fpW;
		}
		
		SpeedDistribute_Four_OmnidriectionalWhile(&nav);



/******************************************************************************单项赛正常上坡***********************************************/		

    }



    
