#include "path.h"



void path_point_choose(ST_Nav *p_nav)
{
	Path_Point.point[0].fpX = robot_pos.fpPosX; // mm
	Path_Point.point[0].fpY = robot_pos.fpPosY; // mm
	Path_Point.point[0].fpW = robot_pos.fpPosQ; // rad
	Path_Point.velt[0].fpX = 0;
	Path_Point.velt[0].fpY = 0;
	Path_Point.velt[0].fpW = 0;

	switch (p_nav->auto_path.number_point)
	{

	case 7://对接
		Path_Point.velt[0].fpX = -500;
		Path_Point.velt[0].fpY = 0;
		Path_Point.velt[0].fpW = 0;	
	
		Path_Point.point[1].fpX = DOCK_X;
		Path_Point.point[1].fpY = DOCK_Y;
		Path_Point.point[1].fpW = DOCK_ANGLE;
		Path_Point.velt[1].fpX = 0;
		Path_Point.velt[1].fpY = 0;
		Path_Point.velt[1].fpW = 0;
		Path_Point.time[0] = 1000;  //ms
		Path_Point.point_num = 1;				
		break;
	
	case 8://让路
		Path_Point.velt[0].fpX = 0;
		Path_Point.velt[0].fpY = 0;
		Path_Point.velt[0].fpW = 0;	
	
		Path_Point.point[1].fpX = DOCK_X + 400;
		Path_Point.point[1].fpY = DOCK_Y;
		Path_Point.point[1].fpW = DOCK_ANGLE;
		Path_Point.velt[1].fpX = 0;
		Path_Point.velt[1].fpY = 0;
		Path_Point.velt[1].fpW = 0;
		Path_Point.time[0] = 700;  //ms		
	
		Path_Point.point[2].fpX = Move_Aside_X;
		Path_Point.point[2].fpY = Move_Aside_Y;
		Path_Point.point[2].fpW = Move_Aside_RAD;
		Path_Point.velt[2].fpX = 0;
		Path_Point.velt[2].fpY = 0;
		Path_Point.velt[2].fpW = 0;
		Path_Point.time[1] = 700;  //ms

		Path_Point.point[3].fpX = start_x;
		Path_Point.point[3].fpY = start_y;
		Path_Point.point[3].fpW = Move_Aside_RAD;
		Path_Point.velt[3].fpX = 0;
		Path_Point.velt[3].fpY = 0;
		Path_Point.velt[3].fpW = 0;
		Path_Point.time[2] = 850;  //ms	
		
		Path_Point.point_num = 3;	
	
	
		break;
	
	
		case 24://从一号出口到三区斜坡入口
		//x -2576 y -7146 w 1.55 起始点
	


		
		Path_Point.velt[0].fpX = 0;
		Path_Point.velt[0].fpY = 0;
		Path_Point.velt[0].fpW = 0;	
	
		Path_Point.point[1].fpX =  MELIN_X_10;//-2863.f;
		Path_Point.point[1].fpY =  nav.auto_path.pos_pid.pid_y.fpFB;//-6755.f;
		Path_Point.point[1].fpW =  MELIN_ANGLE_10;
		Path_Point.velt[1].fpX = -1550;
		Path_Point.velt[1].fpY = 0;
		Path_Point.velt[1].fpW = 0;
		Path_Point.time[0] = 300;  //ms	 cv

		
		Path_Point.point_num = 1;		
		break;
	
	case 25://从二号出口到三区斜坡入口
		
	//qishi x -1377 y-7105 
			
	

	
		Path_Point.velt[0].fpX = 0;
		Path_Point.velt[0].fpY = 0;
		Path_Point.velt[0].fpW = 0;	
	
		
		
		Path_Point.point[1].fpX = MELIN_X_11;//-2863.f;//-2550;//-2616.f;
		Path_Point.point[1].fpY = nav.auto_path.pos_pid.pid_y.fpFB;
		Path_Point.point[1].fpW = MELIN_ANGLE_11;
		Path_Point.velt[1].fpX = -1550.f;
		Path_Point.velt[1].fpY = 0;
		Path_Point.velt[1].fpW = 0;
		Path_Point.time[0] = 450.f;//1050.f;  //ms

		
		Path_Point.point_num = 1;		
		
		break;
	
	case 26://从三号出口到三区斜坡入口
		//x -171 y -7049 

	

		Path_Point.velt[0].fpX = 0;
		Path_Point.velt[0].fpY = 0;
		Path_Point.velt[0].fpW = 0;	


		Path_Point.point[1].fpX =  MELIN_X_11;//-2863.f;//-2616.f;
		Path_Point.point[1].fpY = nav.auto_path.pos_pid.pid_y.fpFB;
		Path_Point.point[1].fpW = MELIN_ANGLE_11;
		Path_Point.velt[1].fpX = -1550.f; 
		Path_Point.velt[1].fpY = 0;
		Path_Point.velt[1].fpW = 0;
		Path_Point.time[0] = 1300.f;  //ms
	

		Path_Point.point_num = 1;	
		break;
	
	
		case 27://从三区启动区到九宫格  重试
		//启动区
		//x -4373 y-10963 -3.14
		Path_Point.velt[0].fpX = 0;
		Path_Point.velt[0].fpY = 0;
		Path_Point.velt[0].fpW = 0;			

		Path_Point.point[1].fpX = -1000.f;//改！！！
		Path_Point.point[1].fpY =	GRID_2_Y;
		Path_Point.point[1].fpW = GRID_RAD;       
		Path_Point.velt[1].fpX = 3000;          
		Path_Point.velt[1].fpY = 0;
		Path_Point.velt[1].fpW = 0;
		Path_Point.time[0] = 1400;              

		Path_Point.point[2].fpX = GRID_2_X;
		Path_Point.point[2].fpY = GRID_2_Y;//右 -9897.f;
		Path_Point.point[2].fpW = GRID_RAD;
		Path_Point.velt[2].fpX = 0;
		Path_Point.velt[2].fpY = 0;
		Path_Point.velt[2].fpW = 0;
		Path_Point.time[1] = 700;              

		Path_Point.point_num = 2;              	
		break;
		
		case 28://从曲线末尾到九宫格
		Path_Point.point[0].fpX = RAMP_CURVE_END_X; // mm
		Path_Point.point[0].fpY = RAMP_CURVE_END_Y; // mm
		
		Path_Point.velt[0].fpX = 1550;
		Path_Point.velt[0].fpY = 0;
		Path_Point.velt[0].fpW = 0;			


	


		Path_Point.point[1].fpX = -1000;//改！！！
		Path_Point.point[1].fpY =  GRID_2_Y ;
		Path_Point.point[1].fpW = GRID_RAD;
		Path_Point.velt[1].fpX = 2900;
		Path_Point.velt[1].fpY = 0 ;
		Path_Point.velt[1].fpW = 0;//1.9;
		Path_Point.time[0] =800 ;
			
			

		Path_Point.point[2].fpX = GRID_2_X;//407.f;
		Path_Point.point[2].fpY = GRID_2_Y ;//右-9897.f;//-9926.f;
		Path_Point.point[2].fpW = GRID_RAD;
		Path_Point.velt[2].fpX = 0;
		Path_Point.velt[2].fpY = 0;
		Path_Point.velt[2].fpW = 0;
		Path_Point.time[1] =  700; 

		Path_Point.point_num = 2; 

		break;
	
	
	
	case 32://DT35大概区域  用于一区
		Path_Point.velt[0].fpX = 0;
		Path_Point.velt[0].fpY = -500;
		Path_Point.velt[0].fpW = 0;	
	
		Path_Point.point[1].fpX = HEAD_AREA_X;
		Path_Point.point[1].fpY = HEAD_AREA_Y ;
		Path_Point.point[1].fpW = HEAD_AREA_RAD;
		Path_Point.velt[1].fpX = 0;
		Path_Point.velt[1].fpY = 0;
		Path_Point.velt[1].fpW = 0;
		Path_Point.time[0] = 1000;  //ms

		Path_Point.point_num = 1;			
		
		break;
		
		case 35://从矛头架回到启动区  恢复启动状态  等待小键盘重试
			
		Path_Point.velt[0].fpX = -500;
		Path_Point.velt[0].fpY = 0;
		Path_Point.velt[0].fpW = 0;	
	
		Path_Point.point[1].fpX = Retry_Dock_X;
		Path_Point.point[1].fpY = Retry_Dock_Y ;
		Path_Point.point[1].fpW = Retry_Dock_RAD;
		Path_Point.velt[1].fpX = 0;
		Path_Point.velt[1].fpY = 0;
		Path_Point.velt[1].fpW = 0;
		Path_Point.time[0] = 1200;  //ms

		Path_Point.point_num = 1;		
		break;
		
		case 51://贝塞尔测试1
			
		Path_Point.velt[0].fpX = 0;
		Path_Point.velt[0].fpY = 0;
		Path_Point.velt[0].fpW = 0;	
	
		Path_Point.point[1].fpX = -1026.75989;
		Path_Point.point[1].fpY = nav.auto_path.pos_pid.pid_y.fpFB;//-10109 ;
		Path_Point.point[1].fpW = 0.0f;
		Path_Point.velt[1].fpX = 800.f;
		Path_Point.velt[1].fpY = 0;
		Path_Point.velt[1].fpW = 0;
		Path_Point.time[0] = 1000;  //ms

		Path_Point.point_num = 1;				
		break;

		case 52://贝塞尔测试2
		Path_Point.velt[0].fpX = 0;
		Path_Point.velt[0].fpY = nav.expect_robot_global_velt.fpY;
		Path_Point.velt[0].fpW = 0;	
	
		Path_Point.point[1].fpX =nav.auto_path.pos_pid.pid_x.fpFB;//258;
		Path_Point.point[1].fpY = -8733.f ;
		Path_Point.point[1].fpW = 0.0f;
		Path_Point.velt[1].fpX = 0;
		Path_Point.velt[1].fpY = 0;
		Path_Point.velt[1].fpW = 0;
		Path_Point.time[0] = 1000;  //ms

		Path_Point.point_num = 1;				
		break;
		
		
	default:
		break;
	}
}





void Navigate_Task(void)
{
	static uint16_t WAIT_TIMER = 0;
	
    switch (nav.nav_state)
    {

    case NAV_POINT_TO_POINT: // 2
        if (Path_Point.flag_point_to_point)
        {
            path_point_choose(&nav);
            Path_Point.flag_point_to_point = 0;
        }
        Point_to_Point(&Path_Point);
        break;

    case NAV_LOCK: // 直接底盘输出置零
        if (flag_lock == 1)
        {
            SET_NAV_PATH_PERMUTATION();
            flag_lock = 0;
        }
				
				if(flag_area_to_point==0) ///只有不需要大范围切点到点，那么直接锁速度停车，防止切区域的时候出现顿挫
				{
						nav.expect_robot_global_velt.fpX = 0;
						nav.expect_robot_global_velt.fpY = 0;
						nav.expect_robot_global_velt.fpW = 0;			
				}else if(flag_area_to_point==2)//一区锁位置，加大底盘硬度
				{
					nav.nav_state = NAV_ONE_AREA_LOCK;
				}
		break;
				
		case NAV_DT35://DT35在一区取头  3
			PID_Calc(&NAV_DT35_PID.pid_x,area_one_target.target_x ,DT35_X_fact);
			PID_Calc(&NAV_DT35_PID.pid_y,area_one_target.target_y, DT35_Y_fact);
			PID_Calc_Angle(&NAV_DT35_PID.pid_w,area_one_target.target_rad, robot_pos.fpPosQ);
		
		  nav.expect_robot_global_velt.fpX = -NAV_DT35_PID.pid_x.fpU;
      nav.expect_robot_global_velt.fpY = -NAV_DT35_PID.pid_y.fpU;  //坐标系导致速度要取反
      nav.expect_robot_global_velt.fpW = NAV_DT35_PID.pid_w.fpU;	
		
		if (fabs(NAV_DT35_PID.pid_x.fpE) < 6.0 && fabs(NAV_DT35_PID.pid_y.fpE) < 6.0 && fabs(NAV_DT35_PID.pid_w.fpE)< 0.02) // 单位是mm、弧度
		{
			NAV_DT35_PID.pid_x.fpSumE = 0;
			NAV_DT35_PID.pid_y.fpSumE = 0;
			NAV_DT35_PID.pid_w.fpSumE = 0;
			chassis_run.wheel_1.fpSumE = 0;
			chassis_run.wheel_2.fpSumE = 0;			
			chassis_run.wheel_3.fpSumE = 0;			
			chassis_run.wheel_4.fpSumE = 0;		
			WAIT_TIMER = 0;
			nav.nav_state = NAV_LOCK;
		}else if(fabs(NAV_DT35_PID.pid_x.fpE) < 11.0 && fabs(NAV_DT35_PID.pid_y.fpE) < 11.0 && fabs(NAV_DT35_PID.pid_w.fpE)< 0.02)
		{
			WAIT_TIMER++;
		if(WAIT_TIMER >= 1500)
			{
				NAV_DT35_PID.pid_x.fpSumE = 0;
				NAV_DT35_PID.pid_y.fpSumE = 0;
				NAV_DT35_PID.pid_w.fpSumE = 0;
				chassis_run.wheel_1.fpSumE = 0;
				chassis_run.wheel_2.fpSumE = 0;			
				chassis_run.wheel_3.fpSumE = 0;			
				chassis_run.wheel_4.fpSumE = 0;						
				WAIT_TIMER = 0;
				nav.nav_state = NAV_LOCK;			
			}
		}
			break;
		
		case NAV_DT35_radar_AREA://  DT35混合雷达放置KFS  三区
			

			PID_Calc(&point_only.pid_x,area_three_target.target_x ,robot_pos.fpPosX);
			PID_Calc(&point_only.pid_y,area_three_target.target_y, robot_pos.fpPosY);
			PID_Calc_Angle(&point_only.pid_w,area_three_target.target_rad, robot_pos.fpPosQ);

		  nav.expect_robot_global_velt.fpX = point_only.pid_x.fpU;
      nav.expect_robot_global_velt.fpY = point_only.pid_y.fpU;
      nav.expect_robot_global_velt.fpW = point_only.pid_w.fpU;	
		
		if (fabs(point_only.pid_x.fpE) < 12.0 && fabs(point_only.pid_y.fpE) < 12.0 && fabs(point_only.pid_w.fpE)< 0.05) // 单位是mm、弧度
		{
			point_only.pid_x.fpSumE = 0;
			point_only.pid_y.fpSumE = 0;
			point_only.pid_w.fpSumE = 0;
			NAV_LOCK_POS_PID.pid_x.fpSumE = 0;
			NAV_LOCK_POS_PID.pid_y.fpSumE = 0;
			NAV_LOCK_POS_PID.pid_w.fpSumE = 0;		
			WAIT_TIMER = 0;
			nav.nav_state = NAV_LOCK_POS_3area;
		}		
		
		if (fabs(point_only.pid_x.fpE) < 28.0 && fabs(point_only.pid_y.fpE) < 28.0 && fabs(point_only.pid_w.fpE)< 0.1) // 单位是mm、弧度
		{	
				WAIT_TIMER++;
			
				if(WAIT_TIMER>=1500)
			{
				point_only.pid_x.fpSumE = 0;
				point_only.pid_y.fpSumE = 0;
				point_only.pid_w.fpSumE = 0;
				NAV_LOCK_POS_PID.pid_x.fpSumE = 0;
				NAV_LOCK_POS_PID.pid_y.fpSumE = 0;
				NAV_LOCK_POS_PID.pid_w.fpSumE = 0;	
				WAIT_TIMER = 0;
				nav.nav_state = NAV_LOCK_POS_3area;
			}
		}				
		
			break;
		
		case NAV_only_point_area2: // 5 二区梅林上  单纯点到点，不依靠路径规划
			choose_point_only();
		
			PID_Calc(&point_2_area.pid_x,area_two_target.target_x,robot_pos.fpPosX);
			PID_Calc(&point_2_area.pid_y,area_two_target.target_y, robot_pos.fpPosY);
			PID_Calc_Angle(&point_2_area.pid_w,area_two_target.target_rad, robot_pos.fpPosQ);			

		  nav.expect_robot_global_velt.fpX = point_2_area.pid_x.fpU;
      nav.expect_robot_global_velt.fpY = point_2_area.pid_y.fpU;
      nav.expect_robot_global_velt.fpW = point_2_area.pid_w.fpU;	
		
		
		if (fabs(point_2_area.pid_x.fpE) < 200 && fabs(point_2_area.pid_y.fpE) < 200 && fabs(point_2_area.pid_w.fpE)< 0.2) // 单位是mm、弧度
		{
				flag_one_area_ready = 1;
		}else {flag_one_area_ready = 0;}						
		
		
		
		
		if (fabs(point_2_area.pid_x.fpE) < allow_point.allow_x + 20 && fabs(point_2_area.pid_y.fpE) < allow_point.allow_y + 20&& fabs(point_2_area.pid_w.fpE)< allow_point.allow_rad+0.2) // 单位是mm、弧度
		{
			WAIT_TIMER++;
			
			if(WAIT_TIMER>=1000)
			{
				point_2_area.pid_x.fpSumE = 0;
				point_2_area.pid_y.fpSumE = 0;
				point_2_area.pid_w.fpSumE = 0;		
			
				NAV_LOCK_POS_PID.pid_x.fpSumE = 0;
				NAV_LOCK_POS_PID.pid_y.fpSumE = 0;
				NAV_LOCK_POS_PID.pid_w.fpSumE = 0;		
				WAIT_TIMER = 0;
				nav.nav_state = NAV_LOCK_POS;
			}
		}	
		
		
		if (fabs(point_2_area.pid_x.fpE) < allow_point.allow_x && fabs(point_2_area.pid_y.fpE) < allow_point.allow_y && fabs(point_2_area.pid_w.fpE)< allow_point.allow_rad) // 单位是mm、弧度
		{
			point_2_area.pid_x.fpSumE = 0;
			point_2_area.pid_y.fpSumE = 0;
			point_2_area.pid_w.fpSumE = 0;		
			
			NAV_LOCK_POS_PID.pid_x.fpSumE = 0;
			NAV_LOCK_POS_PID.pid_y.fpSumE = 0;
			NAV_LOCK_POS_PID.pid_w.fpSumE = 0;
      WAIT_TIMER = 0;			
			nav.nav_state = NAV_LOCK_POS;
		}						
			break;
		
		case NAV_LOCK_POS: // 6 锁位置在2区台阶上
			
			flag_one_area_ready = 0;	
		
			PID_Calc(&NAV_LOCK_POS_PID.pid_x,area_two_target.target_x ,robot_pos.fpPosX);
			PID_Calc(&NAV_LOCK_POS_PID.pid_y,area_two_target.target_y, robot_pos.fpPosY);
			PID_Calc_Angle(&NAV_LOCK_POS_PID.pid_w,area_two_target.target_rad, robot_pos.fpPosQ);				

		  nav.expect_robot_global_velt.fpX = NAV_LOCK_POS_PID.pid_x.fpU;
      nav.expect_robot_global_velt.fpY = NAV_LOCK_POS_PID.pid_y.fpU;
      nav.expect_robot_global_velt.fpW = NAV_LOCK_POS_PID.pid_w.fpU;			
			break;
		
		case NAV_LOCK_POS_3area: // 7 锁位置在三区微调之后 
			
		
			PID_Calc(&NAV_LOCK_POS_PID.pid_x,area_three_target.target_x ,robot_pos.fpPosX);
			PID_Calc(&NAV_LOCK_POS_PID.pid_y,area_three_target.target_y, robot_pos.fpPosY);
			PID_Calc_Angle(&NAV_LOCK_POS_PID.pid_w,area_three_target.target_rad, robot_pos.fpPosQ);				

		  nav.expect_robot_global_velt.fpX = NAV_LOCK_POS_PID.pid_x.fpU;
      nav.expect_robot_global_velt.fpY = NAV_LOCK_POS_PID.pid_y.fpU;
      nav.expect_robot_global_velt.fpW = NAV_LOCK_POS_PID.pid_w.fpU;				
			break;
		
		case NAV_LOCK_up_down: //二区上下台阶锁YAW和三区站立靠2006锁X  8
			if(all_path_state==2)
			{											
				PID_Calc_Angle(&NAV_LOCK_POS_PID.pid_w,GET_KFS_RAD, robot_pos.fpPosQ);			
			  nav.expect_robot_global_velt.fpW = NAV_LOCK_POS_PID.pid_w.fpU;		

			}
			break;
				
		
		case NAV_RAMP:
		 if (flag_lock == 1)
        {
            memset(&Path_Point, 0, sizeof(PATH_POINT));
						Path_Point.flag_point_to_point = 1;
            flag_lock = 0;
        }
			break;
				
		case NAV_ONE_AREA_LOCK:  //一区对接锁位置
			PID_Calc(&NAV_LOCK_POS_PID.pid_x,DOCK_X,robot_pos.fpPosX);
			PID_Calc(&NAV_LOCK_POS_PID.pid_y,DOCK_Y, robot_pos.fpPosY);
			PID_Calc_Angle(&NAV_LOCK_POS_PID.pid_w,DOCK_ANGLE, robot_pos.fpPosQ);		
if(fabs(NAV_LOCK_POS_PID.pid_x.fpE)>=20||fabs(NAV_LOCK_POS_PID.pid_y.fpE)>=20||fabs(NAV_LOCK_POS_PID.pid_w.fpE)>=0.05)
	{ 	
			nav.expect_robot_global_velt.fpX = NAV_LOCK_POS_PID.pid_x.fpU;
      nav.expect_robot_global_velt.fpY = NAV_LOCK_POS_PID.pid_y.fpU;  
      nav.expect_robot_global_velt.fpW = NAV_LOCK_POS_PID.pid_w.fpU;	
	}else {
		chassis_run.wheel_1.fpSumE = 0;
		chassis_run.wheel_2.fpSumE = 0;
		chassis_run.wheel_3.fpSumE = 0;
		chassis_run.wheel_4.fpSumE = 0;

		nav.expect_robot_global_velt.fpX = 0;
		nav.expect_robot_global_velt.fpY = 0;
		nav.expect_robot_global_velt.fpW = 0;
	}
			break;
		
		case NAV_REMOTE:
			
			break;
		
		case NAV_TEST: //测试版  13
			
			PID_Calc(&point_only.pid_x,target_test.target_x,robot_pos.fpPosX);
			PID_Calc(&point_only.pid_y,target_test.target_y, robot_pos.fpPosY);
			PID_Calc_Angle(&point_only.pid_w,target_test.target_rad,robot_pos.fpPosQ);	

		  nav.expect_robot_global_velt.fpX = point_only.pid_x.fpU;
      nav.expect_robot_global_velt.fpY = point_only.pid_y.fpU;  
      nav.expect_robot_global_velt.fpW = point_only.pid_w.fpU;			
		
		break;
				
    default:
        break;
    }
}





//一区总状态机
//默认夹取第一个头

// 板间通讯：
//  inner_send[1]: 0=空闲  1=夹头  2=对接  3=对接完成
 // inner_receive[1]: 0=空闲  1=取头成功  2=取头失败
//
void path_1(void)
{	
	static uint8_t  num_two_area = 0;   //等待系数
	static uint16_t delay_change_area= 0; //延迟切换到二区的计时器
	
	static uint8_t  nav_inited = 0;
	static uint8_t  prev_state = 0;
	static uint8_t  num_header = 0;  //当前是第几个矛头
	
	static uint8_t wait_2 = 1; //R1KFS在2号是否有
	
	if(vision_data_recieve.path_number==2&&vision_data_recieve.R1_two==1){wait_2 = 3;}else{wait_2 = 1;}
		
	static float temp_x[7], temp_y[7];  // 索引1~6使用
		
		if (path_state_1 != prev_state) {
      prev_state = path_state_1;
      nav_inited = 0;} /* 状态变化，重置标志 */ 	
   
	
	switch (path_state_1)
	{
		case 0: //初始状态
			delay_change_area = 0;
			nav_inited = 0;
			num_header = 0;
			break;
		
		case 1://从当前位置到一个夹头点位，若上层机构反馈夹头失败，需要重新取头
			num_header = 1;
			area_one_target.target_rad = HEAD_RAD;
			area_one_target.target_x = header_pos.header_1_x;
			area_one_target.target_y = header_pos.header_1_y;
		
		if(!nav_inited){nav.nav_state = NAV_DT35;nav_inited = 1;}
		
			if (nav.nav_state == NAV_LOCK) {  /* 通知上层：夹头 */
					inner_send[1] = 1;
			if (inner_receive[1] == 1) {
            /* 取头成功 → 去对接 */
            inner_send[1] = 200;
            path_state_1 = 7;
        } else if (inner_receive[1] == 2) {
            /* 取头失败 → 试下一个头 */
            inner_send[1] = 200;
            path_state_1 = 2;
        }else if (inner_receive[1] == 3){//武器头甩掉，回重试区
					inner_send[1] = 200;
					path_state_1 = 12;
				} 
			} 
			break;

		case 2://从当前位置一个夹头点位，上层机构反馈夹头失败,需要重新取头
			num_header = 2;
			area_one_target.target_rad = HEAD_RAD;
			area_one_target.target_x = header_pos.header_2_x;
			area_one_target.target_y = header_pos.header_2_y;
		
			if(!nav_inited){nav.nav_state = NAV_DT35;nav_inited = 1;}
		
			if (nav.nav_state == NAV_LOCK) { if (inner_send[1] != 1) inner_send[1] = 1; 
			
			if (inner_receive[1] == 1) { inner_send[1] = 200; path_state_1 = 7; }
      else if (inner_receive[1] == 2) { inner_send[1] = 200; path_state_1 = 3; }
			else if (inner_receive[1] == 3){//武器头甩掉，回重试区
					inner_send[1] = 200;
					path_state_1 = 12;
				} 
			}
			break;

		case 3://从当前位置到一个夹头点位，上层机构反馈夹头失败,需要重新取头
			num_header = 3;
			area_one_target.target_rad = HEAD_RAD;
			area_one_target.target_x = header_pos.header_3_x;
			area_one_target.target_y = header_pos.header_3_y;			
		
			if(!nav_inited){nav.nav_state = NAV_DT35;nav_inited = 1;}
		
			if (nav.nav_state == NAV_LOCK) { if (inner_send[1] != 1) inner_send[1] = 1; 
			
			if (inner_receive[1] == 1) { inner_send[1] = 200; path_state_1 = 7; }
      else if (inner_receive[1] == 2) { inner_send[1] = 200; path_state_1 = 4; }
			else if (inner_receive[1] == 3){//武器头甩掉，回重试区
					inner_send[1] = 200;
					path_state_1 = 12;
				} 
			}
			break;

		case 4://从当前位置到一个夹头点位，上层机构反馈夹头失败，需要重新取头
			num_header = 4;
			area_one_target.target_rad = HEAD_RAD;
			area_one_target.target_x = header_pos.header_4_x;
			area_one_target.target_y = header_pos.header_4_y;			
		
			if(!nav_inited){nav.nav_state = NAV_DT35;nav_inited = 1;}
		
			if (nav.nav_state == NAV_LOCK) { if (inner_send[1] != 1) inner_send[1] = 1; 
			
			if (inner_receive[1] == 1) { inner_send[1] = 200; path_state_1 = 7; }
      else if (inner_receive[1] == 2) { inner_send[1] = 200; path_state_1 = 5; }
			else if (inner_receive[1] == 3){//武器头甩掉，回重试区
					inner_send[1] = 200;
					path_state_1 = 12;
				} 
			}
			break;

		case 5://从当前位置到一个夹头点位，上层机构反馈夹头失败，需要重新取头
			num_header = 5;
			area_one_target.target_rad = HEAD_RAD;
			area_one_target.target_x = header_pos.header_5_x;
			area_one_target.target_y = header_pos.header_5_y;			
		
			if(!nav_inited){nav.nav_state = NAV_DT35;nav_inited = 1;}
		
			if (nav.nav_state == NAV_LOCK) { if (inner_send[1] != 1) inner_send[1] = 1; 
			
			if (inner_receive[1] == 1) { inner_send[1] = 200; path_state_1 = 7; }
      else if (inner_receive[1] == 2) { inner_send[1] = 200; path_state_1 = 6; }
			else if (inner_receive[1] == 3){//武器头甩掉，回重试区
					inner_send[1] = 200;
					path_state_1 = 12;
				} 
			}
			break;

		case 6://从当前位置到一个夹头点位，上层机构反馈夹头失败，需要重新取头
			num_header = 6;
			area_one_target.target_rad = HEAD_RAD;
			area_one_target.target_x = header_pos.header_6_x;
			area_one_target.target_y = header_pos.header_6_y;		
		
			if(!nav_inited){nav.nav_state = NAV_DT35;nav_inited = 1;}
		
			if (nav.nav_state == NAV_LOCK) { if (inner_send[1] != 1) inner_send[1] = 1; 
			
			if (inner_receive[1] == 1) { inner_send[1] = 200; path_state_1 = 7; }
      else if (inner_receive[1] == 2) { inner_send[1] = 200; path_state_1 = 7; }
			else if (inner_receive[1] == 3){//武器头甩掉，回重试区
					inner_send[1] = 200;
					path_state_1 = 12;
				} 
			}
			break;
		case 7://转动180度（取完矛头转过来与R1对接）并走到对接位置 需要上层机构反馈夹取成功标志位
			if(!nav_inited){nav.auto_path.number_point = 7;nav.nav_state = NAV_POINT_TO_POINT; flag_area_to_point = 2;nav_inited = 1;nav_reach_state = 3;}
			if (nav.nav_state == NAV_ONE_AREA_LOCK) { nav_reach_state = 0; flag_area_to_point = 0;
				
			if (vision_data_recieve.aruco_detect_flag==1) {
				
				nav.auto_path.pos_pid.pid_x.fpE = 0;
				nav.auto_path.pos_pid.pid_y.fpE = 0;
				nav.auto_path.pos_pid.pid_w.fpE = 0;
			
			inner_send[1] = 3;path_state_1 = 8;} 
				}
			break;

		case 8://R1二维码反馈对接成功，延迟200秒（默认打开夹爪）
			delay_change_area++;
			if(delay_change_area>=200) {delay_change_area = 0;path_state_1 = 9;}
			break;

		case 9://给R1让路
			if(!nav_inited){nav.auto_path.number_point = 8;nav.nav_state = NAV_POINT_TO_POINT; nav_inited = 1;nav_reach_state = 2;}
			if (nav.nav_state == NAV_LOCK){nav_reach_state = 0;path_state_1 = 10;}
			break;
		
		case 10://走到二区入口在一区的垂直映射点
			
			if(!nav_inited)	{if(vision_data_recieve.entry_kfs_id!=0){point_only_state=23+vision_data_recieve.entry_kfs_id;}else{point_only_state = 23+vision_data_recieve.path_number;} nav.nav_state = NAV_only_point_area2;nav_inited = 1;only_point_range_state = 1;  }
			
			if(nav.nav_state == NAV_LOCK_POS){if(vision_data_recieve.entry_kfs_id!=0){num_two_area = vision_data_recieve.entry_kfs_id;}else{num_two_area = vision_data_recieve.path_number;}  
					delay_change_area++;
					if(delay_change_area*num_two_area/wait_2>=1200){only_point_range_state = 0;delay_change_area = 0;path_state_1 = 14;}
			}
		
			break;
		
		case 11://一区先靠雷达导航到一个范围，用DT35确定六个点
			if(!nav_inited){flag_area_to_point = 1; nav.auto_path.number_point = 32;nav.nav_state = NAV_POINT_TO_POINT; nav_inited = 1;nav_reach_state = 5;}
			if (nav.nav_state == NAV_LOCK){flag_area_to_point = 0;nav_reach_state = 0;path_state_1 = num_header+1;nav_reach_state = 0;}
			break;
			
		case 12://武器头甩丢了，跑回重试区
			if(!nav_inited){nav.auto_path.number_point = 35;nav.nav_state = NAV_POINT_TO_POINT; nav_inited = 1;nav_reach_state = 1;}
			if (nav.nav_state == NAV_LOCK){if(key_receive[2]==6){path_state_1 = 11;}}			
			break;
			
		case 13://先根据视觉判断取头顺序
			//判断有无漏输
		
			if(vision_data_recieve.header_num_1+vision_data_recieve.header_num_2+vision_data_recieve.header_num_3+vision_data_recieve.header_num_4+vision_data_recieve.header_num_5+vision_data_recieve.header_num_6==21)
			{
				temp_x[1] = HEAD_1_X;
				temp_y[1] = HEAD_1_Y;

				temp_x[2] = HEAD_2_X;
				temp_y[2] = HEAD_2_Y;

				temp_x[3] = HEAD_3_X;
				temp_y[3] = HEAD_3_Y;

				temp_x[4] = HEAD_4_X;
				temp_y[4] = HEAD_4_Y;

				temp_x[5] = HEAD_5_X;
				temp_y[5] = HEAD_5_Y;

				temp_x[6] = HEAD_6_X;
				temp_y[6] = HEAD_6_Y;
				
				header_pos.header_1_x = temp_x[vision_data_recieve.header_num_1];
        header_pos.header_1_y = temp_y[vision_data_recieve.header_num_1];
				
        header_pos.header_2_x = temp_x[vision_data_recieve.header_num_2];
        header_pos.header_2_y = temp_y[vision_data_recieve.header_num_2];
        
        header_pos.header_3_x = temp_x[vision_data_recieve.header_num_3];
        header_pos.header_3_y = temp_y[vision_data_recieve.header_num_3];
        
        header_pos.header_4_x = temp_x[vision_data_recieve.header_num_4];
        header_pos.header_4_y = temp_y[vision_data_recieve.header_num_4];
        
        header_pos.header_5_x = temp_x[vision_data_recieve.header_num_5];
        header_pos.header_5_y = temp_y[vision_data_recieve.header_num_5];
        
        header_pos.header_6_x = temp_x[vision_data_recieve.header_num_6];
        header_pos.header_6_y = temp_y[vision_data_recieve.header_num_6];
			
			}else {
			
				header_pos.header_1_x = HEAD_1_X;
        header_pos.header_1_y = HEAD_1_Y;
				
        header_pos.header_2_x = HEAD_2_X;
        header_pos.header_2_y = HEAD_2_Y;
        
        header_pos.header_3_x = HEAD_3_X;
        header_pos.header_3_y = HEAD_3_Y;
        
        header_pos.header_4_x = HEAD_4_X;
        header_pos.header_4_y = HEAD_4_Y;
        
        header_pos.header_5_x = HEAD_5_X;
        header_pos.header_5_y = HEAD_5_Y;
				
        header_pos.header_6_x = HEAD_6_X;
        header_pos.header_6_y = HEAD_6_Y;			
			}
		
			path_state_1 = 11;
			
			break;
			
		case 14://一区结束切二区
			first_path_state = 0;
			all_path_state = 2;								
			break;

		default:
			break;
	}
		}




		
		
//二区导航状态机
void path_2(void)
{		
	  static uint8_t  nav_inited = 0;
    static uint8_t  prev_state = 0;
		static uint8_t 	first_step_flag = 0;
		static uint8_t  action_ing = 0;  //机械臂正在执行取块
		static uint8_t first_flag = 0;
		static uint8_t first_flag_receive = 0;
		static uint8_t flag_ready = 0;
		static uint8_t flag_receive_complete = 1;//存块完成
		static uint8_t flag_ready_complete = 1; //准备动作完成
		static uint8_t flag_receive_0 = 0; //只有上四百让机械臂姿态改变用
		static uint8_t timer_delay_5 = 0;   //延迟5ms切标志位防止上层卡死
		static uint8_t temp_flag = 0;
	
		static uint8_t flag_ready_first = 0;
  
    if (path_state_2 != prev_state) {
        prev_state = path_state_2;
        nav_inited = 0;  /* 状态变化，重置标志 */
				first_step_flag = 0;
				action_ing = 0;
				first_flag = 0;
				first_flag_receive = 0;
		  	flag_ready = 0;
		}
		
	if(inner_receive[2] == 2){temp_flag = 1;inner_send[1] = 200;}

	if(temp_flag){timer_delay_5++;if(timer_delay_5>=5){flag_receive_complete = 1;timer_delay_5=0;temp_flag = 0;}}
	if(inner_receive[5] == 2){flag_ready_complete = 1;inner_send[5] = 0;}
	
	switch (path_state_2)
	{
		case 0: //初始状态
			nav_inited = 0;	
			break;
		
		case 1://进行判断，如果入口有KFS，那么进入状态2,无KFS则进入状态3
			if(vision_data_recieve.entry_kfs_id!=0){path_state_2 = 2;}
			else {path_state_2 = 3;}
			break;

		case 2://从当前位置前往取块的入口
		choose_action_arm();
		if(vision_data_recieve.entry_kfs_id==2){inner_send[2] = 2;}
		else{inner_send[2] = 3;}
			inner_send[3] = 2;
		
		if (!nav_inited){point_only_state = vision_data_recieve.entry_kfs_id;nav.nav_state = NAV_only_point_area2;nav_inited = 1;}
		if(flag_one_area_ready==1&&flag_ready_first==0){inner_send[5] = 1;flag_ready_first = 1;}
		
		if(nav.nav_state == NAV_LOCK_POS&&inner_receive[5]!=1) {
					inner_send[5] = 2;
					inner_send[1] = 100;
					flag_receive_complete = 0;
				}
		
		if(inner_receive[2] == 3){if(vision_data_recieve.entry_kfs_id==vision_data_recieve.path_number){path_state_2 = 4;reach_kfs_num = 1;}else {path_state_2 = 3;reach_kfs_num = 1;}}
			break;

		case 3://前往规划路径入口
		if (!nav_inited)	{point_only_state = vision_data_recieve.path_number;nav.nav_state = NAV_only_point_area2;only_point_range_state = 1;nav_inited = 1;}
		if(nav.nav_state == NAV_LOCK_POS){only_point_range_state = 0;path_state_2 = 4;}
			break;

		case 4://开环登上第一个台阶
		if(inner_send[5]!=1){first_flag = 1;} 
		if(inner_receive[2]==0){flag_receive_0 = 1;}
		
		if(first_flag==1&&flag_receive_complete==1&&flag_receive_0==1)
	{
		nav.nav_state = NAV_LOCK_up_down;
		if(first_step_flag == 0)
			{
				if(vision_data_recieve.path_number==2){up_down_state = 1;}
				else{inner_send[1] = 13;up_down_state = 2;}
				first_step_flag = 1;
			}
		if(up_down_state==0)
		{
			inner_send[1] = 200;
			first_step_flag = 1;
			if(vision_data_recieve.state_1 == 0){path_state_2 = 7;}				//不吸
			else if(vision_data_recieve.state_1 == 1){path_state_2 = 6;}	//正吸
			else{path_state_2 = 5;}																				//存在侧吸
		}
	}
	
			break;

		case 5://第一个台阶侧吸
	if(flag_receive_complete==1&&inner_receive[6]!=1)
	{choose_action_arm();//选择机械臂 扔 存 持三个动作
		switch(vision_data_recieve.path_number)//根据路径判断机械臂 高 中 低
				{
					case 1:
						inner_send[2] = 1;
						break;
					
					case 2:
						inner_send[2] = 2;
						break;
					
					case 3:
						inner_send[2] = 1;
						break;
				}
		if(vision_data_recieve.state_1==2||vision_data_recieve.state_1==4){inner_send[3] = 1;}//若左侧吸
		else {inner_send[3] = 3;}//若右侧吸
		
			if(flag_ready==0){inner_send[5] = 1;flag_ready = 1;flag_ready_complete = 0;}//准备动作
	}

		
		if (!nav_inited){point_only_state = 6;nav.nav_state = NAV_only_point_area2;nav_inited = 1;only_point_range_state = 0;}
		if(nav.nav_state == NAV_LOCK_POS)
			{	
				if(inner_send[5]!=2&&flag_ready_complete==1&&flag_receive_complete==1){inner_send[5] = 2;inner_send[1] = 100;action_ing = 1;flag_receive_complete = 0;}
				if(action_ing==1){if(inner_receive[2] == 3){reach_kfs_num++;if(vision_data_recieve.state_1==4||vision_data_recieve.state_1==5){path_state_2 = 6;}else{path_state_2 = 7;}}}
			}
			break;

		case 6://第一个台阶正吸
			if(flag_receive_complete==1&&inner_receive[6]!=1)
	{
		choose_action_arm();//选择机械臂 扔 存 持三个动作
		switch(vision_data_recieve.path_number)//根据路径判断机械臂 高 中 低
				{
					case 1:
						inner_send[2] = 1;
						break;
					
					case 2:
						inner_send[2] = 2;
						break;
					
					case 3:
						inner_send[2] = 2;
						break;				
				}
		inner_send[3] = 2;
				
		if(flag_ready==0){inner_send[5] = 1;flag_ready = 1;flag_ready_complete = 0;}//准备动作
	}
		
		if (!nav_inited)	{point_only_state = 5;nav.nav_state = NAV_only_point_area2;nav_inited = 1;only_point_range_state = 0;}
		if(nav.nav_state == NAV_LOCK_POS) 
			{
				if(inner_send[5]!=2&&flag_ready_complete==1&&flag_receive_complete==1){inner_send[5] = 2;inner_send[1] = 100;action_ing = 1;flag_receive_complete = 0;}
				if(action_ing==1){if(inner_receive[2] == 3){reach_kfs_num++;path_state_2 = 8;}}
			}
			break;

		case 7://第一个台阶居中（仅在侧吸一个或不吸时需要居中）
		if (!nav_inited)	{point_only_state = 4;nav.nav_state = NAV_only_point_area2;;nav_inited = 1;only_point_range_state = 1;}
		if(nav.nav_state == NAV_LOCK_POS){only_point_range_state = 0;path_state_2 = 8;}	
			break;

		case 8://开环登上第二个台阶
		nav.nav_state = NAV_LOCK_up_down;
		if(first_step_flag==0)
			{
				if(vision_data_recieve.path_number==1){up_down_state = 3;}
				else {up_down_state = 1;}
				first_step_flag = 1;
			}
	
	if(up_down_state==0)
		{
			first_step_flag = 1;
			if(vision_data_recieve.state_2 == 0){path_state_2 = 11;}				//不吸
			else if(vision_data_recieve.state_2 == 1){path_state_2 = 10;}		//正吸
			else{path_state_2 = 9;}																					//存在侧吸
		}
			break;

		case 9://第二个台阶侧吸
		if(flag_receive_complete==1)
	{
		choose_action_arm();//选择机械臂 扔 存 持三个动作
		switch(vision_data_recieve.path_number)//根据路径判断机械臂 高 中 低
				{
					case 1:
						inner_send[2] = 2;
						break;
					
					case 2:
				if(vision_data_recieve.state_2==2||vision_data_recieve.state_2==4){inner_send[2] = 2;}//左侧
				else{inner_send[2] = 1;}
						break;
					
					case 3:
						inner_send[2] = 1;
						break;
				}
		if(vision_data_recieve.state_2==2||vision_data_recieve.state_2==4){inner_send[3] = 1;}//若左侧吸
		else {inner_send[3] = 3;}//若右侧吸
		if(flag_ready==0){inner_send[5] = 1;flag_ready = 1;flag_ready_complete = 0;}//准备动作	
	}		
		
		
		if (!nav_inited){point_only_state = 9;nav.nav_state = NAV_only_point_area2;;nav_inited = 1;only_point_range_state = 0;}
		if(nav.nav_state == NAV_LOCK_POS)
			{	
				if(inner_send[5]!=2&&flag_ready_complete==1&&flag_receive_complete==1){inner_send[5] = 2;inner_send[1] = 100;action_ing = 1;flag_receive_complete = 0;}
				if(action_ing==1){if(inner_receive[2] == 3){reach_kfs_num++;if(vision_data_recieve.state_2==4||vision_data_recieve.state_2==5){path_state_2 = 10;}else{path_state_2 = 11;}}}
			}
			break;
		
		case 10://第二个台阶正吸
		if(flag_receive_complete==1)
		{
		choose_action_arm();//选择机械臂 扔 存 持三个动作
		switch(vision_data_recieve.path_number)//根据路径判断机械臂 高 中 低
				{
					case 1:
						inner_send[2] = 2;
						break;
					
					case 2:
						inner_send[2] = 2;
						break;
					
					case 3:
						inner_send[2] = 1;
						break;				
				}
		inner_send[3] = 2;
		if(flag_ready==0){inner_send[5] = 1;flag_ready = 1;flag_ready_complete = 0;}//准备动作		
	}
		
				
		if (!nav_inited)	{point_only_state = 8;nav.nav_state = NAV_only_point_area2;nav_inited = 1;only_point_range_state = 0;}
		if(nav.nav_state == NAV_LOCK_POS) 
			{
				if(inner_send[5]!=2&&flag_ready_complete==1&&flag_receive_complete==1){inner_send[5] = 2;inner_send[1] = 100;action_ing = 1;flag_receive_complete = 0;}
				if(action_ing==1){if(inner_receive[2] == 3){reach_kfs_num++;path_state_2 = 12;}}
			}
			break;
		
		case 11://第二个台阶居中
		if (!nav_inited)	{point_only_state = 7;nav.nav_state = NAV_only_point_area2;nav_inited = 1;only_point_range_state = 1;}
		if(nav.nav_state == NAV_LOCK_POS){only_point_range_state = 0;path_state_2 = 12;}				
			break;
		
		case 12://开环登第三个台阶
		nav.nav_state = NAV_LOCK_up_down;
		if(first_step_flag==0)
			{
				if(vision_data_recieve.path_number==3){up_down_state = 3;}
				else {up_down_state = 1;}
				first_step_flag = 1;
			}
		if(up_down_state==0)
		{
			first_step_flag = 1;
			if(vision_data_recieve.state_3 == 0){path_state_2 = 15;}				//不吸
			else if(vision_data_recieve.state_3 == 1){path_state_2 = 14;}		//正吸
			else{path_state_2 = 13;}																					//存在侧吸
		}
			break;
		
		case 13://第三个台阶侧吸
		if(flag_receive_complete==1)
		{
		choose_action_arm();//选择机械臂 扔 存 持三个动作
		switch(vision_data_recieve.path_number)//根据路径判断机械臂 高 中 低
				{
					case 1:
						inner_send[2] = 2;
						break;
					
					case 2:
						inner_send[2] = 1;
						break;
					
					case 3:
						inner_send[2] = 2;
						break;
				}			
		if(vision_data_recieve.state_3==2||vision_data_recieve.state_3==4){inner_send[3] = 1;}//若左侧吸
		else {inner_send[3] = 3;}//若右侧吸
		if(flag_ready==0){inner_send[5] = 1;flag_ready = 1;flag_ready_complete = 0;}//准备动作		
	}
		
		if (!nav_inited){point_only_state = 12;nav.nav_state = NAV_only_point_area2;nav_inited = 1;only_point_range_state = 0;}
		if(nav.nav_state == NAV_LOCK_POS)
			{	
				if(inner_send[5]!=2&&flag_ready_complete==1&&flag_receive_complete==1){inner_send[5] = 2;inner_send[1] = 100;action_ing = 1;flag_receive_complete = 0;}
				if(action_ing==1){if(inner_receive[2] == 3){reach_kfs_num++;if(vision_data_recieve.state_3==4||vision_data_recieve.state_3==5){path_state_2 = 14;}else{path_state_2 = 15;}}}
			}
			break;
		
		case 14://第三个台阶正吸
	if(flag_receive_complete==1)
	{
		choose_action_arm();//选择机械臂 扔 存 持三个动作
		switch(vision_data_recieve.path_number)//根据路径判断机械臂 高 中 低
				{
					case 1:
						inner_send[2] = 1;
						break;
					
					case 2:
						inner_send[2] = 1;
						break;
					
					case 3:
						inner_send[2] = 1;
						break;				
				}			
		inner_send[3] = 2;
		if(flag_ready==0){inner_send[5] = 1;flag_ready = 1;flag_ready_complete = 0;}//准备动作
	}		
				
		if (!nav_inited){point_only_state = 11;nav.nav_state = NAV_only_point_area2;nav_inited = 1;only_point_range_state = 0;}
		if(nav.nav_state == NAV_LOCK_POS) 
			{
				if(inner_send[5]!=2&&flag_ready_complete==1&&flag_receive_complete==1){inner_send[5] = 2;inner_send[1] = 100;action_ing = 1;flag_receive_complete = 0;}
				if(action_ing==1){if(inner_receive[2] == 3){reach_kfs_num++;path_state_2 = 16;}}
			}		
			break;
		
		case 15://第三个台阶居中
		if (!nav_inited)	{point_only_state = 10;nav.nav_state = NAV_only_point_area2;nav_inited = 1;only_point_range_state = 1;}
		if(nav.nav_state == NAV_LOCK_POS){path_state_2 = 16;only_point_range_state = 0;}		
			break;
		
		case 16://开环上第四个台阶
		nav.nav_state = NAV_LOCK_up_down;
		if(first_step_flag==0)
		{up_down_state = 3;first_step_flag = 1;}
	
		if(up_down_state==0)
			{
				first_step_flag = 1;
				if(vision_data_recieve.state_4 == 0){path_state_2 = 18;}				//不吸
				else{path_state_2 = 17;}																				//存在侧吸
			}
			break;
		
		case 17://第四个台阶侧吸
		if(flag_receive_complete==1)
	{
		choose_action_arm();//选择机械臂 扔 存 持三个动作
		switch(vision_data_recieve.path_number)//根据路径判断机械臂 高 中 低
				{
					case 1:
						inner_send[2] = 2;
						break;
					
					case 2:
						inner_send[2] = 1;
						break;
					
					case 3:
						inner_send[2] = 2;
						break;
				}			
		if(vision_data_recieve.state_4==2||vision_data_recieve.state_4==4){inner_send[3] = 1;}//若左侧吸
		else {inner_send[3] = 3;}//若右侧吸
		if(flag_ready==0){inner_send[5] = 1;flag_ready = 1;flag_ready_complete = 0;}//准备动作	
	}
		
		
		if (!nav_inited){point_only_state = 15;nav.nav_state = NAV_only_point_area2;nav_inited = 1;only_point_range_state = 0;}
		if(nav.nav_state == NAV_LOCK_POS)
			{	

				if(inner_send[5]!=2&&flag_ready_complete==1&&flag_receive_complete==1){inner_send[5] = 2;inner_send[1] = 100;action_ing = 1;flag_receive_complete = 0;}
				if(action_ing==1){if(inner_receive[2] == 3){reach_kfs_num++;path_state_2 = 18;}}
			}
			break;

		case 18://第四个台阶居中
		if (!nav_inited)	{point_only_state = 13;nav.nav_state = NAV_only_point_area2;;nav_inited = 1;only_point_range_state = 1;}
		if(nav.nav_state == NAV_LOCK_POS){only_point_range_state = 0;path_state_2 = 19;}				
			break;
		
		case 19://开环下台阶出二区
		if(inner_receive[2]==0)	{first_flag_receive = 1;}
		
		
		if(first_flag_receive == 1)
		{
		nav.nav_state = NAV_LOCK_up_down;
		if(first_step_flag==0)
		{
			if(vision_data_recieve.path_number==2){inner_send[1] = 13;up_down_state = 4;}
			else {up_down_state = 3;}
			first_step_flag = 1;
			inner_send[5] = 0;
		}
	
		if(up_down_state==0)
			{
				inner_send[1] = 200;
				path_state_2 = 20;
			}
		}
			break;
		
		case 20://根据从哪条路径出来判断，走哪条上三区的路
			ramp_test_flag=1;
			if(ramp_state ==6)
			{
				ramp_test_flag=0;
					path_state_2 = 22;
			}		
			break;
				
		case 21://冲上三区
			break;			
		
		case 22://二区结束，切换三区状态机
			first_path_state = 0;
			all_path_state = 3;			
			break;
		
		default:
			break;
	}
}



//视觉二维码 vision_data_recieve.aruco_detect_flag
//	1 对接成功
//	2 三区1列
//	3 三区2列
//	4 三区3列
//	5 合体成功收腿
//	6 可以从R1上拿走KFS
//	7 放KFS
//	8 从R1上传递KFS
//  9 直接合体
//  10 强制打断


//三区导航状态机
void path_3(void)
{		
		static uint8_t  nav_inited = 0;
    static uint8_t  prev_state = 0;
    static uint16_t qrcode_timer = 0;//等待二维码时间，超出1000ms还没识别那么直接走,默认放到2列
	  static uint8_t flag_first = 0;
		static uint8_t flag_qrcode = 0;
		static uint8_t timer_delay = 0; //时间延迟，防止通讯出问题
		static uint8_t flag_get = 0;  //R2取块完成标志位
	static uint8_t flag_R1_POS = 2;  //R1在R2的哪一侧  红场默认为2
	
		static uint8_t KFS_NUM = 0;   //车上KFS的数量
		static uint8_t force_interrupt = 0; //强制打断标志位
		static uint8_t nav_ing = 0;  //导航进行中
		static uint8_t pre_flag_qrcode = 0; //上次导航到达的目标点位，用于可能放置两个中层KFS
		static uint8_t flag_up = 0; //计数是否已经合体（用于强制打断之后）
		static uint8_t timer_delay_5 = 0;  //放完KFS5ms后才能发送改姿态并站起来
		static uint8_t flag_ready_up = 0;  //可以发送改姿态并站起的标志位
		static uint8_t flag_temp = 0; //中间标志位
		static uint8_t flag_put_cnt = 0; //放了几次中层
		static uint8_t flag_get_ok = 0;
	  static uint8_t remain_kfs = 0; //剩余KFS数量
	
	if(vision_data_recieve.arcuo_left_or_right==1){flag_R1_POS = 1;}else if(vision_data_recieve.arcuo_left_or_right==2){flag_R1_POS = 2;}
	inner_send[6] = flag_R1_POS;
	
	if(vision_data_recieve.aruco_detect_flag==4&&vision_data_recieve.arcuo_left_or_right==1){flag_qrcode=3;}
	else if(vision_data_recieve.aruco_detect_flag==3){flag_qrcode=2;}
	else if(vision_data_recieve.aruco_detect_flag==2&&vision_data_recieve.arcuo_left_or_right==2){flag_qrcode=1;}
	
	if(vision_data_recieve.aruco_detect_flag==10){force_interrupt = 1;}  //强制打断的二维码，后面将切到R1控制模式
	
	
    if (path_state_3 != prev_state) {
        prev_state = path_state_3;
        nav_inited = 0;  
				flag_first = 0;/* 状态变化，重置标志 */
				flag_get = 0;
				timer_delay = 0;
				timer_delay_5 = 0;
				flag_temp = 0;
			  flag_ready_up  = 0;
			  flag_get_ok = 0;
		}
				
	switch (path_state_3)
	{
		case 0: //初始状态
			nav_inited = 0;
			flag_first = 0;
			break;
		
		case 1: //识别二维码
		KFS_NUM = vision_data_recieve.KFS_number;
		remain_kfs = KFS_NUM;
		if (!nav_inited){ nav_inited = 1;nav_reach_state = 1;}

		if (nav.nav_state == NAV_LOCK) 
			{
				if(force_interrupt){path_state_3 = 12;}
				else{
//							qrcode_timer++;
							if(flag_qrcode==1&&flag_R1_POS==1){path_state_3 = 2;}
							else if(flag_qrcode==2){path_state_3 = 3;}
							else if(flag_qrcode==3&&flag_R1_POS==2){path_state_3 = 4;}			
//							if(qrcode_timer>=1000){flag_qrcode=2;path_state_3 = 3;}	//路上没有识别到，等待1000ms也没识别到就默认放中间
							else if(flag_qrcode==0){flag_qrcode = 2;path_state_3 = 3; }
						}
			}
			break;

		case 2://从当前位置走到1列九宫格前方，告诉上层放置KFS
			nav_reach_state = 0;
			area_three_target.target_rad = GRID_RAD;
			area_three_target.target_x = GRID_1_X;
			area_three_target.target_y = GRID_1_Y;
				
			if(nav.nav_state != NAV_LOCK_POS_3area&&force_interrupt==1){path_state_3 = 12;}
			
			if(!nav_inited){nav.nav_state = NAV_DT35_radar_AREA;nav_inited = 1;nav_reach_state = 0;}		
			if (nav.nav_state == NAV_LOCK_POS_3area) {{inner_send[1] = 8;}if(inner_receive[3] == 2){inner_send[1] = 200;path_state_3 = 5;}}
				
			break;

		case 3://从当前位置走到2列九宫格前方，告诉上层放置KFS
		nav_reach_state = 0;
		area_three_target.target_rad = GRID_RAD;
		area_three_target.target_x = GRID_2_X;
		area_three_target.target_y = GRID_2_Y;			
		
		if(nav.nav_state != NAV_LOCK_POS_3area&&force_interrupt==1){path_state_3 = 12;}
		else if(nav.nav_state != NAV_LOCK_POS_3area&&force_interrupt!=1){if(flag_qrcode==1){nav.nav_state = NAV_INIT;path_state_3 = 2;}else if(flag_qrcode==3){nav.nav_state = NAV_INIT;path_state_3 = 4;}}
	
		if(!nav_inited){nav.nav_state = NAV_DT35_radar_AREA;nav_inited = 1;nav_reach_state = 0;}
		if (nav.nav_state == NAV_LOCK_POS_3area) {inner_send[1] = 8;if(inner_receive[3] == 2){inner_send[1] = 200;path_state_3 = 5;}}
			break;

		case 4://从当前位置走到3列九宫格前方，告诉上层放置KFS
		nav_reach_state = 0;
		area_three_target.target_rad = GRID_RAD;
		area_three_target.target_x = GRID_3_X;
		area_three_target.target_y = GRID_3_Y;			
		
		if(nav.nav_state != NAV_LOCK_POS_3area&&force_interrupt==1){path_state_3 = 12;}
		
		if(!nav_inited){nav.nav_state = NAV_DT35_radar_AREA;nav_inited = 1;nav_reach_state = 0;}
		if (nav.nav_state == NAV_LOCK_POS_3area) {inner_send[1] = 8;if(inner_receive[3] == 2){inner_send[1] = 200;path_state_3 = 5;}}	
			break;
			
			
		case 5://站立合体
			timer_delay_5++;
		if(timer_delay_5>=5){flag_ready_up=1;timer_delay_5=0;}
		if(flag_ready_up==1)
			
		{	inner_send[1] = 15;
			if(!nav_inited){nav.nav_state = NAV_LOCK; nav_inited = 1;}
			if(flag_first==0){up_down_state = 6;flag_first=1;}if(up_down_state==0){path_state_3 = 6;}
		}
		break;

			
		case 6://合体成功
			if(!nav_inited){nav.nav_state = NAV_LOCK; nav_inited = 1;}
			
			if(KFS_NUM>=2){inner_send[1] = 9;path_state_3 = 7;}else if(KFS_NUM==1){inner_send[1] = 11;path_state_3 = 9;}			
			break;
			
		case 7: //背后有一个KFS
		if(inner_receive[4] == 2){inner_send[1] = 200; flag_get=1;}
		
		if(flag_get){timer_delay++;if(timer_delay>=5){path_state_3 = 10;}}
			break;
			
		case 8: //带一个KFS重试
			KFS_NUM = 1;
		if (!nav_inited){nav_inited = 1;nav_reach_state = 0;nav.auto_path.number_point = 27;nav.nav_state = NAV_POINT_TO_POINT;}
		if (nav.nav_state == NAV_LOCK) 
			{
          flag_put_cnt = 0;
          force_interrupt = 1;
					flag_qrcode = 2;  // 强制去2列
					pre_flag_qrcode = flag_qrcode;
				  path_state_3 = 14;
			}
			break;
			
		case 9://取R1
	if(vision_data_recieve.aruco_detect_flag == 6&&flag_get==0){inner_send[1] = 17;} //可以拿走
	
	if(inner_receive[4] == 2){inner_send[1] = 200;flag_get = 1;}
	
	if(flag_get){timer_delay++; if(timer_delay>=5){path_state_3 = 10;} }
			break;
		
		case 10://等待R1信号 非打断
			if(vision_data_recieve.aruco_detect_flag == 7){inner_send[1] = 10;}
			if(inner_receive[3] ==2 ){inner_send[1] = 200;path_state_3 = 11;}
			
			break;
		
		case 11: //等待R1信号再取一个R1KFS
			if(vision_data_recieve.aruco_detect_flag == 8){inner_send[1] = 11;path_state_3 = 9;}
			break;
			
		case 12: //走完当前的导航  强制打断R2，后面都听R1信息
			if(nav.nav_state == NAV_DT35_radar_AREA){nav_ing = 1;}
			
			if(nav_ing==1){if(nav.nav_state == NAV_LOCK_POS_3area){ flag_put_cnt = 0;path_state_3 = 13;}}
			else if(nav_ing==0){flag_put_cnt = 0;path_state_3 = 13;}
		
			break;
		
		case 13: //放在中层时的导航  （可换列）
				if(flag_qrcode==1){area_three_target.target_rad = GRID_RAD;area_three_target.target_x = GRID_1_X;area_three_target.target_y = GRID_1_Y;}
				else if(flag_qrcode==3){area_three_target.target_rad = GRID_RAD;area_three_target.target_x = GRID_3_X;area_three_target.target_y = GRID_3_Y;}
				else if(flag_qrcode==2||flag_qrcode==0){area_three_target.target_rad = GRID_RAD;area_three_target.target_x = GRID_2_X;area_three_target.target_y = GRID_2_Y;}
				
			if(!nav_inited){nav.nav_state = NAV_DT35_radar_AREA;nav_inited = 1;nav_reach_state = 0;}
			
			if (nav.nav_state == NAV_LOCK_POS_3area){pre_flag_qrcode = flag_qrcode;path_state_3 = 14;}
		
			break;
		
		
		case 14://等待R1信号 打断后
    if(!nav_inited){nav.nav_state = NAV_LOCK;nav_inited = 1;}
    
    //换列检测
    if(flag_qrcode != pre_flag_qrcode){
        pre_flag_qrcode = flag_qrcode;
        nav_inited = 0;
        path_state_3 = 13;
    }
    else if(vision_data_recieve.aruco_detect_flag == 7){
        inner_send[1] = 8;flag_put_cnt++;nav_inited = 0;path_state_3 = 15;
    }
    else if(vision_data_recieve.aruco_detect_flag == 9){
        nav_inited = 0;path_state_3 = 17;
    }
		
			break;
		
		case 15://中层放置KFS
			if(inner_receive[3] == 2 ){inner_send[1] = 200;flag_temp = 1;}	
			if(flag_temp==1){timer_delay_5++;if(timer_delay_5>=5){flag_get_ok = 1;timer_delay_5 = 0;}}
			
			if(flag_get_ok==1)
			{
				remain_kfs = KFS_NUM - flag_put_cnt; 
				if(remain_kfs > 0){inner_send[1] = 9;path_state_3 = 16;}
				else {path_state_3 = 17;}			
			}
			
			break;
		
		
		case 16://中间状态
		if(inner_receive[4] == 2){inner_send[1] = 200; flag_get=1;}
		
		if(flag_get==1)
		{
				if(flag_qrcode != pre_flag_qrcode){ pre_flag_qrcode = flag_qrcode;path_state_3 = 13;}//如果又更改目标点
			 else if(vision_data_recieve.aruco_detect_flag == 7){flag_put_cnt++; inner_send[1] = 8; path_state_3 = 15; }
				else if(vision_data_recieve.aruco_detect_flag == 9){path_state_3 = 17;}  //站起来等待合体
		}
		
			break;
		
		case 17: //站起合体
		remain_kfs = KFS_NUM - flag_put_cnt;
		if(remain_kfs < 0) {remain_kfs = 0;}
		timer_delay_5++;
		if(timer_delay_5>=5){flag_ready_up=1;timer_delay_5=0;}
		if(flag_ready_up==1)
			
		{	inner_send[1] = 15;//改变姿态
			if(!nav_inited){nav.nav_state = NAV_LOCK; nav_inited = 1;}
			if(flag_first==0){up_down_state = 6;flag_first=1;}if(up_down_state==0){path_state_3 = 18;}
		}			
			break;
		
		case 18://已站在R1上
			if(remain_kfs==0){path_state_3 = 11;}//手里无KFS
			 else {if(flag_first==0){inner_send[1] = 23;flag_first = 1;}}
			 
			if(flag_first){if(inner_receive[6] == 2){inner_send[1] = 200;flag_temp = 1;}}
			if(flag_temp){timer_delay_5++;if(timer_delay_5>=5){if(remain_kfs==1){path_state_3 = 10;}else if(remain_kfs==2){path_state_3 = 19;}}}
			break;
		
		case 19://先放一个
			if(vision_data_recieve.aruco_detect_flag == 7){inner_send[1] = 10;}
			if(inner_receive[3] ==2 ){inner_send[1] = 200;path_state_3 = 20;}			
			break;
		
		case 20://取背后一个再切10
			timer_delay_5++;
		if(timer_delay_5>=5){inner_send[1] = 9;timer_delay_5 = 0;path_state_3 = 21;}
			break;
		
		case 21://等待取背后成功
			if(inner_receive[4] == 2){inner_send[1] = 200; flag_get=1;}
			
			if(flag_get){timer_delay++;if(timer_delay>=5){path_state_3 = 10;}}
			break;		
			
		case 22: //带两个KFS重试
		KFS_NUM = 2;
		if (!nav_inited){nav_inited = 1;nav_reach_state = 0;nav.auto_path.number_point = 27;nav.nav_state = NAV_POINT_TO_POINT;}
		if (nav.nav_state == NAV_LOCK) 		
		{
        flag_qrcode = 2;    
			  flag_put_cnt = 0; 
			  force_interrupt = 1;
        pre_flag_qrcode = flag_qrcode; 
        path_state_3 = 14;        
		}
		break;
		
		
		default:
			break;
	}
}




//二区取块高度 inner_send[2] （相对于轮子，存在三种） 0初始化 1低200 2高200 3高400
//二区取块方向 inner_send[3] （相对于车身正面） 0初始化 1左 2正 3右
//二区取块动作 inner_send[4] 0初始  1扔块（向后扔） 2存块 3手持（只有取最后一个KFS才会手持）

void choose_action_arm(void)
{
	switch(vision_data_recieve.KFS_number)
	{
		case 1:
			if(reach_kfs_num==0){inner_send [4] = 3;}//需要取1个，已经经过了0个
			break;
						
		case 2:
			if(reach_kfs_num==0){inner_send [4] = 2;}//需要取2个，已经经过了0个
			else if(reach_kfs_num==1){inner_send [4] = 3;}//需要取2个，已经经过了1个
			break;
						
		case 3:
			if(reach_kfs_num==0){inner_send [4] = 1;}//需要取3个，已经经过了0个
			else if(reach_kfs_num==1){inner_send [4] = 2;}//需要取3个，已经经过了1个
			else if(reach_kfs_num==2){inner_send [4] = 3;}//需要取3个，已经经过了2个
			break;
	}
}





////三区无法大胜，回到一区重试，重走一遍原路径（若没有额外KFS可以取，不必重试）
//void path_4(void) 
//{
//	  static uint8_t  nav_inited = 0;
//    static uint8_t  prev_state = 0;
//		static uint8_t  first_flag = 0;
//		static uint8_t  first_flag_receive = 0;
//		static uint8_t  first_step_flag = 0;
//	
//    if (path_state_4 != prev_state) {
//        prev_state = path_state_4;
//        nav_inited = 0;  /* 状态变化，重置标志 */
//				first_step_flag = 0;
//				first_flag = 0;
//				first_flag_receive = 0;
//		}
//		
//	if(inner_receive[2] == 2){inner_send[1] = 0;}
//	
//	switch (path_state_4)
//	{
//		case 0: //初始状态
//			nav_inited = 0;	
//			break;
//		
//		case 1://直接进入原路径
//		point_2_area.pid_w.fpKp = 3.2;
//		if (!nav_inited)	{point_only_state = vision_data_recieve.path_number;nav.nav_state = NAV_only_point_area2;nav_inited = 1;}
//		if(nav.nav_state == NAV_LOCK_POS){point_2_area.pid_w.fpKp = 1.5;path_state_4 = 2;}
//			break;
//			
//		case 2://开环登上第一个台阶
//		if(inner_send[1]==0){first_flag = 1;}
//		if(inner_receive[2]==0){first_flag_receive = 1;}
//		
//		if(first_flag==1&&first_flag_receive==1)
//	{
//		point_2_area.pid_x.fpKp = 1.6;point_2_area.pid_y.fpKp = 1.6;
//		nav.nav_state = NAV_INIT;
//		if(first_step_flag == 0)
//			{
//				if(vision_data_recieve.path_number==2){up_down_state = 1;}
//				else{inner_send[1] = 11;up_down_state = 2;}
//				first_step_flag = 1;
//			}
//		if(up_down_state==0)
//		{
//			inner_send[1] = 0;
//			first_step_flag = 1;
//			if(vision_data_recieve.state_1_repeat == 0){path_state_4 = 5;}			//不吸
//			else{path_state_4 = 3;}								    									//存在侧吸
//		}
//	}			
//			break;
//	
//		case 3://侧吸第一个
//		if(vision_data_recieve.state_1_repeat==1||vision_data_recieve.state_1_repeat==3){inner_send[3] = 1;}//若左侧吸
//		else {inner_send[3] = 3;}//若右侧吸
//		
//		if (!nav_inited){point_only_state = 16;nav.nav_state = NAV_only_point_area2;nav_inited = 1;}
//		if(nav.nav_state == NAV_LOCK_POS)
//			{	
//				choose_action_arm();//选择机械臂 扔 存 持三个动作
//				switch(vision_data_recieve.path_number)//根据路径判断机械臂 高 中 低
//				{
//					case 1:
//						inner_send[2] = 2;
//						break;
//					
//					case 2:
//						inner_send[2] = 1;
//						break;
//					
//					case 3:
//						inner_send[2] = 2;
//						break;
//				}
//				if(inner_send[1]==0){inner_send[1] = 4;action_ing = 1;}
//				if(action_ing==1){if(inner_receive[2] == 3){reach_kfs_num++;path_state_2 = 18;}}
//			}			
//		
//			
//			break;
//		
//		case 4:	//侧吸第二个
//			
//			break;
//		
//		case 5://居中
//		if (!nav_inited)	{point_only_state = 4;nav.nav_state = NAV_only_point_area2;;nav_inited = 1;only_point_range_state = 1;}
//		if(nav.nav_state == NAV_LOCK_POS){only_point_range_state = 0;path_state_4 = 8;}				
//			break;
//		
//		case 6://登第二个台阶
//		nav.nav_state = NAV_INIT;
//		if(first_step_flag==0)
//			{
//				if(vision_data_recieve.path_number==1){up_down_state = 3;}
//				else {up_down_state = 1;}
//				first_step_flag = 1;
//			}
//	
//	if(up_down_state==0)
//		{
//			first_step_flag = 1;
//			if(vision_data_recieve.state_2 == 0){path_state_2 = 11;}				//不吸
//			else if(vision_data_recieve.state_2 == 1){path_state_2 = 10;}		//正吸
//			else{path_state_2 = 9;}																					//存在侧吸
//		}			
//			break;
//		
//		case 7://侧吸第一个
//		if(vision_data_recieve.state_1_repeat==2||vision_data_recieve.state_4==4){inner_send[3] = 1;}//若左侧吸
//		else {inner_send[3] = 3;}//若右侧吸
//		
//		if (!nav_inited){point_only_state = 15;nav.nav_state = NAV_only_point_area2;nav_inited = 1;}
//		if(nav.nav_state == NAV_LOCK_POS)
//			{	
//				choose_action_arm();//选择机械臂 扔 存 持三个动作
//				switch(vision_data_recieve.path_number)//根据路径判断机械臂 高 中 低
//				{
//					case 1:
//						inner_send[2] = 2;
//						break;
//					
//					case 2:
//						inner_send[2] = 1;
//						break;
//					
//					case 3:
//						inner_send[2] = 2;
//						break;
//				}
//				if(inner_send[1]==0){inner_send[1] = 4;action_ing = 1;}
//				if(action_ing==1){if(inner_receive[2] == 3){reach_kfs_num++;path_state_2 = 18;}}
//			}
//			break;
//		
//		case 8://侧吸第二个
//			break;
//		
//		case 9://居中
//		if (!nav_inited)	{point_only_state = 7;nav.nav_state = NAV_only_point_area2;nav_inited = 1;only_point_range_state = 1;}
//		if(nav.nav_state == NAV_LOCK_POS){only_point_range_state = 0;path_state_4 = 12;}		
//			break;
//		
//		case 10://登第三个台阶
//		nav.nav_state = NAV_INIT;
//		if(first_step_flag==0)
//			{
//				if(vision_data_recieve.path_number==3){up_down_state = 3;}
//				else {up_down_state = 1;}
//				first_step_flag = 1;
//			}
//		if(up_down_state==0)
//		{
//			first_step_flag = 1;
//			if(vision_data_recieve.state_3 == 0){path_state_2 = 15;}				//不吸
//			else if(vision_data_recieve.state_3 == 1){path_state_2 = 14;}		//正吸
//			else{path_state_2 = 13;}																					//存在侧吸
//		}
//			break;
//		
//		case 11://侧吸第一个
//			break;
//		
//		case 12://侧吸第二个
//			break;
//		
//		case 13://居中
//		if (!nav_inited)	{point_only_state = 10;nav.nav_state = NAV_only_point_area2;nav_inited = 1;only_point_range_state = 1;}
//		if(nav.nav_state == NAV_LOCK_POS){path_state_4 = 16;only_point_range_state = 0;}		
//			break;
//		
//		case 14://登第四个台阶
//		nav.nav_state = NAV_INIT;
//		if(first_step_flag==0)
//		{up_down_state = 3;first_step_flag = 1;}
//	
//		if(up_down_state==0)
//			{
//				first_step_flag = 1;
//				if(vision_data_recieve.state_4 == 0){path_state_2 = 18;}				//不吸
//				else{path_state_2 = 17;}																				//存在侧吸
//			}
//			break;
//		
//		case 15://侧吸第一个
//			
//			break;
//		
//		case 16://侧吸第二个
//			
//			break;
//		
//		case 17://居中
//		if (!nav_inited)	{point_only_state = 13;nav.nav_state = NAV_only_point_area2;nav_inited = 1;only_point_range_state = 1;}
//		if(nav.nav_state == NAV_LOCK_POS){only_point_range_state = 0;path_state_2 = 19;}				
//			break;
//		
//		case 18://开环登台阶出二区
//			if(inner_send[1] == 0)
//		{
//		nav.nav_state = NAV_INIT;
//		if(first_step_flag==0)
//		{
//			if(vision_data_recieve.path_number==2){inner_send[1] = 11;up_down_state = 4;}
//			else {up_down_state = 3;}
//			first_step_flag = 1;
//		}
//	
//		if(up_down_state==0)
//			{
//				inner_send[1] = 0;
//				path_state_4 = 19;
//			}
//		}
//			break;
//		
//		case 19:
//			
//			break;
//		
//		default:
//			break;
//	}
//}



void all_path_logic(void)
{
	switch (all_path_state)
	{
		case 0://初始化
			break;
		
		case 1://一区导航
		if(first_path_state==0){path_state_1 = 13;first_path_state = 1;}
		path_1();			
			break;
		
		case 2://二区导航
		if(first_path_state == 0){path_state_2 = 1;first_path_state = 1;}
		path_2();			
			break;
		
		case 3://三区导航
		if(first_path_state==0){path_state_3 = 1;first_path_state = 1;}			
		path_3();
			break;
		
		case 4://无法大胜，在一区重试

			break;
		
		case 5://三区导航重试
		if(first_path_state==0){path_state_3 = 8;first_path_state = 1;}			
		path_3();
			break;	
		
		case 6: //三区导航重试  拿两个KFS
		if(first_path_state==0){path_state_3 = 22;first_path_state = 1;}			
		path_3();			
			break;
			
		
		default:
			break;
	
	}
}


//硬件键盘
void start_key(void)
{	
	static uint8_t first_flag = 0;
	static uint8_t all_start_state = 0;
	static uint8_t all_start_flag = 0;
	
	static uint16_t timer_repeat_delay = 0;
	
	if(key_receive[2]==2&&first_flag==0){all_start_state = 1;first_flag = 1;}
	else if(key_receive[2]==3&&first_flag==0){all_start_state = 2;first_flag = 1;}
	else if(key_receive[2]==4&&first_flag==0){all_start_state = 3;inner_send[1] = 19;first_flag = 1;}
	else if(key_receive[2]==5&&first_flag==0) {all_start_state = 4;inner_send[1] = 19;first_flag = 1;}
	else if(key_receive[2]==8){all_start_state=5;}
	else if(key_receive[2]==7){all_start_state=6;}	
	
	if(key_receive[1]==1){all_start_flag = 1;}
	
	
	if(all_start_flag==1)
	{
		switch(all_start_state)
		{
			case 0://初始化
				break;
			
			
			case 1://正常启动
				ctrl_flag.chassis_flag = 1;
				ctrl_flag.dji_flag = 1;
				all_path_state = 1;
				all_start_state = 0;
				break;
			
			case 2://一区不夹头直接重试
				ctrl_flag.chassis_flag = 1;
				ctrl_flag.dji_flag = 1;
				all_path_state = 2;
				all_start_state = 0;
				break;
			
			case 3://三区重试
				ctrl_flag.chassis_flag = 1;
				ctrl_flag.dji_flag = 1;
				all_path_state = 5;
				inner_send[1] = 14;
				all_start_state = 0;
				break;
			
			case 4://三区重试 拿两个KFS
				ctrl_flag.chassis_flag = 1;
				ctrl_flag.dji_flag = 1;
				all_path_state = 6;
				inner_send[1] = 14;
				all_start_state = 0;				
				
				break;
			
			case 5://收腿解锁底盘
				if(key_receive[1]==1)
				{ctrl_flag.chassis_flag = 0;ctrl_flag.dji_flag = 0;all_path_state = 0;up_down_state = 7;}
				break;
				
				
			case 6://放下腿重试
				if(key_receive[1]==1)				
				{ctrl_flag.chassis_flag = 0;ctrl_flag.dji_flag = 0;all_path_state = 0;up_down_state = 8;}				
				break;
			
			default:
				break;
		}
	}
}













//给单纯点到点赋值
void choose_point_only(void)
{
	if(only_point_range_state==0){allow_point.allow_x = 15.0;allow_point.allow_y = 15.0;allow_point.allow_rad = 0.02;} //取KFS点位要求
	else if(only_point_range_state==1){allow_point.allow_x = 150.0;allow_point.allow_y = 200.0;allow_point.allow_rad = 0.5;}	//归中要求
	
	switch(point_only_state)
	{
		case 0://初始化
			break;

		case 1://二区一号入口
		area_two_target.target_x = ENTRY_1_X;
		area_two_target.target_y = ENTRY_1_Y;		
		area_two_target.target_rad = GET_KFS_RAD;		
			break;
		
		case 2://二区二号入口
		area_two_target.target_x = ENTRY_2_X;
		area_two_target.target_y = ENTRY_2_Y;
		area_two_target.target_rad = GET_KFS_RAD;			
			break;
		
		case 3://二区三号入口
		area_two_target.target_x = ENTRY_3_X;
		area_two_target.target_y = ENTRY_3_Y;
		area_two_target.target_rad = GET_KFS_RAD;			
			break;		
	
		case 4://第一个台阶中心点
		area_two_target.target_x = vision_data_recieve.s1_center_x + offset_x;
		area_two_target.target_y = vision_data_recieve.s1_center_y + offset_y;
		area_two_target.target_rad = GET_KFS_RAD;
		break;	
	
		case 5://第一个台阶正吸点
		area_two_target.target_x = vision_data_recieve.s1_x + offset_x;
		area_two_target.target_y = vision_data_recieve.s1_y + offset_y;
		area_two_target.target_rad = GET_KFS_RAD;
		break;	
	
		case 6://第一个台阶侧吸点
	if(vision_data_recieve.state_1==2||vision_data_recieve.state_1==4)
		{
			area_two_target.target_x = vision_data_recieve.s1_side_left_x + offset_x;
			area_two_target.target_y = vision_data_recieve.s1_side_left_y + offset_y;
			area_two_target.target_rad = GET_KFS_RAD;
		}
	else{
				area_two_target.target_x = vision_data_recieve.s1_side_right_x + offset_x;
				area_two_target.target_y = vision_data_recieve.s1_side_right_y + offset_y;
				area_two_target.target_rad = GET_KFS_RAD;
			}
		break;	
	
		case 7://第二个台阶中心点
		area_two_target.target_x = vision_data_recieve.s2_center_x + offset_x;
		area_two_target.target_y = vision_data_recieve.s2_center_y + offset_y;
		area_two_target.target_rad = GET_KFS_RAD;
		break;	
		
		case 8://第二个台阶正吸点
		area_two_target.target_x = vision_data_recieve.s2_x + offset_x;
		area_two_target.target_y = vision_data_recieve.s2_y + offset_y;
		area_two_target.target_rad = GET_KFS_RAD;		
		break;
	
		case 9://第二个台阶侧吸点
	if(vision_data_recieve.state_2==2||vision_data_recieve.state_2==4)
		{
			area_two_target.target_x = vision_data_recieve.s2_side_left_x + offset_x;
			area_two_target.target_y = vision_data_recieve.s2_side_left_y + offset_y;
			area_two_target.target_rad = GET_KFS_RAD;
		}
	else{
				area_two_target.target_x = vision_data_recieve.s2_side_right_x + offset_x;
				area_two_target.target_y = vision_data_recieve.s2_side_right_y + offset_y;
				area_two_target.target_rad = GET_KFS_RAD;
			}
		break;
	
		case 10://第三个台阶中心点
		area_two_target.target_x = vision_data_recieve.s3_center_x + offset_x;
		area_two_target.target_y = vision_data_recieve.s3_center_y + offset_y;
		area_two_target.target_rad = GET_KFS_RAD;
		break;
	
		case 11://第三个台阶正吸点
		area_two_target.target_x = vision_data_recieve.s3_x + offset_x;
		area_two_target.target_y = vision_data_recieve.s3_y + offset_y;
		area_two_target.target_rad = GET_KFS_RAD;
		break;
	
		case 12://第三个台阶侧吸点
	if(vision_data_recieve.state_3==2||vision_data_recieve.state_3==4)
		{
			area_two_target.target_x = vision_data_recieve.s3_side_left_x + offset_x;
			area_two_target.target_y = vision_data_recieve.s3_side_left_y + offset_y;
			area_two_target.target_rad = GET_KFS_RAD;
		}
	else{
				area_two_target.target_x = vision_data_recieve.s3_side_right_x + offset_x;
				area_two_target.target_y = vision_data_recieve.s3_side_right_y + offset_y;
				area_two_target.target_rad = GET_KFS_RAD;
			}
		break;
	
		case 13://第四个台阶中心点
		area_two_target.target_x = vision_data_recieve.s4_center_x + offset_x;
		area_two_target.target_y = vision_data_recieve.s4_center_y + offset_y;
		area_two_target.target_rad = GET_KFS_RAD;
		break;
	
		case 14://第四个台阶正吸点（占位）
		area_two_target.target_x = vision_data_recieve.s4_x + offset_x;
		area_two_target.target_y = vision_data_recieve.s4_y + offset_y;
		area_two_target.target_rad = GET_KFS_RAD;
		break;

		case 15://第四个台阶侧吸点
	if(vision_data_recieve.state_4==2||vision_data_recieve.state_4==4)
		{
			area_two_target.target_x = vision_data_recieve.s4_side_left_x + offset_x;
			area_two_target.target_y = vision_data_recieve.s4_side_left_y + offset_y;
			area_two_target.target_rad = GET_KFS_RAD;
		}
	else{
				area_two_target.target_x = vision_data_recieve.s4_side_right_x + offset_x;
				area_two_target.target_y = vision_data_recieve.s4_side_right_y + offset_y;
				area_two_target.target_rad = GET_KFS_RAD;
			}
		break;	
			
		case 16://重试 第一个台阶第一个侧吸点（若吸两个，第一个为左吸）
	if(vision_data_recieve.state_1_repeat==1||vision_data_recieve.state_1_repeat==3)
		{
			area_two_target.target_x = vision_data_recieve.s1_side_left_x + offset_x;
			area_two_target.target_y = vision_data_recieve.s1_side_left_y + offset_y;
			area_two_target.target_rad = GET_KFS_RAD;
		}
	else{
				area_two_target.target_x = vision_data_recieve.s1_side_right_x + offset_x;
				area_two_target.target_y = vision_data_recieve.s1_side_right_y + offset_y;
				area_two_target.target_rad = GET_KFS_RAD;
			}			
			break;
		
		case 17://重试 第一个台阶第二个侧吸点,只会是右侧吸
		area_two_target.target_x = vision_data_recieve.s1_side_right_x + offset_x;
		area_two_target.target_y = vision_data_recieve.s1_side_right_y + offset_y;
		area_two_target.target_rad = GET_KFS_RAD;				
			break;
		
		case 18://重试 第二个台阶第一个侧吸点
	if(vision_data_recieve.state_2_repeat==1||vision_data_recieve.state_2_repeat==3)
		{
			area_two_target.target_x = vision_data_recieve.s2_side_left_x + offset_x;
			area_two_target.target_y = vision_data_recieve.s2_side_left_y + offset_y;
			area_two_target.target_rad = GET_KFS_RAD;
		}
	else{
				area_two_target.target_x = vision_data_recieve.s2_side_right_x + offset_x;
				area_two_target.target_y = vision_data_recieve.s2_side_right_y + offset_y;
				area_two_target.target_rad = GET_KFS_RAD;
			}			
			break;
		
		case 19://重试 第二个台阶第二个侧吸点
		area_two_target.target_x = vision_data_recieve.s2_side_right_x + offset_x;
		area_two_target.target_y = vision_data_recieve.s2_side_right_y + offset_y;
		area_two_target.target_rad = GET_KFS_RAD;		
			break;
		
		case 20://重试 第三个台阶第一个侧吸点
	if(vision_data_recieve.state_3_repeat==1||vision_data_recieve.state_3_repeat==3)
		{
			area_two_target.target_x = vision_data_recieve.s3_side_left_x + offset_x;
			area_two_target.target_y = vision_data_recieve.s3_side_left_y + offset_y;
			area_two_target.target_rad = GET_KFS_RAD;
		}
	else{
				area_two_target.target_x = vision_data_recieve.s3_side_right_x + offset_x;
				area_two_target.target_y = vision_data_recieve.s3_side_right_y + offset_y;
				area_two_target.target_rad = GET_KFS_RAD;
			}			
			break;
		
		case 21://重试 第三个台阶第二个侧吸点
		area_two_target.target_x = vision_data_recieve.s3_side_right_x + offset_x;
		area_two_target.target_y = vision_data_recieve.s3_side_right_y + offset_y;
		area_two_target.target_rad = GET_KFS_RAD;		
			break;	
		
		case 22://重试 第四个台阶第一个侧吸点
	if(vision_data_recieve.state_4_repeat==1||vision_data_recieve.state_4_repeat==3)
		{
			area_two_target.target_x = vision_data_recieve.s4_side_left_x + offset_x;
			area_two_target.target_y = vision_data_recieve.s4_side_left_y + offset_y;
			area_two_target.target_rad = GET_KFS_RAD;
		}
	else{
				area_two_target.target_x = vision_data_recieve.s4_side_right_x + offset_x;
				area_two_target.target_y = vision_data_recieve.s4_side_right_y + offset_y;
				area_two_target.target_rad = GET_KFS_RAD;
			}			
			break;
		
		case 23://重试 第四个台阶第二个侧吸点
		area_two_target.target_x = vision_data_recieve.s4_side_right_x + offset_x;
		area_two_target.target_y = vision_data_recieve.s4_side_right_y + offset_y;
		area_two_target.target_rad = GET_KFS_RAD;		
			break;	
		
		case 24://1号台阶在一区映射
		area_two_target.target_x = ENTRY_ONE_AREA_1X;
		area_two_target.target_y = ENTRY_ONE_AREA_1Y;		
		area_two_target.target_rad = GET_KFS_RAD;					
			break;
		
		case 25://2号台阶在一区映射
		area_two_target.target_x = ENTRY_ONE_AREA_2X;
		area_two_target.target_y = ENTRY_ONE_AREA_2Y;		
		area_two_target.target_rad = GET_KFS_RAD;					
			break;
		
		case 26: //3号台阶在一区映射
		area_two_target.target_x = ENTRY_ONE_AREA_3X;
		area_two_target.target_y = ENTRY_ONE_AREA_3Y;		
		area_two_target.target_rad = GET_KFS_RAD;					
			break;
	
	default:
		break;
	}
}
