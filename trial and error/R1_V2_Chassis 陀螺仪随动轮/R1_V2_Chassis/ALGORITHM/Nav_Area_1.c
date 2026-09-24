#include "Nav_Area_1.h"


void Nav_Area_1_Task(void){
//        chassis_run.pid_state = VELT_LOOP;
//		chassis_run.feed_forward_state = WITHOUT_FORWARD;
	
		switch (state1)
		{
			
			
	//平移一步达到取杆位置		
		case 0:
			point_end.x = 433;
			point_end.y = 1527;//2272-28;//2055;//2080+25;
			point_end.q = 0.446633625;
			NavLineMoveWithHeading(&nav);
			break;
	//旋转加平移，一步到达存杆位置		
		case 1:
			point_end.x = 600; 
			point_end.y = 2500;//2055;//2080+25;
			point_end.q = 0.446633625;
			NavLineMoveWithHeading(&nav);
			break;

	//前进到达对接位置			
		case 2:
			point_end.x = 171;//343+7;//318; 
			point_end.y = 3863.;//2242//2080+25;
			point_end.q = -90.7887085;
			NavLineMoveWithHeading(&nav);
			break;
		
		
	//回退，旋转，预备进入二区	
		case 3:
			point_end.x = 800; 
			point_end.y = 2575;
			point_end.q = 0;
			NavLineMoveWithHeading(&nav);
		
			break;

		
		
		default:
			break;
		}
		
		
		
		
		Nav_PID_Adjust();
		
		
		
		
		
		
		
		
		
		
		
		PID_Calc_New(&nav.auto_path.pos_pid.x);
		PID_Calc_New(&nav.auto_path.pos_pid.y);
		PID_Calc_New(&nav.auto_path.pos_pid.w);

//		nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.x.fpU  /*nav.auto_path.basic_velt.fpVx*/;
//		nav.expect_robot_global_velt.fpY = nav.auto_path.pos_pid.y.fpU  /*nav.auto_path.basic_velt.fpVy*/;
//		nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU  /*nav.auto_path.basic_velt.fpW*/;
		
//		nav.expect_robot_global_velt.fpX = (float)0.3*nav.auto_path.pos_pid.x.fpU + (float)0.7*nav.auto_path.basic_velt.fpVx;
//		nav.expect_robot_global_velt.fpY = (float)0.3*nav.auto_path.pos_pid.y.fpU + (float)0.7*nav.auto_path.basic_velt.fpVy;
////		nav.expect_robot_global_velt.fpW = (float)0.3*nav.auto_path.pos_pid.w.fpU + (float)0.7*nav.auto_path.basic_velt.fpW;
//		nav.expect_robot_global_velt.fpW = (float)0.5*nav.auto_path.pos_pid.w.fpU + (float)0.5*nav.auto_path.basic_velt.fpW;


		nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.x.fpU + nav.auto_path.basic_velt.fpVx;
		nav.expect_robot_global_velt.fpY = nav.auto_path.pos_pid.y.fpU + nav.auto_path.basic_velt.fpVy;
		nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU + nav.auto_path.basic_velt.fpW;
		
		
		SpeedDistribute_Four_OmnidriectionalWhile(&nav);
    }
