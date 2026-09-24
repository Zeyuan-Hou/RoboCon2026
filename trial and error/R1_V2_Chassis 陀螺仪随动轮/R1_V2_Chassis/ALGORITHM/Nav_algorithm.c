#include "Nav_algorithm.h"

void NavLineMove_VelocityControl(ST_Nav *p_nav, fp32 v_start, fp32 v_end)//目标角度直接锁死不规划
{
	static fp32 Vmax = 2000.0f;
	static fp32 Amax = 1000.0f;
	static fp32 alpha;

	// 路径初始化（完全保留你风格）
	if (flag_record)
	{
		nav.auto_path.run_time = 0;

		StartX = p_nav->auto_path.pos_pid.x.fpFB;
		StartY = p_nav->auto_path.pos_pid.y.fpFB;
		StartQ = p_nav->auto_path.pos_pid.w.fpFB;

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;

		DELTA_X = point_end.x - StartX;
		DELTA_Y = point_end.y - StartY;
		DELTA_Q = point_end.q - StartQ;

		flag_record = 0;
	}

	t_run = Ts * p_nav->auto_path.run_time;
	fp32 Total_Distance = Geometric_mean(DELTA_X, DELTA_Y);



	alpha = atan2f(DELTA_Y, DELTA_X);

//梯形速度位置规划
	fp32 v0 = v_start;
	fp32 v1 = v_end;
	if (v0 > Vmax) v0 = Vmax;
	if (v1 > Vmax) v1 = Vmax;

	fp32 s_acc = (Vmax*Vmax - v0*v0) / (2.0f * Amax);
	fp32 s_dec = (Vmax*Vmax - v1*v1) / (2.0f * Amax);
	fp32 v_peak = Vmax;

	if (s_acc + s_dec > Total_Distance)
	{
		v_peak = sqrtf((2*Amax*Total_Distance + v0*v0 + v1*v1) / 2.0f);
		s_acc = (v_peak*v_peak - v0*v0) / (2*Amax);
		s_dec = (v_peak*v_peak - v1*v1) / (2*Amax);
	}

	fp32 t_acc = (v_peak - v0) / Amax;
	fp32 t_dec = (v_peak - v1) / Amax;
	fp32 t_const = (Total_Distance - s_acc - s_dec) / v_peak;
	if (t_const < 0) t_const = 0;

	fp32 Total_Time = t_acc + t_const + t_dec;//计算总时长

	
	
	
	
	
	
	fp32 s_des = 0.0f;
	fp32 vel_des = 0.0f;

	if (t_run <= 0)//边界判断
	{
		s_des = 0;
		vel_des = v0;
	}
		else if (t_run <= t_acc)
	{
		vel_des = v0 + Amax * t_run;
		s_des = v0*t_run + 0.5f*Amax*t_run*t_run;
	}
		else if (t_run <= t_acc + t_const)
	{
		vel_des = v_peak;
		s_des = s_acc + v_peak * (t_run - t_acc);
	}
		else if (t_run <= Total_Time)
	{
		fp32 t_b = t_run - (t_acc + t_const);
		vel_des = v_peak - Amax * t_b;
		s_des = s_acc + v_peak*t_const + v_peak*t_b - 0.5f*Amax*t_b*t_b;
	}
	else//边界判断
	{
		s_des = Total_Distance;
		vel_des = v1;
	}




	// 位置输出
	p_nav->auto_path.pos_pid.x.fpDes = StartX + s_des * cosf(alpha);
	p_nav->auto_path.pos_pid.y.fpDes = StartY + s_des * sinf(alpha);

	// 速度输出（完全无跳变）
	p_nav->auto_path.basic_velt.fpVx = vel_des * cosf(alpha);
	p_nav->auto_path.basic_velt.fpVy = vel_des * sinf(alpha);

      //角度锁死
	p_nav->auto_path.pos_pid.w.fpDes = point_end.q;
	p_nav->auto_path.basic_velt.fpW = 0;
	
	if(t_run>=Total_Time)
	{
        if(one_turn_state>=1)
        {  
        if(one_turn_state==3)//先判断是否已经完成路径
        {spot_pre=spot;
        //下面重置一下    
        one_turn_state=0;
        two_turn_state=0;

        flag_record=1;
        nav.auto_path.run_time=0;
        }//也就是走完了单转弯换区
        else
        {one_turn_state+=1;
         flag_record=1;
         nav.auto_path.run_time=0;}
        }
    
     
        else if(two_turn_state>=1)
        {  
        if(two_turn_state==5)//先判断是否已经完成路径
        {spot_pre=spot;
        //下面重置一下    
        one_turn_state=0;
        two_turn_state=0;

        flag_record=1;
        nav.auto_path.run_time=0;
        }//也就是走完了双转弯换区
        else
        {two_turn_state+=1;
         flag_record=1;
         nav.auto_path.run_time=0;}
        }
}

}

void NavLineMove(ST_Nav *p_nav) //一段任意方向，任意大小的直线，不改变朝向，锁角度。所以不要乱用这个，车会直接转到对应角度//有到位判断
{
	
	static fp32 Vmax, Amax;
	static fp32 alpha;
	
	
	
//flag_record=1时开始走一段新的路径	
if (flag_record){


   //清空运行时间	
   nav.auto_path.run_time = 0;

  //记录初始位置和角度
	StartX = p_nav->auto_path.pos_pid.x.fpFB;
	StartY = p_nav->auto_path.pos_pid.y.fpFB;
	StartQ = p_nav->auto_path.pos_pid.w.fpFB;

	//清空位置环pid Error积累
	p_nav->auto_path.pos_pid.x.fpSumE = 0;
	p_nav->auto_path.pos_pid.y.fpSumE = 0;
	p_nav->auto_path.pos_pid.w.fpSumE = 0;

	//计算目标和实际差
	DELTA_X = point_end.x - StartX;
	DELTA_Y = point_end.y - StartY;
	DELTA_Q = point_end.q - StartQ;
  

	
	flag_record = 0;
}
	

	
	
	Vmax = 3000;//设定最大线速度mm/s
	Amax = 1000;//设定最大线加速度

	alpha = atan2f(DELTA_Y, DELTA_X);
	
	t_run = Ts * p_nav->auto_path.run_time;
		
		
		// 计算总距离
	fp32 Total_Distance = Geometric_mean(DELTA_X, DELTA_Y);



	// 计算总时间
	fp32 Total_Time = sqrtf(4 * Total_Distance / Amax);
	if (Total_Distance > 0.1f) 
		{
		fp32 t_vel_limit = Total_Distance / Vmax;//计算最短时间限制
		if (Total_Time < t_vel_limit) Total_Time = t_vel_limit;
	}

	// 归一化时间 0~1,获取t_norm
	fp32 t_norm = t_run / Total_Time;
	if (t_norm > 1.0f) t_norm = 1.0f;
	
	
	

//	// S 形五次多项式
//	fp32 t3 = t_norm * t_norm * t_norm;
//	fp32 t4 = t3 * t_norm;
//	fp32 t5 = t4 * t_norm;
//	fp32 s = 10.0f * t3 - 15.0f * t4 + 6.0f * t5;


	// 三次S曲线（更快、更滑、更稳）
	fp32 t_2 = t_norm * t_norm;
	fp32 t_3 = t_2 * t_norm;
	fp32 s = 3.0f * t_2 - 2.0f * t_3;
	
	
	
		




			
			//角度强制归位
			p_nav->auto_path.pos_pid.w.fpDes = point_end.q;
			p_nav->auto_path.basic_velt.fpW = 0;
		


	
	
	
	
	
float err_x = fabs(point_end.x - p_nav->auto_path.pos_pid.x.fpFB);
float err_y = fabs(point_end.y - p_nav->auto_path.pos_pid.y.fpFB);
float err = sqrtf(err_x*err_x + err_y*err_y);

	
	
	// S 形位置输出
	if (t_run <= Total_Time)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + s * DELTA_X;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + s * DELTA_Y;

		// ========================
		// 三次S速度输出（平滑、快、无冲击）
		// ========================
		fp32 s_vel = 6.0f * t_norm - 6.0f * t_2;
		fp32 vel = s_vel * Total_Distance / Total_Time;
		// 2. S 型非线性衰减（真正柔停，不是线性！）
    float decel_dis = 200.0f;  // 开始减速的距离
    float ratio = err / decel_dis;
    if(ratio > 1.0f) ratio = 1.0f;
		
		float feedrate = ratio * ratio * (3.0f - 2.0f * ratio);
    // 下限保护，不让速度卡死
		
		vel *= feedrate;
if(feedrate < 0.04f) feedrate = 0.04f;
		p_nav->auto_path.basic_velt.fpVx = vel * cosf(alpha);
		p_nav->auto_path.basic_velt.fpVy = vel * sinf(alpha);
	}
	else if (t_run > Total_Time)
	{
//		p_nav->auto_path.run_time = 0;
//        flag_record=1;
	
		p_nav->auto_path.pos_pid.x.fpDes = point_end.x;
		p_nav->auto_path.pos_pid.y.fpDes = point_end.y;
		
		p_nav->auto_path.basic_velt.fpVx = 0;
		p_nav->auto_path.basic_velt.fpVy = 0;
	}
	
	

		
		
		if (fabs(stRobot.stPos.fpPosX - point_end.x) <= 200 && fabs(stRobot.stPos.fpPosY - point_end.y) <= 200 && fabs(nav.auto_path.pos_pid.w.fpFB - point_end.q) <= 20)
		{ 
			
			
			
			

	
//			p_nav->auto_path.basic_velt.fpVx = 0;
//		  p_nav->auto_path.basic_velt.fpVy = 0;
//			p_nav->auto_path.basic_velt.fpW  = 0;

			
			//如果是下面这种自动切换路径的状态就是自动刷新flag_record,不自动刷新的话就是定点，目标值不变
			if (nav.nav_state == NAV_AREA_3&&(state3!=3))
			{ 
				
							//为下一次做准备
		    p_nav->auto_path.run_time = 0;
			flag_record=1;
			
			//清零
		  p_nav->auto_path.pos_pid.x.fpSumE = 0;
		  p_nav->auto_path.pos_pid.y.fpSumE = 0;
		  p_nav->auto_path.pos_pid.w.fpSumE = 0;
							p_nav->auto_path.basic_velt.fpVx = 0;
		  p_nav->auto_path.basic_velt.fpVy = 0;
			p_nav->auto_path.basic_velt.fpW  = 0;
			state3++;
			}			
			else if (nav.nav_state == NAV_AREA_2 && enter != 1)
			{
				enter = 1;
			}
			else if (nav.nav_state == NAV_AREA_3_RESET&&state3_RESET==0) {
				
				p_nav->auto_path.run_time = 0;
		  flag_record=1;
			
			//清零
		  p_nav->auto_path.pos_pid.x.fpSumE = 0;
		  p_nav->auto_path.pos_pid.y.fpSumE = 0;
			p_nav->auto_path.pos_pid.w.fpSumE = 0;
							p_nav->auto_path.basic_velt.fpVx = 0;
		  p_nav->auto_path.basic_velt.fpVy = 0;
			p_nav->auto_path.basic_velt.fpW  = 0;
				
			state3_RESET++;}



		  if (nav.nav_state == NAV_AREA_2 && spot != spot_pre)
			{
				spot_pre = spot;
			}



			
			
		}
		
	}








void NavLineMoveWithHeading(ST_Nav *p_nav)//这个是有到位判断的 //一段任意方向，任意大小的直线，同时改变朝向，用于执行动作前的校准	//总时间受路径长度的影响，因此如果路径长度很短，时间就会很短。假如此时转角比较大，就会转的特别快(关于旋转部分已经被修改)
{
	
	static fp32 Vmax, Amax;
	static fp32 alpha;
	
	
	
//flag_record=1时开始走一段新的路径	
if (flag_record){


   //清空运行时间	
   nav.auto_path.run_time = 0;

  //记录初始位置和角度
	StartX = p_nav->auto_path.pos_pid.x.fpFB;
	StartY = p_nav->auto_path.pos_pid.y.fpFB;
	StartQ = p_nav->auto_path.pos_pid.w.fpFB;

	//清空位置环pid Error积累
	p_nav->auto_path.pos_pid.x.fpSumE = 0;
	p_nav->auto_path.pos_pid.y.fpSumE = 0;
	p_nav->auto_path.pos_pid.w.fpSumE = 0;

	//计算目标和实际差
	DELTA_X = point_end.x - StartX;
	DELTA_Y = point_end.y - StartY;
	DELTA_Q = point_end.q - StartQ;
  

	
	flag_record = 0;
}
	

	
	
	Vmax = 2000;//设定最大线速度mm/s
	Amax = 1000;//设定最大线加速度

	alpha = atan2f(DELTA_Y, DELTA_X);
	
	t_run = Ts * p_nav->auto_path.run_time;
		
		
		// 计算总距离
	fp32 Total_Distance = Geometric_mean(DELTA_X, DELTA_Y);



	// 计算总时间
	fp32 Total_Time = sqrtf(4 * Total_Distance / Amax);
	if (Total_Distance > 0.1f) 
		{
		fp32 t_vel_limit = Total_Distance / Vmax;//计算最短时间限制
		if (Total_Time < t_vel_limit) Total_Time = t_vel_limit;
	}

	// 归一化时间 0~1,获取t_norm
	fp32 t_norm = t_run / Total_Time;
	if (t_norm > 1.0f) t_norm = 1.0f;
	
	
	

//	// S 形五次多项式
//	fp32 t3 = t_norm * t_norm * t_norm;
//	fp32 t4 = t3 * t_norm;
//	fp32 t5 = t4 * t_norm;
//	fp32 s = 10.0f * t3 - 15.0f * t4 + 6.0f * t5;


	// 三次S曲线（更快、更滑、更稳）
	fp32 t_2 = t_norm * t_norm;
	fp32 t_3 = t_2 * t_norm;
	fp32 s = 3.0f * t_2 - 2.0f * t_3;
	
	
	
		





// 角度 三次S形 平滑旋转
    if((Total_Time) > 0.25f) 
	{
		if(t_run <= Total_Time)
		{
			// 角度三次S
			fp32 s_rot = 3.0f * t_2 - 2.0f * t_3;
			fp32 s_vel_w = 6.0f * t_norm - 6.0f * t_2;
			
			// 角度目标
			p_nav->auto_path.pos_pid.w.fpDes = StartQ + DELTA_Q * s_rot;
			// 角速度S形
			p_nav->auto_path.basic_velt.fpW = (DELTA_Q / Total_Time) * s_vel_w;
		}
		else
		{
			// 超时 → 强制归位
			p_nav->auto_path.pos_pid.w.fpDes = point_end.q;
			p_nav->auto_path.basic_velt.fpW = 0;
			p_nav->auto_path.pos_pid.w.fpSumE = 0;
		}
	}
	
	
	
	else 
	{
		if(t_run <= 0.25f)
		{
			fp32 t_min_norm = t_run / 0.25f;
			if(t_min_norm > 1.0f) t_min_norm = 1.0f;
			
			fp32 t2_min = t_min_norm * t_min_norm;
			fp32 t3_min = t2_min * t_min_norm;
			fp32 s_rot_min = 3.0f * t2_min - 2.0f * t3_min;
			fp32 s_vel_w_min = 6.0f * t_min_norm - 6.0f * t2_min;
			
			p_nav->auto_path.pos_pid.w.fpDes = StartQ + DELTA_Q * s_rot_min;
			p_nav->auto_path.basic_velt.fpW = (DELTA_Q / 0.25f) * s_vel_w_min;
		}
		else
		{
			// 超时 → 强制归位
			p_nav->auto_path.pos_pid.w.fpDes = point_end.q;
			p_nav->auto_path.basic_velt.fpW = 0;
		}
	}

float err_x = fabs(point_end.x - p_nav->auto_path.pos_pid.x.fpFB);
float err_y = fabs(point_end.y - p_nav->auto_path.pos_pid.y.fpFB);
float err = sqrtf(err_x*err_x + err_y*err_y);

	
	
	// S 形位置输出
	if (t_run <= Total_Time)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX + s * DELTA_X;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + s * DELTA_Y;

		// ========================
		// 三次S速度输出（平滑、快、无冲击）
		// ========================
		fp32 s_vel = 6.0f * t_norm - 6.0f * t_2;
		fp32 vel = s_vel * Total_Distance / Total_Time;
		// 2. S 型非线性衰减（真正柔停，不是线性！）
    float decel_dis = 200.0f;  // 开始减速的距离
    float ratio = err / decel_dis;
    if(ratio > 1.0f) ratio = 1.0f;
		
		float feedrate = ratio * ratio * (3.0f - 2.0f * ratio);
    // 下限保护，不让速度卡死
		
		vel *= feedrate;
if(feedrate < 0.04f) feedrate = 0.04f;
		p_nav->auto_path.basic_velt.fpVx = vel * cosf(alpha);
		p_nav->auto_path.basic_velt.fpVy = vel * sinf(alpha);
	}
	else if (t_run > Total_Time)
	{
//		p_nav->auto_path.run_time = 0;
//        flag_record=1;
	
		p_nav->auto_path.pos_pid.x.fpDes = point_end.x;
		p_nav->auto_path.pos_pid.y.fpDes = point_end.y;
		
		p_nav->auto_path.basic_velt.fpVx = 0;
		p_nav->auto_path.basic_velt.fpVy = 0;
	}
	
	

		
		
		if (fabs(stRobot.stPos.fpPosX - point_end.x) <= 200 && fabs(stRobot.stPos.fpPosY - point_end.y) <= 200 && fabs(nav.auto_path.pos_pid.w.fpFB - point_end.q) <= 20)
		{ 
			
			
			
			

	
//			p_nav->auto_path.basic_velt.fpVx = 0;
//		  p_nav->auto_path.basic_velt.fpVy = 0;
//			p_nav->auto_path.basic_velt.fpW  = 0;

			
			//如果是下面这种自动切换路径的状态就是自动刷新flag_record,不自动刷新的话就是定点，目标值不变
			if (nav.nav_state == NAV_AREA_3&&(state3<=2))
			{ 
				
							//为下一次做准备
		    p_nav->auto_path.run_time = 0;
			flag_record=1;
			
			//清零
		  p_nav->auto_path.pos_pid.x.fpSumE = 0;
		  p_nav->auto_path.pos_pid.y.fpSumE = 0;
		  p_nav->auto_path.pos_pid.w.fpSumE = 0;
			p_nav->auto_path.basic_velt.fpVx = 0;
		  p_nav->auto_path.basic_velt.fpVy = 0;
			p_nav->auto_path.basic_velt.fpW  = 0;
			state3++;
			}
			else 	if (nav.nav_state == NAV_AREA_3_SINGLE&&(state3_SINGLE<=2))
			{ 
				
							//为下一次做准备
		    p_nav->auto_path.run_time = 0;
			flag_record=1;
			
			//清零
		  p_nav->auto_path.pos_pid.x.fpSumE = 0;
		  p_nav->auto_path.pos_pid.y.fpSumE = 0;
		  p_nav->auto_path.pos_pid.w.fpSumE = 0;
			p_nav->auto_path.basic_velt.fpVx = 0;
		  p_nav->auto_path.basic_velt.fpVy = 0;
			p_nav->auto_path.basic_velt.fpW  = 0;
			state3_SINGLE++;
			}




		// 	else if (nav.nav_state == NAV_AREA_1)//一区不再需要自动前进了，每个都是一步
		// 	{ 							
		// 		//为下一次做准备
		//   p_nav->auto_path.run_time = 0;
		//   flag_record=1;
			
		// 	//清零
		//   p_nav->auto_path.pos_pid.x.fpSumE = 0;
		//   p_nav->auto_path.pos_pid.y.fpSumE = 0;
		// 	p_nav->auto_path.pos_pid.w.fpSumE = 0;
		// 					p_nav->auto_path.basic_velt.fpVx = 0;
		//   p_nav->auto_path.basic_velt.fpVy = 0;
		// 	p_nav->auto_path.basic_velt.fpW  = 0;
		// 	state1++;
		// 	}
			





			else if (nav.nav_state == NAV_AREA_2 && enter != 1)
			{
				enter = 1;
			}
			else if (nav.nav_state == NAV_AREA_3_RESET&&state3_RESET==0) {
				
				p_nav->auto_path.run_time = 0;
		  flag_record=1;
			
			//清零
		  p_nav->auto_path.pos_pid.x.fpSumE = 0;
		  p_nav->auto_path.pos_pid.y.fpSumE = 0;
			p_nav->auto_path.pos_pid.w.fpSumE = 0;
							p_nav->auto_path.basic_velt.fpVx = 0;
		  p_nav->auto_path.basic_velt.fpVy = 0;
			p_nav->auto_path.basic_velt.fpW  = 0;
				
			state3_RESET++;}



		  if (nav.nav_state == NAV_AREA_2 && spot != spot_pre)
			{
				spot_pre = spot;
			}



			
			
		}
		
	}





void Nav_Uphill(ST_Nav *p_nav, Speed *speed) //上坡没有定位只控速度//没有到位判断，时间到就结束
{ 
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	if(nav.nav_state==NAV_AREA_3){
	  uphilling = 1;
	
	if (first_up == 0)
	{
		p_nav->auto_path.run_time = 0;
		first_up = 1;
	}

	t_run = Ts * p_nav->auto_path.run_time;
	fp32 Vmax = speed->a1 * speed->t1;
	fp32 t3 = Vmax / speed->a2;
	if (t_run < speed->t1)
	{
		p_nav->auto_path.basic_velt.fpVx = speed->a1 * cos(speed->alpha) * t_run;
		p_nav->auto_path.basic_velt.fpVy = speed->a1 * sin(speed->alpha) * t_run;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else if (t_run < speed->t1 + speed->t2)
	{

		p_nav->auto_path.basic_velt.fpVx = Vmax * cos(speed->alpha);
		p_nav->auto_path.basic_velt.fpVy = Vmax * sin(speed->alpha);
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else if (t_run < speed->t1 + speed->t2 + t3)
	{

		p_nav->auto_path.basic_velt.fpVx = speed->a2 * cos(speed->alpha) * (speed->t1 + speed->t2 + t3 - t_run);
		p_nav->auto_path.basic_velt.fpVy = speed->a2 * sin(speed->alpha) * (speed->t1 + speed->t2 + t3 - t_run);
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else if (t_run < speed->t1 + speed->t2 + t3 + 0.1f)
	{
		p_nav->auto_path.basic_velt.fpVx = 0;
		p_nav->auto_path.basic_velt.fpVy = 0;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else//时间结束而结束
	{
		p_nav->auto_path.run_time = 0;
		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;

		nav.auto_path.pos_pid.x.fpDes = nav.auto_path.pos_pid.x.fpFB;
		nav.auto_path.pos_pid.y.fpDes = nav.auto_path.pos_pid.y.fpFB;
		nav.auto_path.pos_pid.w.fpDes = nav.auto_path.pos_pid.w.fpFB;
		

		uphill = 1;
		uphilling = 0;
		state3++;
	}
	
	if(test_stRobot.stPos.fpPosZ>440/*&&(fabs(gyro_data.roll)<15)&&(fabs(gyro_data.pitch)<15)*/)//位置差不多了就结束//这个位置还要测
	{
		p_nav->auto_path.run_time = 0;
		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;

		nav.auto_path.pos_pid.x.fpDes = nav.auto_path.pos_pid.x.fpFB;
		nav.auto_path.pos_pid.y.fpDes = nav.auto_path.pos_pid.y.fpFB;
		nav.auto_path.pos_pid.w.fpDes = nav.auto_path.pos_pid.w.fpFB;	
		
		uphill = 1;
		uphilling = 0;
		state3++;
	}	
}
	




















	 else if(nav.nav_state==NAV_AREA_3_SINGLE){
	  uphilling = 1;
	
	if (first_up == 0)
	{
		p_nav->auto_path.run_time = 0;
		first_up = 1;
	}

	t_run = Ts * p_nav->auto_path.run_time;
	fp32 Vmax = speed->a1 * speed->t1;
	fp32 t3 = Vmax / speed->a2;
	if (t_run < speed->t1)
	{
		p_nav->auto_path.basic_velt.fpVx = speed->a1 * cos(speed->alpha) * t_run;
		p_nav->auto_path.basic_velt.fpVy = speed->a1 * sin(speed->alpha) * t_run;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else if (t_run < speed->t1 + speed->t2)
	{

		p_nav->auto_path.basic_velt.fpVx = Vmax * cos(speed->alpha);
		p_nav->auto_path.basic_velt.fpVy = Vmax * sin(speed->alpha);
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else if (t_run < speed->t1 + speed->t2 + t3)
	{

		p_nav->auto_path.basic_velt.fpVx = speed->a2 * cos(speed->alpha) * (speed->t1 + speed->t2 + t3 - t_run);
		p_nav->auto_path.basic_velt.fpVy = speed->a2 * sin(speed->alpha) * (speed->t1 + speed->t2 + t3 - t_run);
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else if (t_run < speed->t1 + speed->t2 + t3 + 0.1f)
	{
		p_nav->auto_path.basic_velt.fpVx = 0;
		p_nav->auto_path.basic_velt.fpVy = 0;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else//时间结束而结束
	{
		p_nav->auto_path.run_time = 0;
		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;

		nav.auto_path.pos_pid.x.fpDes = nav.auto_path.pos_pid.x.fpFB;
		nav.auto_path.pos_pid.y.fpDes = nav.auto_path.pos_pid.y.fpFB;
		nav.auto_path.pos_pid.w.fpDes = nav.auto_path.pos_pid.w.fpFB;
		

		uphill = 1;
		uphilling = 0;
		state3_SINGLE++;
	}
	
	if(test_stRobot.stPos.fpPosZ>350/*&&(fabs(gyro_data.roll)<15)&&(fabs(gyro_data.pitch)<15)*/)//位置差不多了就结束//这个位置还要测
	{
		p_nav->auto_path.run_time = 0;
		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;

		nav.auto_path.pos_pid.x.fpDes = nav.auto_path.pos_pid.x.fpFB;
		nav.auto_path.pos_pid.y.fpDes = nav.auto_path.pos_pid.y.fpFB;
		nav.auto_path.pos_pid.w.fpDes = nav.auto_path.pos_pid.w.fpFB;	
		
		uphill = 1;
		uphilling = 0;
		state3_SINGLE++;
	}	
}












}



void Nav_Rotation(ST_Nav *p_nav, float angle)//上坡后角度自动纠正//这里的angle是总转角//有到位判断
{
	
	static fp32  W;
	if(flag_record)
	{
		p_nav->auto_path.run_time = 0;
		StartQ = p_nav->auto_path.pos_pid.w.fpFB;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
		DELTA_Q = angle - StartQ;
		W = DELTA_Q;
		flag_record = 0;
	}

	t_run = Ts * p_nav->auto_path.run_time;



	if (t_run < 1)
	{
		p_nav->auto_path.pos_pid.w.fpDes = StartQ + W * t_run;
		p_nav->auto_path.basic_velt.fpW = W;
	}
	else if (t_run < 1 + 0.1f)
	{
		p_nav->auto_path.pos_pid.w.fpDes = StartQ + DELTA_Q;
		p_nav->auto_path.basic_velt.fpW = 0;
	}
	else
	{
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
		p_nav->auto_path.run_time = 0;
		flag_record = 1;

		if (fabs(fpSumPosQ - angle * 10) <= 35)
		{

			if (region3_state == 2)//当完成了定位重新刷新后
			{ 
				if(nav.nav_state==NAV_AREA_3)
				{state3++;}
				else if(nav.nav_state==NAV_AREA_3_SINGLE)
        {state3_SINGLE++;}
				
				//nav.nav_state=NAV_OFF;
			}
		}
	}
}



void Nav_PID_Adjust(void)
{
//   if (fabs(nav.auto_path.pos_pid.x.fpFB - point_end.x) >= 5)//4
//   {
//	    nav.auto_path.pos_pid.x.fpKp = 8.f;
//        nav.auto_path.pos_pid.x.fpKi = 0.f;
//        nav.auto_path.pos_pid.x.fpKd = 0.4f;
//        nav.auto_path.pos_pid.x.fpUMax = 2000.f;
//        nav.auto_path.pos_pid.x.fpUpMax = 1000.f;
//        nav.auto_path.pos_pid.x.fpUdMax = 500.f;
//        nav.auto_path.pos_pid.x.fpSumEMax = 0.f;
//        nav.auto_path.pos_pid.x.fpEMax = 500.f;
//        nav.auto_path.pos_pid.x.fpEMin = 1.5f;
//   }
//   else if (fabs(nav.auto_path.pos_pid.x.fpFB - point_end.x) >= 3)
//   {


//		    nav.auto_path.pos_pid.x.fpKp = 8.f;
//        nav.auto_path.pos_pid.x.fpKi = 0.0004f;
//        nav.auto_path.pos_pid.x.fpKd = 0.4f;
//        nav.auto_path.pos_pid.x.fpUMax = 2000.f;
//        nav.auto_path.pos_pid.x.fpUpMax = 1000.f;
//        nav.auto_path.pos_pid.x.fpUdMax = 500.f;
//        nav.auto_path.pos_pid.x.fpSumEMax = 2000.f;
//        nav.auto_path.pos_pid.x.fpEMax = 500.f;
//        nav.auto_path.pos_pid.x.fpEMin = 0.1f;
//   }

//   else  
//   {
//		 

//		 
//		 		nav.auto_path.pos_pid.x.fpKp = 0.f;
//        nav.auto_path.pos_pid.x.fpKi = 0.f;
//        nav.auto_path.pos_pid.x.fpKd = 0.f;
//        nav.auto_path.pos_pid.x.fpUMax = 0.f;
//        nav.auto_path.pos_pid.x.fpUpMax = 0.f;
//        nav.auto_path.pos_pid.x.fpUdMax = 0.f;
//        nav.auto_path.pos_pid.x.fpSumEMax = 0.f;
//        nav.auto_path.pos_pid.x.fpEMax = 0.f;
//        nav.auto_path.pos_pid.x.fpEMin = 0.1f;
//   }




//   if (fabs(nav.auto_path.pos_pid.y.fpFB - point_end.y) >= 5)//4
//   {
//	      nav.auto_path.pos_pid.y.fpKp = 8.f;
//        nav.auto_path.pos_pid.y.fpKi = 0.f;
//        nav.auto_path.pos_pid.y.fpKd = 0.4f;
//        nav.auto_path.pos_pid.y.fpUMax = 2000.f;
//        nav.auto_path.pos_pid.y.fpUpMax = 1000.f;
//        nav.auto_path.pos_pid.y.fpUdMax = 500.f;
//        nav.auto_path.pos_pid.y.fpSumEMax = 0.f;
//        nav.auto_path.pos_pid.y.fpEMax = 500.f;
//        nav.auto_path.pos_pid.y.fpEMin = 1.5f;
//   }

//    else  if (fabs(nav.auto_path.pos_pid.y.fpFB - point_end.y) >= 3)//4
//   {

//		    nav.auto_path.pos_pid.y.fpKp = 8.f;
//        nav.auto_path.pos_pid.y.fpKi = 0.0004f;
//        nav.auto_path.pos_pid.y.fpKd = 0.4f;
//        nav.auto_path.pos_pid.y.fpUMax = 2000.f;
//        nav.auto_path.pos_pid.y.fpUpMax = 1000.f;
//        nav.auto_path.pos_pid.y.fpUdMax = 500.f;
//        nav.auto_path.pos_pid.y.fpSumEMax = 0.f;
//        nav.auto_path.pos_pid.y.fpEMax = 500.f;
//        nav.auto_path.pos_pid.y.fpEMin = 1.5f;
//   }
//   else
//   {
//		 
//		 		nav.auto_path.pos_pid.y.fpKp = 0.f;
//        nav.auto_path.pos_pid.y.fpKi = 0.f;
//        nav.auto_path.pos_pid.y.fpKd = 0.f;
//        nav.auto_path.pos_pid.y.fpUMax = 0.f;
//        nav.auto_path.pos_pid.y.fpUpMax = 0.f;
//        nav.auto_path.pos_pid.y.fpUdMax = 0.f;
//        nav.auto_path.pos_pid.y.fpSumEMax = 0.f;
//        nav.auto_path.pos_pid.y.fpEMax = 0.f;
//        nav.auto_path.pos_pid.y.fpEMin = 0.1f;
//   }








float err_x = fabs(nav.auto_path.pos_pid.x.fpFB - nav.auto_path.pos_pid.x.fpDes);
    float err_y = fabs(nav.auto_path.pos_pid.y.fpFB -nav.auto_path.pos_pid.y.fpDes);
		float err_w = fabs(nav.auto_path.pos_pid.w.fpFB -nav.auto_path.pos_pid.w.fpDes);

    // ========== X轴：死区 3.0，末端小积分 ==========
    float Kp_x, Ki_x, Kd_x;
    float Umax_x = 2500.0f;
    float UpMax_x = 1500.0f;
    float UdMax_x = 800.0f;
    float SumEMax_x = 0.0f;
    float EMax_x = 500.0f;
    float EMin_x = 3.0f;                // ? X轴死区：误差<3时完全关闭

    if (err_x >= 8.0f) {                // 大误差：高刚性
        Kp_x = 9.0f;
        Kd_x = 2.5f;
        Ki_x = 0.0f;
        SumEMax_x = 0.0f;
    } else if (err_x > 5.0f) {          // 中段：线性过渡
        float r = (err_x - 5.0f) / 3.0f; // 5 → 8
        Kp_x = 4.0f + r * 5.0f;         // 4.0 → 9.0
        Kd_x = 1.4f + r * 1.1f;         // 1.4 → 2.5
        Ki_x = 0.0f;
        SumEMax_x = 0.0f;
    } else if (err_x > EMin_x) {        // 末端小积分 (3~5)
        Kp_x = 3.0f;
        Kd_x = 0.8f;
        Ki_x = 0.002f;                  // 极小的积分
        SumEMax_x = 300.0f;             // 积分限幅
        Umax_x = 1000.0f;               // 末端出力仍足够
        UpMax_x = 1000.0f;
        UdMax_x = 500.0f;
    } else {                            // 死区内：完全关闭
        Kp_x = 0.0f; Ki_x = 0.0f; Kd_x = 0.0f;
        Umax_x = 0.0f; UpMax_x = 0.0f; UdMax_x = 0.0f;
    }

    nav.auto_path.pos_pid.x.fpKp     = Kp_x;
    nav.auto_path.pos_pid.x.fpKi     = Ki_x;
    nav.auto_path.pos_pid.x.fpKd     = Kd_x;
    nav.auto_path.pos_pid.x.fpUMax   = Umax_x;
    nav.auto_path.pos_pid.x.fpUpMax  = UpMax_x;
    nav.auto_path.pos_pid.x.fpUdMax  = UdMax_x;
    nav.auto_path.pos_pid.x.fpSumEMax = SumEMax_x;
    nav.auto_path.pos_pid.x.fpEMax   = EMax_x;
    nav.auto_path.pos_pid.x.fpEMin   = EMin_x;

    // ========== Y轴：死区 3.0，末端小积分 ==========
    float Kp_y, Ki_y, Kd_y;
    float Umax_y = 2500.0f;
    float UpMax_y = 1500.0f;
    float UdMax_y = 800.0f;
    float SumEMax_y = 0.0f;
    float EMax_y = 500.0f;
    float EMin_y = 3.0f;                // ? Y轴死区：误差<3时完全关闭

    if (err_y >= 8.0f) {
        Kp_y = 9.0f;
        Kd_y = 2.5f;
        Ki_y = 0.0f;
        SumEMax_y = 0.0f;
    } else if (err_y > 5.0f) {
        float r = (err_y - 5.0f) / 3.0f;
        Kp_y = 4.0f + r * 5.0f;
        Kd_y = 1.4f + r * 1.1f;
        Ki_y = 0.0f;
        SumEMax_y = 0.0f;
    } else if (err_y > EMin_y) {
        Kp_y = 3.0f;
        Kd_y = 0.8f;
        Ki_y = 0.002f;
        SumEMax_y = 300.0f;
        Umax_y = 1000.0f;
        UpMax_y = 1000.0f;
        UdMax_y = 500.0f;
    } else {
        Kp_y = 0.0f; Ki_y = 0.0f; Kd_y = 0.0f;
        Umax_y = 0.0f; UpMax_y = 0.0f; UdMax_y = 0.0f;
    }

    nav.auto_path.pos_pid.y.fpKp     = Kp_y;
    nav.auto_path.pos_pid.y.fpKi     = Ki_y;
    nav.auto_path.pos_pid.y.fpKd     = Kd_y;
    nav.auto_path.pos_pid.y.fpUMax   = Umax_y;
    nav.auto_path.pos_pid.y.fpUpMax  = UpMax_y;
    nav.auto_path.pos_pid.y.fpUdMax  = UdMax_y;
    nav.auto_path.pos_pid.y.fpSumEMax = SumEMax_y;
    nav.auto_path.pos_pid.y.fpEMax   = EMax_y;
    nav.auto_path.pos_pid.y.fpEMin   = EMin_y;

    // ========== 角度轴W：死区 0.2°，纯比例+微分 ==========
    float Kp_w, Ki_w, Kd_w;
    float Umax_w = 90.0f;
    float UpMax_w = 90.0f;
    float UdMax_w = 50.0f;
    float SumEMax_w = 0.0f;
    float EMax_w = 90.0f;
    float EMin_w = 0.1f;                 // ? 角度死区：0.2°

    if (err_w >= 1.5f) {
        Kp_w = 8.0f;
        Kd_w = 4.0f;
        Ki_w = 0.0f;
        Umax_w = 90.0f;
        UpMax_w = 90.0f;
        UdMax_w = 50.0f;
    } else if (err_w > 0.2f) {
        float r = (err_w - 0.2f) / 1.3f;
        Kp_w = 6.0f + r * 2.0f;
        Kd_w = 1.8f + r * 2.2f;
        Ki_w = 0.0f;
        Umax_w = 50.0f + r * 40.0f;
        UpMax_w = Umax_w;
        UdMax_w = 25.0f + r * 25.0f;
    } else if (err_w > EMin_w) {        // 接近死区 (0.2~1.0)
        Kp_w = 6.f;
        Kd_w = 1.2f;
        Ki_w = 0.0f;                    // 角度不积分
        Umax_w = 35.0f;
        UpMax_w = 35.0f;
        UdMax_w = 18.0f;
    } else {
        Kp_w = 0.0f; Ki_w = 0.0f; Kd_w = 0.0f;
        Umax_w = 0.0f; UpMax_w = 0.0f; UdMax_w = 0.0f;
    }

    nav.auto_path.pos_pid.w.fpKp     = Kp_w;
    nav.auto_path.pos_pid.w.fpKi     = Ki_w;
    nav.auto_path.pos_pid.w.fpKd     = Kd_w;
    nav.auto_path.pos_pid.w.fpUMax   = Umax_w;
    nav.auto_path.pos_pid.w.fpUpMax  = UpMax_w;
    nav.auto_path.pos_pid.w.fpUdMax  = UdMax_w;
    nav.auto_path.pos_pid.w.fpSumEMax = SumEMax_w;
    nav.auto_path.pos_pid.w.fpEMax   = EMax_w;
    nav.auto_path.pos_pid.w.fpEMin   = EMin_w;

//    if (fabs(nav.auto_path.pos_pid.w.fpFB - point_end.q) >= 3)
//    {
//        nav.auto_path.pos_pid.w.fpKp = 8.f;
//        nav.auto_path.pos_pid.w.fpKi = 0.f;
//        nav.auto_path.pos_pid.w.fpKd = 2.f;
//        nav.auto_path.pos_pid.w.fpUMax = 90.f;
//        nav.auto_path.pos_pid.w.fpUpMax = 90.f;
//        nav.auto_path.pos_pid.w.fpUdMax = 40.f;
//        nav.auto_path.pos_pid.w.fpSumEMax = 0.f;
//        nav.auto_path.pos_pid.w.fpEMax = 90.f;
//        nav.auto_path.pos_pid.w.fpEMin = 0.4f;
//    }
//		
//	else  if (fabs(nav.auto_path.pos_pid.w.fpFB - point_end.q) >= 0.4)
//    {  

//        nav.auto_path.pos_pid.w.fpKp = 6.f;
//        nav.auto_path.pos_pid.w.fpKi = 0.f;
//        nav.auto_path.pos_pid.w.fpKd = 0.f;
//        nav.auto_path.pos_pid.w.fpUMax = 50.f;
//        nav.auto_path.pos_pid.w.fpUpMax = 50.f;
//        nav.auto_path.pos_pid.w.fpUdMax = 20.f;
//        nav.auto_path.pos_pid.w.fpSumEMax = 0.f;
//        nav.auto_path.pos_pid.w.fpEMax = 90.f;
//        nav.auto_path.pos_pid.w.fpEMin = 0.f;
//    }
//    else
//    {
//        nav.auto_path.pos_pid.w.fpKp = 0.f;
//        nav.auto_path.pos_pid.w.fpKi = 0.f;
//        nav.auto_path.pos_pid.w.fpKd = 0.f;
//        nav.auto_path.pos_pid.w.fpUMax = 0.f;
//        nav.auto_path.pos_pid.w.fpUpMax = 10.f;
//        nav.auto_path.pos_pid.w.fpUdMax = 0.f;
//        nav.auto_path.pos_pid.w.fpSumEMax = 0.f;
//        nav.auto_path.pos_pid.w.fpEMax = 0.f;
//        nav.auto_path.pos_pid.w.fpEMin = 0.f;
//    }



}

