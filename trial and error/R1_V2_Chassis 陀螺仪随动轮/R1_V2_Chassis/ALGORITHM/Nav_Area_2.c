#include "Nav_Area_2.h"








//初始上电位置是（0，0）


//x-25，y-22
POINT point_init[14]=
{                          {.x=2000-length_differX_1-100-length_differX_2-15,.y=1800-length_differY_2},{.x=2000-length_differX_1-100-length_differX_2-15,.y=3000-length_differY_2},{.x=2000-length_differX_1-100-length_differX_2-15,.y=4200-length_differY_2},
  {.x=2600-length_differX_2,.y=1200-length_differX_1-100-length_differY_2-30},/*删了-20*/                                                                            {.x=2600-length_differX_2,.y=4800+length_differX_1+100-length_differY_2+10},
  {.x=3800-length_differX_2,.y=1200-length_differX_1-100-length_differY_2-30},                                                                                       {.x=3800-length_differX_2,.y=4800+length_differX_1+100-length_differY_2+10},
  {.x=5000-length_differX_2,.y=1200-length_differX_1-100-length_differY_2-30},                                                                                       {.x=5000-length_differX_2,.y=4800+length_differX_1+100-length_differY_2+10},
  {.x=6200-length_differX_2,.y=1200-length_differX_1-100-length_differY_2-30},                                                                                       {.x=6200-length_differX_2,.y=4800+length_differX_1+100-length_differY_2+10},
                            {.x=6800+length_differX_1+100-length_differX_2-40+10,.y=1800-length_differY_2},{.x=6800+length_differX_1+100-length_differX_2-40+10,.y=3000-length_differY_2},{.x=6800+length_differX_1+100-length_differX_2-40+10,.y=4200-length_differY_2},
};
POINT point_turn_1to2={.x=1500-length_differX_2,.y=1200-length_differY_2,};//1区转2区的转弯点
POINT point_turn_1to3={.x=1500-length_differX_2,.y=4800-length_differY_2};//1区转3区的转弯点

POINT point_turn_2to1={.x=2000-length_differX_2,.y=700-length_differY_2};//2区转1区的转弯点
POINT point_turn_2to4={.x=6800-length_differX_2,.y=700-length_differY_2};//2区转4区的转弯点

POINT point_turn_3to1={.x=2000-length_differX_2,.y=5300-length_differY_2};//3区转1区的转弯点
POINT point_turn_3to4={.x=6800-length_differX_2,.y=5300-length_differY_2};//3区转4区的转弯点

POINT point_turn_4to2={.x=7300-length_differX_2,.y=1200-length_differY_2};//4区转2区的转弯点
POINT point_turn_4to3={.x=7300-length_differX_2,.y=4800-length_differY_2};//4区转3区的转弯点



uint16_t spot_pre=1;
uint16_t spot=1;//spot0-spot13分别对应14个点，初始位置是spot1



uint16_t Flag_record=1;//这个变量用来记录是否已经记录过spot_pre了，第一次进入函数的时候需要记录，之后就不需要再记录了，直到spot发生改变
Arc arc;//圆弧参数结构体
u8 enter=0;//这个变量用来判断是否是进入二区，如果是进入二区导航状态就先走到设定位置（不论是第几次进入）

fp32 T1, T2, T3;//三段速度规划的时间，分别对应转弯前直线，转弯，转弯后直线，单位为？
double  T_run,distance_already,alpha,w_pre,pre_d= 0.001;
//startQ在再次进入二区导航时总会被重新赋值为point_end.q，同时在走圆弧路径的时候会被赋值为当前点位的角度，作为转弯的起始角度


//二区走圆弧速度规划参数
float Vmax = 500;//  速度上限，单位mm/s
float Amax = 500;//  加速度上限，单位mm/s^2





/*********************************************************************************************************
函数名称：void Nav_Area_2(ST_Nav *p_nav)
函数功能：点位切换主逻辑，根据目标点位所在区域选择直线/圆弧路径规划
输入:     1.p_nav        导航控制结构体指针，包含路径规划、PID控制等参数
输出:     无返回值，更新导航参数或底盘电机输出
备注:     1.全局变量依赖：
          - enter：初始化标志（0-首次进入，触发初始点位设置）
          - spot_pre/spot：上一/当前目标点位索引
          - point_init[]：点位坐标数组；point_end：目标点位坐标
          - arc：圆弧路径参数结构体；luy/lux/ruy/rux等：区域边界坐标
          2.核心逻辑：
            - enter=0时初始化首个目标点位（坐标+角度）并校准路径
            - spot_pre≠spot时触发点位切换，判断点位所属区域：
              - 同区域：调用path_task走直线路径
              - 不同区域：调用arc_function走圆弧过渡路径
            - spot_pre=spot时，底盘电机输出置0（停止运动）
          3.divide函数用于判断点位所属区域（1-4区）
**********************************************************************************************************/
void Nav_Area_2_Task(ST_Nav *p_nav)
{
	if(enter==0)//enter用来判断是否是进入二区，如果是进入二区导航状态就先走到设定位置（不论是第几次进入）
	{
		point_end.x=point_init[spot].x;
		point_end.y=point_init[spot].y;
		
		switch(divide(&point_init[spot])){	


			
			case 1:
				point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0) ;
			break;
			
			case 2:
				point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
			break;
			
			case 3:
			    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
			break;
			
			case 4:
				point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
			break;
			



			default:				
			break;
		}
		

		
		
		
		
		
		NavLineMoveWithHeading(&nav);//如果是第一次进入二区，直接走直线到设定点位，之后就根据点位的变化来选择走直线还是圆弧//有到位判断，到位把enter置1
		// StartQ=point_end.q;//更新一下角度
		
		
		
		spot_pre=spot;//这样就不会进下面那个if判断了//其实加上这一句之后马上就走到下面哪个NavLineMove了
	}
	
	
	
	
	
	
	if(spot_pre!=spot)
	{   //现在要根据走的路径来具体决定point_end




		if(divide(&point_init[spot_pre])==divide(&point_init[spot]))
		{//同区域分支，直接走直线      
            
            
             point_end.x=point_init[spot].x;
		         point_end.y=point_init[spot].y;

            switch(divide(&point_init[spot])){	

			case 1:
				point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0) ;
			break;
			
			case 2:
				point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
			break;
			
			case 3:
			  point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
			break;
			
			case 4:
				point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
			break;
			default:				
			break;
		}



			NavLineMoveWithHeading(p_nav);//如果点位在同一区域，直接走直线
			// StartQ=point_end.q;
		}



		else//不同区域分支
		{

		if(divide(&point_init[spot_pre])==1)//大分支，1区转其他区
			{
				if(divide(&point_init[spot])==2)
				{   
                    switch(one_turn_state){
                    case 0:
                    one_turn_state=1;
										flag_record=1;
										break;
                    case 1: 
                    point_end.x=point_turn_1to2.x;
                    point_end.y=point_turn_1to2.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 2:
                    arc.circle_x=2000-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=1200-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=PI;//起始角度
                    arc.theta_end=1.5f*PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 3:
                    point_end.x=point_init[spot].x;
                    point_end.y=point_init[spot].y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
                    NavLineMove_VelocityControl(&nav, 500, 0);
                    //依照正常运形，此时spot_pre和spot已经相等了，进入另一个分支了
                    break;
                    default:
                    break;
                    
                    }

				}
				else if(divide(&point_init[spot])==3)
				{
					switch(one_turn_state){
                    case 0:
                    one_turn_state=1;
										flag_record=1;
										break;
                    case 1: 
                    point_end.x=point_turn_1to3.x;
                    point_end.y=point_turn_1to3.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 2:
                    arc.circle_x=2000-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=4800-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=PI;//起始角度
                    arc.theta_end=0.5f*PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 3:
                    point_end.x=point_init[spot].x;
                    point_end.y=point_init[spot].y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
                    NavLineMove_VelocityControl(&nav, 500, 0);
                    //依照正常运形，此时spot_pre和spot已经相等了，进入另一个分支了
                    break;
                    default:
                    break;
                    
                    }
				}
                else if(divide(&point_init[spot])==4)
                {way=Spot_Judge();
                 if(way==-1)//逆时针，1to2to4
                 {

                   switch(two_turn_state){
                case 0:
                    two_turn_state=1;
								    flag_record=1;
																		break;
                    case 1:
                    point_end.x=point_turn_1to2.x;
                    point_end.y=point_turn_1to2.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 2:
                    arc.circle_x=2000-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=1200-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=PI;//起始角度
                    arc.theta_end=1.5f*PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 3:                
                    point_end.x=point_turn_2to4.x;
                    point_end.y=point_turn_2to4.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 4:
                    arc.circle_x=6800-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=1200-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=1.5f*PI;//起始角度
                    arc.theta_end=2.f*PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 5:
                    point_end.x=point_init[spot].x;
                    point_end.y=point_init[spot].y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
                    NavLineMove_VelocityControl(&nav, 500, 0);
                    break;
                    default:
                    break;
                    
                    



  


                   }













                 }
                 else if(way==1)//顺时针，1to3to4
                  {
                    switch(two_turn_state){
                    case 0:
                    two_turn_state=1;
										flag_record=1;
										break;
                    case 1:
                    point_end.x=point_turn_1to3.x;
                    point_end.y=point_turn_1to3.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 2:
                    arc.circle_x=2000-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=4800-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=PI;//起始角度
                    arc.theta_end=0.5f*PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 3:                
                    point_end.x=point_turn_3to4.x;
                    point_end.y=point_turn_3to4.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 4:
                    arc.circle_x=6800-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=4800-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=0.5f*PI;//起始角度
                    arc.theta_end=0.f;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 5:
                    point_end.x=point_init[spot].x;
                    point_end.y=point_init[spot].y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
                    NavLineMove_VelocityControl(&nav, 500, 0);
                    break;
                    default:
                    break;
                }

                 }
                }
                

            
                
            
            

            }
			
			else if(divide(&point_init[spot_pre])==2)//大分支，2区转其他区			
				{
				if(divide(&point_init[spot])==1)
				{
					switch(one_turn_state){
                    case 0:
                    one_turn_state=1;
										flag_record=1;
																				break;
                    case 1: 
                    point_end.x=point_turn_2to1.x;
                    point_end.y=point_turn_2to1.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 2:
                    arc.circle_x=2000-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=1200-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=1.5f*PI;//起始角度
                    arc.theta_end=PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 3:
                    point_end.x=point_init[spot].x;
                    point_end.y=point_init[spot].y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0);
                    NavLineMove_VelocityControl(&nav, 500, 0);
                    //依照正常运形，此时spot_pre和spot已经相等了，进入另一个分支了
                    break;
                    default:
                    break;
                    
                    }
				}
				else if(divide(&point_init[spot])==4)
				{
					switch(one_turn_state){
                    case 0:
                    one_turn_state=1;
										flag_record=1;
																				break;
                    case 1: 
                    point_end.x=point_turn_2to4.x;
                    point_end.y=point_turn_2to4.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 2:
                    arc.circle_x=6800-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=1200-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=1.5f*PI;//起始角度
                    arc.theta_end=2*PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 3:
                    point_end.x=point_init[spot].x;
                    point_end.y=point_init[spot].y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
                    NavLineMove_VelocityControl(&nav, 500, 0);
                    //依照正常运形，此时spot_pre和spot已经相等了，进入另一个分支了
                    break;
                    default:
                    break;
                    
                    }
				}

                else if(divide(&point_init[spot])==3)
                {way=Spot_Judge();
                 if(way==1)//顺时针，2to1to3
                 {

                   switch(two_turn_state){
                case 0:
                    two_turn_state=1;
								    flag_record=1;
																		break;
                    case 1:
                    point_end.x=point_turn_2to1.x;
                    point_end.y=point_turn_2to1.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 2:
                    arc.circle_x=2000-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=1200-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=1.5f*PI;//起始角度
                    arc.theta_end=PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 3:                
                    point_end.x=point_turn_1to3.x;
                    point_end.y=point_turn_1to3.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 4:
                    arc.circle_x=2000-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=4800-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=PI;//起始角度
                    arc.theta_end=0.5f*PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 5:
                    point_end.x=point_init[spot].x;
                    point_end.y=point_init[spot].y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
                    NavLineMove_VelocityControl(&nav, 500, 0);
                    break;
                    default:
                    break;
                    
                    



  


                   }













                 }
                 else if(way==-1)//逆时针，2to4to3
                  {
                    switch(two_turn_state){
                    case 0:
                    two_turn_state=1;
										flag_record=1;
																				break;
                    case 1:
                    point_end.x=point_turn_2to4.x;
                    point_end.y=point_turn_2to4.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 2:
                    arc.circle_x=6800-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=1200-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=1.5f*PI;//起始角度
                    arc.theta_end=2.f*PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 3:                
                    point_end.x=point_turn_4to3.x;
                    point_end.y=point_turn_4to3.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 4:
                    arc.circle_x=6800-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=4800-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=0;//起始角度
                    arc.theta_end=0.5f*PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 5:
                    point_end.x=point_init[spot].x;
                    point_end.y=point_init[spot].y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
                    NavLineMove_VelocityControl(&nav, 500, 0);
                    break;
                    default:
                    break;
                }

                 }
                }
			}
			
			else if(divide(&point_init[spot_pre])==3)//大分支，3区转其他区
			{
				if(divide(&point_init[spot])==1)
				{
					switch(one_turn_state){
                    case 0:
                    one_turn_state=1;
										flag_record=1;
																				break;
                    case 1: 
                    point_end.x=point_turn_3to1.x;
                    point_end.y=point_turn_3to1.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 2:
                    arc.circle_x=2000-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=4800-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=0.5f*PI;//起始角度
                    arc.theta_end=PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 3:
                    point_end.x=point_init[spot].x;
                    point_end.y=point_init[spot].y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0);
                    NavLineMove_VelocityControl(&nav, 500, 0);
                    //依照正常运形，此时spot_pre和spot已经相等了，进入另一个分支了
                    break;
                    default:
                    break;
                    
                    }
				}
				else if(divide(&point_init[spot])==4)
				{
					switch(one_turn_state){
                    case 0:
                    one_turn_state=1;
										flag_record=1;
																				break;
                    case 1: 
                    point_end.x=point_turn_3to4.x;
                    point_end.y=point_turn_3to4.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 2:
                    arc.circle_x=6800-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=4800-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=0.5f*PI;//起始角度
                    arc.theta_end=0;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 3:
                    point_end.x=point_init[spot].x;
                    point_end.y=point_init[spot].y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
                    NavLineMove_VelocityControl(&nav, 500, 0);
                    //依照正常运形，此时spot_pre和spot已经相等了，进入另一个分支了
                    break;
                    default:
                    break;
                    
                    }
				}
                else if(divide(&point_init[spot])==2)
                {way=Spot_Judge();
                 if(way==-1)//逆时针，3to1to2
                 {

                   switch(two_turn_state){
                case 0:
                    two_turn_state=1;
								    flag_record=1;
																		break;
                    case 1:
                    point_end.x=point_turn_3to1.x;
                    point_end.y=point_turn_3to1.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 2:
                    arc.circle_x=2000-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=4800-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=0.5f*PI;//起始角度
                    arc.theta_end=PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 3:                
                    point_end.x=point_turn_1to2.x;
                    point_end.y=point_turn_1to2.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 4:
                    arc.circle_x=2000-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=1200-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=PI;//起始角度
                    arc.theta_end=1.5f*PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 5:
                    point_end.x=point_init[spot].x;
                    point_end.y=point_init[spot].y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
                    NavLineMove_VelocityControl(&nav, 500, 0);
                    break;
                    default:
                    break;
                    
                    



  


                   }













                 }
                 else if(way==1)//顺时针，3to4to2
                  {
                    switch(two_turn_state){
                    case 0:
                    two_turn_state=1;
										flag_record=1;
																				break;
                    case 1:
                    point_end.x=point_turn_3to4.x;
                    point_end.y=point_turn_3to4.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 2:
                    arc.circle_x=6800-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=4800-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=0.5f*PI;//起始角度
                    arc.theta_end=0.f;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 3:                
                    point_end.x=point_turn_4to2.x;
                    point_end.y=point_turn_4to2.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 4:
                    arc.circle_x=6800-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=1200-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=0;//起始角度
                    arc.theta_end=-0.5f*PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 5:
                    point_end.x=point_init[spot].x;
                    point_end.y=point_init[spot].y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
                    NavLineMove_VelocityControl(&nav, 500, 0);
                    break;
                    default:
                    break;
                }

                 }
                }
			}
			
			else if(divide(&point_init[spot_pre])==4)//大分支，4区转其他区
			{
				if(divide(&point_init[spot])==2)
				{
					switch(one_turn_state){
                    case 0:
                    one_turn_state=1;
										flag_record=1;
																				break;
                    case 1: 
                    point_end.x=point_turn_4to2.x;
                    point_end.y=point_turn_4to2.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 2:
                    arc.circle_x=6800-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=1200-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=0;//起始角度
                    arc.theta_end=-0.5*PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 3:
                    point_end.x=point_init[spot].x;
                    point_end.y=point_init[spot].y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
                    NavLineMove_VelocityControl(&nav, 500, 0);
                    //依照正常运形，此时spot_pre和spot已经相等了，进入另一个分支了
                    break;
                    default:
                    break;
                    
                    }
				}
				else if(divide(&point_init[spot])==3)
				{
					switch(one_turn_state){
                    case 0:
                    one_turn_state=1;
										flag_record=1;
																				break;
                    case 1: 
                    point_end.x=point_turn_4to3.x;
                    point_end.y=point_turn_4to3.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 2:
                    arc.circle_x=6800-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=4800-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=0.f;//起始角度
                    arc.theta_end=0.5f*PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 3:
                    point_end.x=point_init[spot].x;
                    point_end.y=point_init[spot].y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
                    NavLineMove_VelocityControl(&nav, 500, 0);
                    //依照正常运形，此时spot_pre和spot已经相等了，进入另一个分支了
                    break;
                    default:
                    break;
                    
                    }
				}
                else if(divide(&point_init[spot])==1)
                {way=Spot_Judge();
                 if(way==1)//顺时针，4to2to1
                 {

                   switch(two_turn_state){
                    case 0:
                    two_turn_state=1;
										flag_record=1;
																				break;
                    case 1:
                    point_end.x=point_turn_4to2.x;
                    point_end.y=point_turn_4to2.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 2:
                    arc.circle_x=6800-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=1200-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=0.f;//起始角度
                    arc.theta_end=-0.5f*PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 3:                
                    point_end.x=point_turn_2to1.x;
                    point_end.y=point_turn_2to1.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 4:
                    arc.circle_x=2000-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=1200-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=1.5f*PI;//起始角度
                    arc.theta_end=PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 5:
                    point_end.x=point_init[spot].x;
                    point_end.y=point_init[spot].y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0);
                    NavLineMove_VelocityControl(&nav, 500, 0);
                    break;
                    default:
                    break;
                    
                    



  


                   }













                 }
                 else if(way==-1)//逆时针，4to3to1
                  {
                    switch(two_turn_state){
                    case 0:
                    two_turn_state=1;
										flag_record=1;
																				break;
                    case 1:
                    point_end.x=point_turn_4to3.x;
                    point_end.y=point_turn_4to3.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 2:
                    arc.circle_x=6800-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=4800-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=0.f;//起始角度
                    arc.theta_end=0.5f*PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 3:                
                    point_end.x=point_turn_3to1.x;
                    point_end.y=point_turn_3to1.y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
                    NavLineMove_VelocityControl(&nav, 0, 500);
                    break;
                    case 4:
                    arc.circle_x=2000-length_differX_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.circle_y=4800-length_differY_2;//圆心坐标，后续需要根据转弯方向和起始点位计算
                    arc.theta_start=0.5f*PI;//起始角度
                    arc.theta_end=PI;//结束角度
                    arc.R=500;//半径，后续需要根据转弯方向和起始点位计算
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0);
                    NavArcMoving(&arc,&nav);//圆弧形规划角度和位置，其中平移规划用了arc的theta_start和theta_end，转角规划还是回到了startQ和point_end.q
                    break;
                    case 5:
                    point_end.x=point_init[spot].x;
                    point_end.y=point_init[spot].y;
                    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0);
                    NavLineMove_VelocityControl(&nav, 500, 0);
                    break;
                    default:
                    break;
                }//switch结束

            }//顺时针结束
                }//四区两次转弯结束
							}//四区转其他区结束
			
							
							
									
					}		//不同区分支结束
		}//同区不同点结束
	
	else if (spot_pre==spot)
	{         
		
			point_end.x=point_init[spot].x;
		  point_end.y=point_init[spot].y;

       switch(divide(&point_init[spot])){	

			case 1:
				point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)0) ;
			break;
			
			case 2:
				point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)90);
			break;
			
			case 3:
			    point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)270);
			break;
			
			case 4:
				point_end.q=nav.auto_path.pos_pid.w.fpFB+(float)GetMinTurnAngle((double)nav.auto_path.pos_pid.w.fpFB,(double)180);
			break;
			default:				
			break;
		}
      NavLineMove(p_nav);//在这里用于稳定角度
//		  StartQ=point_end.q;

	}//同spot分支结束
	
}//整体函数结束

/*********************************************************************************************************
函数名称：uint16_t divide(POINT* point)
函数功能：判断点位所属区域，返回区域编号（1-4）
输入:     1.point         点位坐标结构体指针，包含x/y坐标值
输出:     返回值：uint16_t类型，1-4分别对应不同区域
备注:     1.区域划分规则：
          - 区域1：y∈(1100,3900) 且 x∈(800,1200)
          - 区域2：y∈(0,400)     且 x∈(2000,6000)
          - 区域3：y∈(4800,5200) 且 x∈(2000,6000)
          - 区域4：y∈(1200,4000) 且 x∈(6800,7200)
          2.坐标单位：未明确标注，默认与导航系统一致（如mm）
          3.无匹配区域时无返回值（建议补充默认返回0，增强鲁棒性）
**********************************************************************************************************/
uint16_t divide(POINT* point)//这个函数用来判断一个点在哪个区域，返回值1-4分别对应四个区域，0代表不在任何一个区域内
{
	if((point->y>1100&&point->y<3900)&&(point->x>800&&point->x<1200))
	{
		return 1;
	}
	else if((point->y>0&&point->y<400)&&(point->x>1800&&point->x<6000))
	{
		return 2;
	}
	else if((point->y>4800&&point->y<5200)&&(point->x>1800&&point->x<6000))
	{
		return 3;
	}
	else if((point->y>1200&&point->y<4000)&&(point->x>6800&&point->x<7200))
	{
		return 4;
	}
	else
		return 0;//源代码这一句都没写，也就是其他情况无return值，是后来补上的
}


void NavArcMoving(Arc *p_arc, ST_Nav *p_nav)//最终的角度是point_end.q，最终的位置由R和角度决定
{
    static fp32 V_arc = 500.0f;  // 匀速圆弧速度，你自己改
    static fp32 StartQ, EndQ;

    // 路径初始化（完全和你其他函数一样）
    if (flag_record)
    {
        nav.auto_path.run_time = 0;

        StartQ = p_nav->auto_path.pos_pid.w.fpFB;
        EndQ = point_end.q;

        p_nav->auto_path.pos_pid.x.fpSumE = 0;
        p_nav->auto_path.pos_pid.y.fpSumE = 0;
        p_nav->auto_path.pos_pid.w.fpSumE = 0;

        flag_record = 0;
    }

    t_run = Ts * nav.auto_path.run_time;

    fp32 circle_x = p_arc->circle_x;
    fp32 circle_y = p_arc->circle_y;
    fp32 R = p_arc->R;
    fp32 theta_start = p_arc->theta_start;
    fp32 theta_end = p_arc->theta_end;

    fp32 delta_theta = theta_end - theta_start;//计算一下角度差
    fp32 total_arc_len = fabsf(R * delta_theta);//计算一下总路程
    fp32 total_time = total_arc_len / V_arc;//计算一下总时长

    // 圆弧进度百分比（0~1）
    fp32 percent = t_run / total_time;
    if (percent > 1.0f) percent = 1.0f;

    // 圆弧位置
    fp32 theta_now = theta_start + delta_theta * percent;
    p_nav->auto_path.pos_pid.x.fpDes = circle_x + R * cosf(theta_now);
    p_nav->auto_path.pos_pid.y.fpDes = circle_y + R * sinf(theta_now);

    // 切向速度
    fp32 dir_x = -sinf(theta_now);
    fp32 dir_y =  cosf(theta_now);

    if (delta_theta < 0)
    {
        dir_x = -dir_x;
        dir_y = -dir_y;
    }
    p_nav->auto_path.basic_velt.fpVx = V_arc * dir_x;
    p_nav->auto_path.basic_velt.fpVy = V_arc * dir_y;

    // 角度跟随圆弧百分比（完全对齐）
    p_nav->auto_path.pos_pid.w.fpDes = StartQ + (EndQ - StartQ) * percent;
    p_nav->auto_path.basic_velt.fpW = (EndQ - StartQ) / total_time;

    if (t_run>=total_time)//时间结束直接跳转到下一阶段
    {
        if(one_turn_state==2)
        {one_turn_state=3;
					flag_record=1;
					nav.auto_path.run_time=0;
				 }
        if(two_turn_state==2||two_turn_state==4)
        {two_turn_state+=1;
				flag_record=1;
				nav.auto_path.run_time=0;}  
    }
    
}

int8_t Spot_Judge(void)//获取在二区连续转两个弯时要怎么转弯，顺时针还是逆时针
{   int16_t num_pre,num,steps;
    num_pre=exchange_spot(spot_pre);
    num=exchange_spot(spot);
    steps=num-num_pre;//要正走几步
    if(steps>0)
    {
        // 处理正向移动
        if (steps>7)//顺时针走太远
        {
            return -1;//逆时针走
        }
        else
        {
            return 1;//顺时针走
        }
        
    }
    else    
    {
        // 处理反向移动
        steps=-steps;//要倒走几步
        if (steps>=7)//逆时针走太远
        {
            return 1;//顺时针走
        }
        else
        {
            return -1;//逆时针走
        }
    }
}

uint16_t exchange_spot(uint16_t spot)//这个函数用来把spot编号转换成顺时针编号，方便Spot_Judge函数计算要正走几步还是倒走几步
{
    switch(spot)
    {
        case 0:
        return 0;

        case 1:
        return 1;

        case 2:
        return 2;

        case 3:
        return 13;

        case 4:
        return 3;

        case 5:
        return 12;

        case 6:
        return 4;

        case 7:
        return 11;

        case 8:
        return 5;

        case 9:
        return 10;

        case 10:
        return 6;

        case 11:
        return 9;

        case 12:
        return 8;

        case 13:
        return 7;
        default:
        return 0;//真走到这一步会炸的
    }
} 
