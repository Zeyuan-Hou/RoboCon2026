#include "path_algorithm.h"
/**********************************************************************************************************************************************************
版权声明：HITCRT(哈工大竞技机器人协会)
文件名：path_algorithm.c
最近修改日期：2025.2.14
版本：1.1
----------------------------------------------------------------------------------------------------------------------------------------------------------
模块描述：
函数列表：


			基本完善好了
----------------------------------------------------------------------------------------------------------------------------------------------------------
修订记录：
	 作者        	时间            版本     	说明
	 XSH       2025.2.14        	1.1      在路径组合的同时加入了自旋组合，同时具有路径校准的AUTO_PATH模式
**********************************************************************************************************************************************************/

//普通路径的依赖变量
u8 flag_record = 0;
float StartX, StartY, StartQ, DELTA_X, DELTA_Y, DELTA_Q, t_run,t_rotation, Ts = 0.001;
fp32 t1, t2, t3, t4, t5, t6, t7, t8;

u8 flag_rotation=0;

u8 P_Num,W_Num;//默认的初始值就是0
/*****************************************************************************
说明：
	flag_record：初始化autopath打点
	flag_permutation_path：在navigation()里开启选择路径
	flag_rotation：决定permutation是否进行自旋，防止之前的自旋接口不清零而出错
*****************************************************************************/
void SET_NAV_PATH_PERMUTATION(u8 number) //唯一接口，别的方式开启路径可能会由于run_time、rotation_time、P_Num、W_Num和flag_rotation不清零而出错
{
	nav.auto_path.number_permutation = number;  
	nav.nav_state = NAV_PERMUTATION_PATH;        
	nav.auto_path.run_time = 0;    
	nav.auto_path.rotation_time = 0; 
	nav.auto_path.run_Sumtime = 0;   
	path_state = PATH_ONGOING;               
	flag_permutation_path = 1;  
	flag_rotation = 0;
	P_Num=0;
	W_Num=0;
	
	nav.auto_path.pos_pid.x.fpSumE = 0;
	nav.auto_path.pos_pid.y.fpSumE = 0;
	nav.auto_path.pos_pid.w.fpSumE = 0;
}
void SET_NAV_PATH_AUTO(u8 number)  //唯一接口，别的方式开启路径可能会由于run_time不清零而出错
{
	nav.auto_path.number = number;   
	nav.nav_state = NAV_AUTO_PATH;     
	nav.auto_path.run_time = 0;    
	path_state = PATH_ONGOING;     
	flag_record = 1;               
}
/*************************************************************************************************************************************************
	路径排列的弊端：
	1.由于该方法的原理是根据几个给定数据来得出该条直/曲线的整体数据，其中的T是由函数自己算出来的，因此对于一段未知路径，直接套用该函数的话是无法适应于所有情况的
		比如：对于一段长度任意的直线，由于在匀加匀减过程中V和A是要自己定的，因此无论多长的路径，匀加匀减过程经过的路程是恒定的（即V^2/2A），无法做到很短的路径
					正确做法是at^2=L来求，T计算的公式不再是V/A。
	2.只有走路径，姿态角的旋转需要自己再写一个NavRotation()函数，路径和姿态角相互独立。
*************************************************************************************************************************************************/
void Path_Permutation(ST_VECTOR* Point_Start, ST_VECTOR* Point_Inc, ST_VECTOR* V_Start, ST_VECTOR* V_End,
		                        PATH_TYPE* Path_Type, fp32* A, fp32* R, fp32* T, u8 num_module)
{
		int i;
		for(i=0;i<num_module;i++)
	{
			if(i == 0)
			{ //指定启动的速度和方向
				Path_Permuta.V_Start[0].fpLength = V_Start->fpLength;
				Path_Permuta.V_Start[0].fpThetha = V_Start->fpThetha;
				Path_Permuta.V_Start[0].type = V_Start->type;
				Covert_coordinate(Path_Permuta.V_Start);
				//指定启动位置坐标
				Path_Permuta.Point_Start[0].fpX = Point_Start->fpX;
				Path_Permuta.Point_Start[0].fpY = Point_Start->fpY;
				Path_Permuta.Point_Start[0].type = Point_Start->type;
				Covert_coordinate(Path_Permuta.Point_Start);
			
				Path_Permuta.num_module = num_module;
			}
			if(i < num_module)
			{
				Path_Permuta.Path_Type[i] = Path_Type[i];
				Covert_coordinate(Path_Permuta.Point_Start + i);
				Covert_coordinate(Path_Permuta.V_Start + i);
				
				if(Path_Type[i] == LINE)
				{
					if( fabs(A[i]) <= 1e-5) //匀速直线，只拿路径坐标增量计算即可
					{//!!!!!!!!!!这里是通过加速度来判断匀速和加减速的，因此在下面走匀速路线的时候不能只给Point_Inc，同时也得给A[i]置0
						Path_Permuta.Point_Inc[i].fpLength = Point_Inc[i].fpLength;//给第i段坐标增量赋值
						Path_Permuta.Point_Inc[i].fpThetha = Point_Inc[i].fpThetha;
						Path_Permuta.Point_Inc[i].type = Point_Inc[i].type;
						Covert_coordinate(Path_Permuta.Point_Inc + i);//求出位移增量的fpX和fpY，方便打点
						Path_Permuta.V_End[i].fpX = Path_Permuta.V_Start[i].fpX;//匀速下终止速度和起始速度一样
						Path_Permuta.V_End[i].fpY = Path_Permuta.V_Start[i].fpY;
						Path_Permuta.V_End[i].type = CARTESIAN;
						Covert_coordinate(Path_Permuta.V_End + i);
						Path_Permuta.T[i] = Path_Permuta.Point_Inc[i].fpLength / Path_Permuta.V_Start[i].fpLength;//时间等于路程除以速度，用来计算时间
					}
					else //加速或减速直线，主要需要路径终止速度和加速度，可以依次解算出路程和时间
					{ 
						Path_Permuta.V_End[i].fpLength = (V_End+i)->fpLength;//给第i段路径终止速度赋值
						Path_Permuta.V_End[i].fpThetha = (V_End+i)->fpThetha;
						Path_Permuta.V_End[i].type = (V_End+i)->type;
						Covert_coordinate(Path_Permuta.V_End + i);
						Path_Permuta.A[i] = A[i];//给第i段路径加速度赋值
						Path_Permuta.T[i] =  Fabs((Path_Permuta.V_End[i].fpLength - Path_Permuta.V_Start[i].fpLength) / Path_Permuta.A[i]); //加速时间
						Path_Permuta.Point_Inc[i].fpLength = (Path_Permuta.V_End[i].fpLength * Path_Permuta.V_End[i].fpLength - Path_Permuta.V_Start[i].fpLength * Path_Permuta.V_Start[i].fpLength) / 
																			(2.f * Path_Permuta.A[i]); //计算路程
						if(Path_Permuta.Point_Inc[i].fpLength >= 0)//路径坐标增加时，方向和初始速度一致
						{
							Path_Permuta.Point_Inc[i].fpThetha = Path_Permuta.V_Start[i].fpThetha;
						}
						else if(Path_Permuta.Point_Inc[i].fpLength < 0)//路径坐标减小时，方向和初始速度相反
						{
							Path_Permuta.Point_Inc[i].fpThetha = Path_Permuta.V_Start[i].fpThetha + 180.f;
						}	
						Path_Permuta.Point_Inc[i].type = POLAR;
						Covert_coordinate(Path_Permuta.Point_Inc+i);//求出位移增量的fpX和fpY，方便打点
					}
				}
		
				else if(Path_Type[i] == CIRCLE) //匀速圆弧，只需要终止速度（方向）和半径即可，可计算出时间和位移增量
				{//************！！！！！！！这里有可能会转反，以后需要注意！！！！！**************
					Path_Permuta.V_End[i].fpLength = (V_End+i)->fpLength;//先给终止速度赋值
					Path_Permuta.V_End[i].fpThetha = (V_End+i)->fpThetha;
					Path_Permuta.V_End[i].type = (V_End+i)->type;
					Covert_coordinate(Path_Permuta.V_End + i);	 //最终速度转换为笛卡尔坐标系，得到fpX和fpY
					if(Path_Permuta.V_End[i].fpThetha - Path_Permuta.V_Start[i].fpThetha > 0 && Path_Permuta.V_End[i].fpThetha - Path_Permuta.V_Start[i].fpThetha <= 180)//这里用来判断最后是顺时针旋转还是逆时针旋转省事
					{
						Path_Permuta.R[i] = R[i];
					}
					else if(Path_Permuta.V_End[i].fpThetha - Path_Permuta.V_Start[i].fpThetha > 180)
					{
						Path_Permuta.R[i] = -R[i];
					}
					else if(Path_Permuta.V_End[i].fpThetha - Path_Permuta.V_Start[i].fpThetha < 0 && Path_Permuta.V_End[i].fpThetha - Path_Permuta.V_Start[i].fpThetha >= -180)
					{
						Path_Permuta.R[i] = -R[i];
					}
					else if(Path_Permuta.V_End[i].fpThetha - Path_Permuta.V_Start[i].fpThetha < -180)
					{
						Path_Permuta.R[i] = R[i];
					}
					Path_Permuta.T[i] = (Path_Permuta.V_End[i].fpThetha - Path_Permuta.V_Start[i].fpThetha)* RADIAN * Path_Permuta.R[i] / Path_Permuta.V_Start[i].fpLength;//弧长除以速度即为时间
					Path_Permuta.Point_Inc[i].fpX = Path_Permuta.R[i] * (sinf(Path_Permuta.V_End[i].fpThetha*RADIAN) - sinf(Path_Permuta.V_Start[i].fpThetha*RADIAN)); //计算x，y方向的位移
					Path_Permuta.Point_Inc[i].fpY = Path_Permuta.R[i] * (cosf(Path_Permuta.V_Start[i].fpThetha*RADIAN) - cosf(Path_Permuta.V_End[i].fpThetha*RADIAN));
					Path_Permuta.Point_Inc[i].type = CARTESIAN;
					Covert_coordinate(Path_Permuta.Point_Inc+i);
				}
				
				
				Path_Permuta.Point_Start[i+1].fpX = Path_Permuta.Point_Start[i].fpX + Path_Permuta.Point_Inc[i].fpX;//每段路径过后都会更新下一段路径的起点
				Path_Permuta.Point_Start[i+1].fpY = Path_Permuta.Point_Start[i].fpY + Path_Permuta.Point_Inc[i].fpY;
				Path_Permuta.V_Start[i+1].fpX = Path_Permuta.V_End[i].fpX;//每段路径结束后更新下一段的初始速度
				Path_Permuta.V_Start[i+1].fpY = Path_Permuta.V_End[i].fpY;
				
			}
			nav.auto_path.run_Sumtime+=Path_Permuta.T[i]*1000;
	}
}

/*********************************************************************************************************
函数名称：void NavPosition(ST_Nav *p_nav,PATH_PERMUTATION *Path_Permuta)
函数功能：计算组合路径的速度期望，位置期望
输入:     1.p_nav        输出自动路径的实时参数--在全局坐标系的速度、机器人当前状态、跑自动路径的一些参数number等
					2.Path_Permuta 输入生成路径的基本参数--具体的规划速度加速度坐标等等
备注:生成位置和速度期望，无自转

		每一段路径都会根据Path_Permutation函数（会生成T）进行计算离散化的目标速度和目标位置
**********************************************************************************************************/ 
void NavPosition(ST_Nav *p_nav,PATH_PERMUTATION *Path_Permuta)
{
	fp32 alpha;
	t_run = Ts * p_nav->auto_path.run_time;  //t_run单位就是秒了    
	
	if(Path_Permuta->Path_Type[P_Num] == LINE)
  {
    if(fabs(Path_Permuta->A[P_Num]) <= 1e-5)//匀速直线
		{ //将路径拆分成一段一段的，之后用P_Num标志，每段结束有标志位这样来卡一下路径速度
			  p_nav->auto_path.pos_pid.x.fpDes = Path_Permuta->Point_Start[P_Num].fpX +  Path_Permuta->V_Start[P_Num].fpX * t_run;
		    p_nav->auto_path.pos_pid.y.fpDes = Path_Permuta->Point_Start[P_Num].fpY +  Path_Permuta->V_Start[P_Num].fpY * t_run;
		    p_nav->auto_path.basic_velt.fpVx = Path_Permuta->V_Start[P_Num].fpX;
 		    p_nav->auto_path.basic_velt.fpVy = Path_Permuta->V_Start[P_Num].fpY;
			
		}
    else//加速直线，每一段的打点位S0+Vt+0.5at2
		{
			p_nav->auto_path.pos_pid.x.fpDes = Path_Permuta->Point_Start[P_Num].fpX +  Path_Permuta->V_Start[P_Num].fpX * t_run + 
				                                 Path_Permuta->A[P_Num] * cosf(Path_Permuta->V_End[P_Num].fpThetha*RADIAN) * pow(t_run,2.f) / 2.f;
				
		  p_nav->auto_path.pos_pid.y.fpDes = Path_Permuta->Point_Start[P_Num].fpY +  Path_Permuta->V_Start[P_Num].fpY * t_run + 
				                                 Path_Permuta->A[P_Num] * sinf(Path_Permuta->V_End[P_Num].fpThetha*RADIAN) * pow(t_run,2.f) / 2.f;
				
		  p_nav->auto_path.basic_velt.fpVx = Path_Permuta->V_Start[P_Num].fpX + Path_Permuta->A[P_Num] * t_run * cosf(Path_Permuta->V_End[P_Num].fpThetha*RADIAN);
 		  p_nav->auto_path.basic_velt.fpVy = Path_Permuta->V_Start[P_Num].fpY + Path_Permuta->A[P_Num] * t_run * sinf(Path_Permuta->V_End[P_Num].fpThetha*RADIAN);
    }
  }
	if(Path_Permuta->Path_Type[P_Num] == CIRCLE)
	{
		alpha = Path_Permuta->V_End[P_Num].fpThetha - Path_Permuta->V_Start[P_Num].fpThetha;
			
		if(alpha > 180  )//处理顺时针还是逆时针旋转
		alpha -=  360;
		if( alpha < -180)
		alpha +=  360;
				
		p_nav->auto_path.pos_pid.x.fpDes = Path_Permuta->Point_Start[P_Num].fpX +  Path_Permuta->R[P_Num] * 
				                                    (sinf((Path_Permuta->V_Start[P_Num].fpThetha + alpha  * t_run / Path_Permuta->T[P_Num])*RADIAN) - 
				                                     sinf(Path_Permuta->V_Start[P_Num].fpThetha*RADIAN));//每两点间的坐标增加向量
		p_nav->auto_path.pos_pid.y.fpDes = Path_Permuta->Point_Start[P_Num].fpY +  Path_Permuta->R[P_Num] * 
				                                    (cosf(Path_Permuta->V_Start[P_Num].fpThetha*RADIAN)- 
				                                     cosf((Path_Permuta->V_Start[P_Num].fpThetha + alpha * t_run / Path_Permuta->T[P_Num])*RADIAN));
		p_nav->auto_path.basic_velt.fpVx  = Path_Permuta->V_Start[P_Num].fpLength * 
				                                     cosf((Path_Permuta->V_Start[P_Num].fpThetha + alpha * t_run / Path_Permuta->T[P_Num])*RADIAN);//匀速圆周下每一点的速度都是总速度的cos，sin
 		p_nav->auto_path.basic_velt.fpVy = Path_Permuta->V_Start[P_Num].fpLength * 
				                                     sinf((Path_Permuta->V_Start[P_Num].fpThetha + alpha * t_run / Path_Permuta->T[P_Num])*RADIAN);
	 }
      
}



void Rotation_Permutation(fp32* Rotation_Start,fp32* W_Start,fp32* Rotation_Inc,fp32* W_End
													,fp32* A_W,fp32* T_W,u8 num_modele_w )
{
	int i;
	for(i=0;i<num_modele_w;i++)
	{
		if(i==0)
		{
			Path_Permuta.Rotation_Start[0] = Rotation_Start[0];
			Path_Permuta.W_Start[0] = W_Start[0];
			
			Path_Permuta.num_module_w = num_modele_w;
		}
		if(i < num_modele_w)
		{
			if( fabs(A_W[i] ) <= 1e-5)
			{
				Path_Permuta.Rotation_Inc[i] = Rotation_Inc[i];
				Path_Permuta.W_End[i] = Path_Permuta.W_Start[i];
				Path_Permuta.T_W[i] = Path_Permuta.Rotation_Inc[i]/Path_Permuta.W_Start[i];
			}
			else
			{
				Path_Permuta.W_End[i] = W_End[i];
				Path_Permuta.A_W[i] = A_W[i];
				Path_Permuta.T_W[i] = (Path_Permuta.W_End[i]-Path_Permuta.W_Start[i])/Path_Permuta.A_W[i];
				Path_Permuta.Rotation_Inc[i] = (Path_Permuta.W_End[i]*Path_Permuta.W_End[i] - Path_Permuta.W_Start[i]*Path_Permuta.W_Start[i])
																				/(2.f * Path_Permuta.A_W[i]);
			}
			Path_Permuta.W_Start[i+1] = Path_Permuta.W_End[i];
			Path_Permuta.Rotation_Start[i+1] = Path_Permuta.Rotation_Start[i]+Path_Permuta.Rotation_Inc[i];
		}
	}
}
/*********************************************************************************************************
函数名称：void NavRotation(ST_Nav *p_nav,PATH_PERMUTATION *Path_Permuta)
函数功能：设置组合路径的转速，角度期望
输入:     1.p_nav        输出自动路径的实时参数
					2.Path_Permuta 输入生成路径的基本参数
备注:自转

		自旋进行打点拟合
**********************************************************************************************************/ 
void NavRotation(ST_Nav *p_nav,PATH_PERMUTATION *Path_Permuta)
{
	t_rotation = Ts * p_nav->auto_path.rotation_time;
	
	  if(t_rotation < Path_Permuta->T_W[W_Num] )
    { 
			if(fabs(Path_Permuta->A_W[W_Num])> 1e-5)
			{
			  p_nav->auto_path.pos_pid.w.fpDes =  Path_Permuta->Rotation_Start[W_Num] + Path_Permuta->W_Start[W_Num] * t_rotation + Path_Permuta->A_W[W_Num] * pow(t_rotation,2) / 2.f;
  		  p_nav->auto_path.basic_velt.fpW = Path_Permuta->W_Start[W_Num] + Path_Permuta->A_W[W_Num] * t_rotation;
				
			}
			else
			{
			  p_nav->auto_path.pos_pid.w.fpDes =  Path_Permuta->Rotation_Start[W_Num] + Path_Permuta->W_Start[W_Num] * t_rotation;
  		  p_nav->auto_path.basic_velt.fpW = Path_Permuta->W_Start[W_Num];
			}
		}
	if(W_Num == Path_Permuta->num_module_w)
	{
		  p_nav->auto_path.pos_pid.w.fpDes = Path_Permuta->Rotation_Start[W_Num-1] + Path_Permuta->Rotation_Inc[W_Num-1];//去掉这一句之后角度会莫名其妙归零，所以留着
  		p_nav->auto_path.basic_velt.fpW = 0;
	}
	
}



/*********************************************************************************************************
函数名称：void CheckPathEnd(ST_Nav *p_nav,PATH_PERMUTATION *Path_Permuta)
函数功能：判断路径状态（是否结束），给标志位赋值
输入:     1.p_nav        输出自动路径的实时参数
					2.Path_Permuta 输入生成路径的基本参数
备注:

		监测路径状态是否走完
**********************************************************************************************************/ 
void CheckPathEnd(ST_Nav *p_nav,PATH_PERMUTATION *Path_Permuta)
{
	if(P_Num < Path_Permuta->num_module && t_run >= Path_Permuta->T[P_Num])//总路径还没走完，该分路径已走完
	{
	  P_Num++;//进行下一路径
		p_nav->auto_path.run_time = 0;//下一路径初始化
		p_nav->auto_path.pos_pid.x.fpSumE = 0;//开启路径之前先清零积分项
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
	}
	if(W_Num < Path_Permuta->num_module_w && t_rotation >= Path_Permuta->T_W[W_Num])//总路径还没走完，该分路径已走完
	{
	  W_Num++;//进行下一路径
		p_nav->auto_path.rotation_time = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;  
	}
	if(P_Num == Path_Permuta->num_module)//最后一段减速路径
	{
		if(flag_rotation==0||(flag_rotation==1&&W_Num == Path_Permuta->num_module_w))
		{
			if(t_run < 0.1f)
			{
				p_nav->auto_path.pos_pid.x.fpDes = Path_Permuta->Point_Start[P_Num - 1].fpX +  Path_Permuta->Point_Inc[P_Num - 1].fpX;
				p_nav->auto_path.pos_pid.y.fpDes = Path_Permuta->Point_Start[P_Num - 1].fpY +  Path_Permuta->Point_Inc[P_Num - 1].fpY;
				p_nav->auto_path.pos_pid.w.fpDes = Path_Permuta->Rotation_Start[W_Num -1] + Path_Permuta->Rotation_Inc[W_Num -1];

				p_nav->auto_path.basic_velt.fpVx  = 0;
				p_nav->auto_path.basic_velt.fpVy = 0;
				p_nav->auto_path.basic_velt.fpW = 0;
			}
			else
			{			
				P_Num = 0;
				W_Num = 0;
				p_nav->auto_path.run_time = 0;
				p_nav->auto_path.rotation_time = 0;
				p_nav->auto_path.run_Sumtime = 0;
				path_state = PATH_END;
				nav.nav_state = NAV_LOCK;
				flag_lock = 1;//最后进入停止导航
			}
		}
	}

}

/*********************************************************************************************************
函数名称：void Path_Permutation_Set(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav)
函数功能：输入路径参数
输入:     Path_Permuta 输入生成路径的基本参数
备注:    
         ************************************************************************
         **路径参数要求: 直线匀速    参数: 位置增量(极坐标)                       **
         **             直线加减速  参数: 终止速度(极),加速度(区分正负)           **
         **             圆弧        参数: 终止速度(极),半径(为正)                **
         ************************************************************************
         
         Point_Start,V_Start只需付一次初值，如果初值为0可以默认初始化:
         

				 不同路径的参数，共八个路径Num_module，第八个路径为最后减速
					Point_Start,Point_Inc,V_Start,V_End,Path_Type,A,R,T,num_module,P_Num
					所有路径均需Point_Start、V_Start和Path_Type，直线匀速要Point_Inc，直线匀加减速要V_End和A，圆弧需要V_End和R
					路径时间均由Path_Permutation函数算出。
**********************************************************************************************************/ 
void Path_Permutation_Set_1(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav) //23年路径
{
  ST_VECTOR Point_Start[10]; //路程起始坐标
	ST_VECTOR Point_Inc[10]; //路程变化量
	ST_VECTOR V_Start[10];   //速度起始点
	ST_VECTOR V_End[10];     //速度终止点
	PATH_TYPE Path_Type[10];   //10段路径各自的类型：直线/圆弧/（贝赛尔）（没写）
	fp32 A[10];//10段各自的加速度
	fp32 R[10];//10段各自的半径（直线给0）
	fp32 T[10];//10段各自的时间
	u8 num_module = 8;
	
	if(P_Num == 0)  //刚开始跑组合路径
	{
		Point_Start[0].fpX = p_nav->auto_path.pos_pid.x.fpFB;//立刻读取反馈作为x,y起始坐标
		Point_Start[0].fpY = p_nav->auto_path.pos_pid.y.fpFB;
		Point_Start[0].type = CARTESIAN;
	}
	//每一段大路径的首尾速度均为0，路径切换时也不会发生速度突变
	
	//1.斜线匀加速
	//起始、终止速度和加速度（刚开始启动的时候比较特殊需要给一个总初值，之后会自动拿上一次中止当下一次的start）
	V_Start[0].fpLength = 0.001;
	V_Start[0].fpThetha = 15;
	V_Start[0].type = POLAR;
	
	V_End[0].fpLength = 2600;//1500;//
	V_End[0].fpThetha = 15;
	V_End[0].type = POLAR;
	Path_Type[0] = LINE;
	A[0] = 1500;//2000;
	
	//2.斜线匀速‘
	//极坐标位置增量
	Point_Inc[1].fpLength = 1600;//2600;//
  Point_Inc[1].fpThetha = 15;
	Point_Inc[1].type = POLAR;
	Path_Type[1] = LINE;
	A[1] = 0; 
	//3.圆弧
	//终止速度和半径
	V_End[2].fpLength = 2600;//1500;//
	V_End[2].fpThetha = 90;
	R[2] = 2500;
	V_End[2].type = POLAR;
	Path_Type[2] = CIRCLE;
	
	//4.直线匀速
	Point_Inc[3].fpLength = 1000;//1000;
  Point_Inc[3].fpThetha = 90;
	Point_Inc[3].type = POLAR;
	Path_Type[3] = LINE;
	A[3] = 0;
	
	//5.直线减速
	V_End[4].fpLength = 1300;
	V_End[4].fpThetha = 90;
	V_End[4].type = POLAR;
	A[4] = -2300;
	Path_Type[4] = LINE;
	
	//6.圆弧
	V_End[5].fpLength = 1300;
	V_End[5].fpThetha = 180;
	R[5] = 1200;
	V_End[5].type = POLAR;
	Path_Type[5] = CIRCLE;
	
	//7..直线匀速
	Point_Inc[6].fpLength = 1000;
  Point_Inc[6].fpThetha = 180;
	Point_Inc[6].type = POLAR;
	Path_Type[6] = LINE;
	A[6] = 0;
	
	//8.直线减速
	V_End[7].fpLength = 0.001;
	V_End[7].fpThetha = 180;
	V_End[7].type = POLAR;
	A[7] = -2000;
	Path_Type[7] = LINE;
	//赋好值以后只需要调用初始化函数
  Path_Permutation(Point_Start,Point_Inc,V_Start,V_End,Path_Type,A,R,T,num_module);
	
}
void Path_Permutation_Set_2(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav) //Y方向直线
{
  ST_VECTOR Point_Start[10]; //路程起始坐标
	ST_VECTOR Point_Inc[10]; //路程变化量
	ST_VECTOR V_Start[10];   //速度起始点
	ST_VECTOR V_End[10];     //速度终止点
	PATH_TYPE Path_Type[10];   //10段路径各自的类型：直线/圆弧/（贝赛尔）（没写）
	fp32 A[10];//10段各自的加速度
	fp32 R[10];//10段各自的半径（直线给0）
	fp32 T[10];//10段各自的时间
	u8 num_module = 3;
	
	if(P_Num == 0)  //刚开始跑组合路径
	{
		Point_Start[0].fpX = p_nav->auto_path.pos_pid.x.fpFB;//立刻读取反馈作为x,y起始坐标
		Point_Start[0].fpY = p_nav->auto_path.pos_pid.y.fpFB;
		Point_Start[0].type = CARTESIAN;
	}
	
	//1.直线匀加速
	//起始、终止速度和加速度（刚开始启动的时候比较特殊需要给一个总初值，之后会自动拿上一次中止当下一次的start）
	V_Start[0].fpLength = 0.001;
	V_Start[0].fpThetha = 90;
	V_Start[0].type = POLAR;
	
	V_End[0].fpLength = 1000;//1500;//
	V_End[0].fpThetha = 90;
	V_End[0].type = POLAR;
	Path_Type[0] = LINE;
	A[0] = 1000;//2000;
	
	//2.直线匀速‘
	//极坐标位置增量
	Point_Inc[1].fpLength = 1500;//2600;//
  Point_Inc[1].fpThetha = 90;
	Point_Inc[1].type = POLAR;
	Path_Type[1] = LINE;
	A[1] = 0;
	
	//3.直线减速
	V_End[2].fpLength = 0.001;
	V_End[2].fpThetha = 90;
	V_End[2].type = POLAR;
	A[2] = -1000;
	Path_Type[2] = LINE;
	
	//赋好值以后只需要调用初始化函数
  Path_Permutation(Point_Start,Point_Inc,V_Start,V_End,Path_Type,A,R,T,num_module);
	
}

void Path_Permutation_Set_3(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav) //X方向->圆弧->Y方向
{
  ST_VECTOR Point_Start[10]; //路程起始坐标
	ST_VECTOR Point_Inc[10]; //路程变化量
	ST_VECTOR V_Start[10];   //速度起始点
	ST_VECTOR V_End[10];     //速度终止点
	PATH_TYPE Path_Type[10];   //10段路径各自的类型：直线/圆弧/（贝赛尔）（没写）
	fp32 A[10];//10段各自的加速度
	fp32 R[10];//10段各自的半径（直线给0）
	fp32 T[10];//10段各自的时间
	u8 num_module = 5;
	
	if(P_Num == 0)  //刚开始跑组合路径
	{
		Point_Start[0].fpX = p_nav->auto_path.pos_pid.x.fpFB;//立刻读取反馈作为x,y起始坐标
		Point_Start[0].fpY = p_nav->auto_path.pos_pid.y.fpFB;
		Point_Start[0].type = CARTESIAN;
	}
	
	
	//1.直线匀加速
	//起始、终止速度和加速度（刚开始启动的时候比较特殊需要给一个总初值，之后会自动拿上一次中止当下一次的start）
	V_Start[0].fpLength = 0.001;
	V_Start[0].fpThetha = 0;
	V_Start[0].type = POLAR;
	
	V_End[0].fpLength = 1000;//1500;//
	V_End[0].fpThetha = 0;
	V_End[0].type = POLAR;
	Path_Type[0] = LINE;
	A[0] = 600;//2000;
	
	//2.直线匀速
	//极坐标位置增量
	Point_Inc[1].fpLength = 1000;//2600;//
  Point_Inc[1].fpThetha = 0;
	Point_Inc[1].type = POLAR;
	Path_Type[1] = LINE;
	A[1] = 0;

	//3.圆弧
	V_End[2].fpLength = 1000;
	V_End[2].fpThetha = 90;
	V_End[2].type = POLAR;
	R[2] = 750;
	Path_Type[2] = CIRCLE;
	
	//4.直线匀速
	Point_Inc[3].fpLength = 1000;//2600;//
  Point_Inc[3].fpThetha = 90;
	Point_Inc[3].type = POLAR;
	Path_Type[3] = LINE;
	A[3] = 0;
	
	//5.直线减速
	V_End[4].fpLength = 0.001;
	V_End[4].fpThetha = 90;
	V_End[4].type = POLAR;
	A[4] = -2000;
	Path_Type[4] = LINE;
	
	//赋好值以后只需要调用初始化函数
  Path_Permutation(Point_Start,Point_Inc,V_Start,V_End,Path_Type,A,R,T,num_module);
	
}
void Rotation_Permutation_Set_1(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav)//逆时针旋转90°
{
	fp32 Rotation_Start[10],W_Start[10],Rotation_Inc[10],W_End[10],A_W[10],T_W[10];
	u8 num_module_w = 3;
	
	if(W_Num == 0)
	{
		Path_Permuta->Rotation_Start[0] = p_nav->auto_path.pos_pid.w.fpFB;//立刻读取角度反馈
	}
	
	//1.匀加速旋转
	W_Start[0] =0.001;
	
	W_End[0] = 60;
	A_W[0] = 60;
	
	//2.匀速旋转
	Rotation_Inc[1] = 30;
	A_W[1] = 0;
	
	//3.匀减速旋转
	W_End[2] = 0.001;
	A_W[2] = -60;
	
	Rotation_Permutation(Rotation_Start,W_Start,Rotation_Inc,W_End,A_W,T_W,num_module_w);
}

/*********************************************************************************************************
函数名称：void path_permutation_choose(ST_Nav *p_nav)
函数功能：选择路径组合
输入:     组合路径序号：nav.auto_path.number_permutation
备注:                  
**********************************************************************************************************/ 
void path_permutation_choose(ST_Nav *p_nav)
{
  switch (p_nav->auto_path.number_permutation)
	{
	  case 0:
		// none
		 break;
		case 1:
			Path_Permutation_Set_1(&Path_Permuta,p_nav);
			Rotation_Permutation_Set_1(&Path_Permuta,p_nav);
			flag_rotation=1;
			break;
		case 2:
			Path_Permutation_Set_2(&Path_Permuta,p_nav);
			break;
		case 3:
			Path_Permutation_Set_3(&Path_Permuta,p_nav);
			break;
	  default:
		 break;
	
  }
}

static void Path_Calibration(ST_Nav *p_nav)//一段任意方向，任意大小的直线，同时改变朝向，用于执行动作前的校准
//总时间受路径长度的影响，因此如果路径长度很短，时间就会很短。假如此时转角比较大，就会转的特别快
{
  static fp32 DELTA_X, DELTA_Y;
	static fp32 Vmax,Amax,W;
	static fp32 alpha;
  if (flag_record) //初始化加速度，运动时间，位移etc
	{
		StartX = p_nav->auto_path.pos_pid.x.fpFB;
		StartY = p_nav->auto_path.pos_pid.y.fpFB;
		StartQ = p_nav->auto_path.pos_pid.w.fpFB;
		
		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
		
		DELTA_X = point_end.x - StartX;
		DELTA_Y = point_end.y - StartY;
		DELTA_Q = point_end.q - StartQ;
		while(fabs(DELTA_Q) >= 180)
		{
		  if(DELTA_Q >= 0)
			{
			  DELTA_Q -= 360;
			}
			else
			{
			  DELTA_Q += 360;
			}
		}
		
		Vmax = 1500;
		Amax = 1500;
		
		alpha = atan2f(DELTA_Y, DELTA_X);
		if(Geometric_mean(DELTA_X, DELTA_Y)-pow(Vmax,2)/Amax > 0)
		{
			t1 = Vmax / Amax;
		  t2 = (Geometric_mean(DELTA_X, DELTA_Y)-t1*t1*Amax) / Vmax;
		  t3 = t1;
		}
		else
		{
		  t1 = sqrt(Geometric_mean(DELTA_X, DELTA_Y)/ Amax);
		  t2 = 0;
		  t3 = t1;
		}
		
		W = DELTA_Q / (0.8f * (t1 + t2 + t3));
		
		flag_record = 0;
	}
	
	t_run = Ts * p_nav->auto_path.run_time;
	if (t_run < 0.8f * (t1 + t2 + t3))
	{
    p_nav->auto_path.pos_pid.w.fpDes = StartQ + W * t_run;
    p_nav->auto_path.basic_velt.fpW = W;
  } else if (t_run < t1 + t2 + t3 + 0.1f) {
    p_nav->auto_path.pos_pid.w.fpDes = StartQ + DELTA_Q;
    p_nav->auto_path.basic_velt.fpW = 0;
  } else {
    p_nav->auto_path.pos_pid.w.fpSumE = 0;
  }
	
	if (t_run < t1)
	{
	  p_nav->auto_path.pos_pid.x.fpDes = StartX + Amax*cos(alpha)*pow(t_run,2)/2.f;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + Amax*sin(alpha)*pow(t_run,2)/2.f;
		p_nav->auto_path.basic_velt.fpVx = Amax*cos(alpha)*t_run;
 		p_nav->auto_path.basic_velt.fpVy = Amax*sin(alpha)*t_run;
	}
	else if (t_run < t1 + t2)
	{
	  p_nav->auto_path.pos_pid.x.fpDes = StartX + Amax*cos(alpha)*pow(t1,2)/2 + Vmax*cos(alpha)*(t_run-t1);
		p_nav->auto_path.pos_pid.y.fpDes = StartY + Amax*sin(alpha)*pow(t1,2)/2 + Vmax*sin(alpha)*(t_run-t1);
		p_nav->auto_path.basic_velt.fpVx = Vmax*cos(alpha);
 		p_nav->auto_path.basic_velt.fpVy = Vmax*sin(alpha);
	}
	else if (t_run < t1 + t2 + t3)
	{
	  p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X - Amax*cos(alpha)*pow(t1 + t2 + t3 - t_run,2)/2.f;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y - Amax*sin(alpha)*pow(t1 + t2 + t3 - t_run,2)/2.f;
		p_nav->auto_path.basic_velt.fpVx = Amax*cos(alpha)*(t1 + t2 + t3 - t_run);
 		p_nav->auto_path.basic_velt.fpVy = Amax*sin(alpha)*(t1 + t2 + t3 - t_run);
	}
	else if (t_run < t1 + t2 + t3 + 0.1f)
	{
	  p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X ;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y;
		p_nav->auto_path.basic_velt.fpVx = 0;
 		p_nav->auto_path.basic_velt.fpVy = 0;
	}
	else
	{
		p_nav->auto_path.run_time = 0;
		
		p_nav->nav_state = NAV_LOCK;
		flag_lock  = 1;

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		
		path_state = PATH_END;       
	}
	
}

static void Path_AimEnd(ST_Nav *p_nav)//投篮赛的路径，只有路径，Yaw的转动只依靠pos_pid.w.fpDes
{
  static fp32 DELTA_X, DELTA_Y;
	static fp32 Vmax,Amax;
	static fp32 alpha;
	
	Aim_Yaw();
	nav.auto_path.pos_pid.w.fpDes = target_q;
	nav.auto_path.basic_velt.fpW = 0;//Yaw轴基础速度为0
	
  if (flag_record) //初始化加速度，运动时间，位移etc
	{
		StartX = p_nav->auto_path.pos_pid.x.fpFB;
		StartY = p_nav->auto_path.pos_pid.y.fpFB;
		
		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		
		DELTA_X = point_end.x - StartX;
		DELTA_Y = point_end.y - StartY;
		
		Vmax = 1200;
		Amax = 1200;
		
		alpha = atan2f(DELTA_Y, DELTA_X);
		if(Geometric_mean(DELTA_X, DELTA_Y)-pow(Vmax,2)/Amax > 0)
		{
			t1 = Vmax / Amax;
		  t2 = (Geometric_mean(DELTA_X, DELTA_Y)-t1*t1*Amax) / Vmax;
		  t3 = t1;
		}
		else
		{
		  t1 = sqrt(Geometric_mean(DELTA_X, DELTA_Y)*2.f/ Amax);
		  t2 = 0;
		  t3 = t1;
		}
		
		flag_record = 0;
	}
	
	t_run = Ts * p_nav->auto_path.run_time;
	
	if (t_run < t1)
	{
	  p_nav->auto_path.pos_pid.x.fpDes = StartX + Amax*cos(alpha)*pow(t_run,2)/2.f;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + Amax*sin(alpha)*pow(t_run,2)/2.f;
		p_nav->auto_path.basic_velt.fpVx = Amax*cos(alpha)*t_run;
 		p_nav->auto_path.basic_velt.fpVy = Amax*sin(alpha)*t_run;
	}
	else if (t_run < t1 + t2)
	{
	  p_nav->auto_path.pos_pid.x.fpDes = StartX + Amax*cos(alpha)*pow(t1,2)/2 + Vmax*cos(alpha)*(t_run-t1);
		p_nav->auto_path.pos_pid.y.fpDes = StartY + Amax*sin(alpha)*pow(t1,2)/2 + Vmax*sin(alpha)*(t_run-t1);
		p_nav->auto_path.basic_velt.fpVx = Vmax*cos(alpha);
 		p_nav->auto_path.basic_velt.fpVy = Vmax*sin(alpha);
	}
	else if (t_run < t1 + t2 + t3)
	{
	  p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X - Amax*cos(alpha)*pow(t1 + t2 + t3 - t_run,2)/2.f;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y - Amax*sin(alpha)*pow(t1 + t2 + t3 - t_run,2)/2.f;
		p_nav->auto_path.basic_velt.fpVx = Amax*cos(alpha)*(t1 + t2 + t3 - t_run);
 		p_nav->auto_path.basic_velt.fpVy = Amax*sin(alpha)*(t1 + t2 + t3 - t_run);
	}
	else if (t_run < t1 + t2 + t3 + 0.1f)
	{
	  p_nav->auto_path.pos_pid.x.fpDes = StartX + DELTA_X ;
		p_nav->auto_path.pos_pid.y.fpDes = StartY + DELTA_Y;
		p_nav->auto_path.basic_velt.fpVx = 0;
 		p_nav->auto_path.basic_velt.fpVy = 0;
	}
	else
	{
		p_nav->auto_path.run_time = 0;
		
		p_nav->nav_state = NAV_LOCK;
		flag_lock  = 1;

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		
		path_state = PATH_END;       
	}
	
}

//任意角度的一段自转过程
static void path_rotation(ST_Nav *p_nav)
{
	
	static fp32 t1 , t2 , t3;
	static fp32 A_W_up;
	static fp32 A_W_down;
	static fp32 Alpha,W;

	if (flag_record) 
	{
		A_W_up = 90;
		A_W_down = 90;
		W = 45;
		StartX = p_nav->auto_path.pos_pid.x.fpFB;
		StartY = p_nav->auto_path.pos_pid.y.fpFB;
		StartQ = p_nav->auto_path.pos_pid.w.fpFB;
    Alpha = point_end.q - StartQ;
		while(fabs(Alpha) >= 180)
		{
		  if(Alpha >= 0)
			{
			  Alpha -= 360;
			}
			else
			{
			  Alpha += 360;
			}
		}
		if(Alpha >= 0)
		{
		  A_W_up = 90;
		  A_W_down = 90;
		  W = 45;
		}
		else
		{
		  A_W_up = -90;
		  A_W_down = -90;
		  W = -45;
		}
		
		if(fabs(Alpha) <= fabs(pow(W,2)/(A_W_up * 2.f) + pow(W,2)/(A_W_down * 2.f)))//总角度小于匀加和匀减过程转过的角度
		{ 
			t1 = sqrt(Alpha / A_W_up);
		  t2 = 0;
		  t3 = sqrt(Alpha / A_W_down);
    }else
		{
		  t1 = W / A_W_up;
		  t2 = (Alpha - A_W_up*pow(t1,2)) / W;
		  t3 = W / A_W_down ;
		}
		
		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
		flag_record = 0;
	}

	t_run = Ts * p_nav->auto_path.run_time;
	
	if (t_run < t1 )
	{
		
		p_nav->auto_path.pos_pid.x.fpDes = StartX ;
		p_nav->auto_path.pos_pid.y.fpDes = StartY ;
		p_nav->auto_path.basic_velt.fpVx = 0;
 		p_nav->auto_path.basic_velt.fpVy = 0;
    p_nav->auto_path.pos_pid.w.fpDes = StartQ + A_W_up * pow(t_run,2) / 2.f;
		p_nav->auto_path.basic_velt.fpW  = W * t_run / t1;
	}
	else if (t_run < t1 + t2 )
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX;
		p_nav->auto_path.pos_pid.y.fpDes = StartY;
		p_nav->auto_path.basic_velt.fpVx = 0;
 		p_nav->auto_path.basic_velt.fpVy = 0 ;
		p_nav->auto_path.pos_pid.w.fpDes = StartQ + A_W_up * pow(t1,2) / 2.f + W * (t_run - t1);
		p_nav->auto_path.basic_velt.fpW  = W ;
	}
	else if (t_run < t1 + t2 + t3 )
  {
		p_nav->auto_path.pos_pid.x.fpDes = StartX;
		p_nav->auto_path.pos_pid.y.fpDes = StartY;
		p_nav->auto_path.basic_velt.fpVx = 0;
 		p_nav->auto_path.basic_velt.fpVy = 0 ;
		p_nav->auto_path.pos_pid.w.fpDes = StartQ + Alpha - A_W_up * pow(t1 + t2 + t3 - t_run,2) / 2.f ;
		p_nav->auto_path.basic_velt.fpW  = W * (t1 + t2 + t3 - t_run) / t3 ;
	}
	else if (t_run < t1 + t2 + t3 +0.1f)
	{
		p_nav->auto_path.pos_pid.x.fpDes = StartX;
		p_nav->auto_path.pos_pid.y.fpDes = StartY;
		p_nav->auto_path.basic_velt.fpVx = 0;
 		p_nav->auto_path.basic_velt.fpVy = 0;
		p_nav->auto_path.pos_pid.w.fpDes = StartQ + Alpha;
		p_nav->auto_path.basic_velt.fpW = 0 ;
	}
	else 
	{
		p_nav->auto_path.run_time = 0;
		p_nav->nav_state = NAV_LOCK;
		flag_lock  = 1;

		p_nav->auto_path.pos_pid.x.fpSumE = 0;
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
		
		path_state = PATH_END;       
	}

}

void path_choose(ST_Nav *p_nav)//通过SET_NAV_PATH_AUTO(number)来设置auto_path.number，选择使用哪个打点路径，最后用遥控器来控制Number
{
	switch (p_nav->auto_path.number)
	{
	case 0:
		break;
	case 1:
		Path_Calibration(&nav);
		break;
	case 2:
		path_rotation(&nav);
		break;
	case 3:
		Path_AimEnd(&nav);
		break;
	default:
		break;
	}
}

fp32 target_q;
	
void Aim_Yaw(void)
{
	static fp32 DELTA_X, DELTA_Y;
	static fp32 StartX,StartY;
	
	StartX = nav.auto_path.pos_pid.x.fpFB;
	StartY = nav.auto_path.pos_pid.y.fpFB;
	
	DELTA_X = 100 - StartX;
	DELTA_Y = 2800 - StartY;
	
	target_q = atan2f(DELTA_Y, DELTA_X)/RADIAN - 90.f;
	while(fabs(target_q) >= 180)
	{
		if(target_q >= 0)
		{
			target_q -= 360;
		}
		else
		{
			target_q += 360;
		}
	}
}

