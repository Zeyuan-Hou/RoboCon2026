
#include "Crane_3508_Ctrl.h"


void Crane_3508_Ctrl(void){

//进行对起重机3508电机的控制，Crane1向上是角度增，Crane2向上是角度减，所以两个目标角度要相反	
		switch(Crane_State)
		{
			case Crane_Init://初始位置就设定在最上方好了
				
				Input_3508=0;
				break;
			
			
			
			case Crane_Up://up
				
				Input_3508=-100;//-10
       break;
			case Crane_Down://down
		  Input_3508=-2628;//2370

				break;
		
			default:
				break;
		}
		
		M3508_AngleSmoothTransition(Crane_Time);
		Crane_3508_Calc();
		
		if(Crane_State!=Crane_Init){
			
		CAN_Sendcurrent(&hcan1,0x200,Crane_3508_1.motor_current ,Crane_3508_2.motor_current,0,0);
		system_monitor.can_send_cnt_3508++;
			
		}
		
  }




void Crane_3508_Calc(void)//在这个函数里产生平滑的角度目标值并进行两个3508双环pid的计算，获取要发送的电流大小
	{

		if(posi_1>=0)//最大限位
		{posi_1= 0;}
		else if(posi_1<=-2628)//最小限位
		{posi_1= -2628;}	
		
		if(posi_2>=2628)//最大限位
		{posi_2= 2628;}
		else if(posi_2<=0)//最小限位
		{posi_2= 0;}	

		
		
		
		
  Crane_3508_TD_1.m_aim=posi_1;
	CalTD(&Crane_3508_TD_1);
		
	Crane_3508_TD_2.m_aim=posi_2;
	CalTD(&Crane_3508_TD_2);	
		
	Crane_3508_1.motor_pid.outer.fpDes=Crane_3508_TD_1.m_x1;
	Crane_3508_1.motor_pid.inner.fpDes=Crane_3508_1.motor_pid.outer.fpU;
	Crane_3508_1.motor_pid.outer.fpFB=Crane_3508_1.angle;
	Crane_3508_1.motor_pid.inner.fpFB=Crane_3508_1.anglev;
		
	Crane_3508_2.motor_pid.outer.fpDes=Crane_3508_TD_2.m_x1;
	Crane_3508_2.motor_pid.inner.fpDes=Crane_3508_2.motor_pid.outer.fpU;
	Crane_3508_2.motor_pid.outer.fpFB=Crane_3508_2.angle;
	Crane_3508_2.motor_pid.inner.fpFB=Crane_3508_2.anglev;	
		
	// PID 计算
	PID_Calc_New(&Crane_3508_1.motor_pid.outer);
	PID_Calc_New(&Crane_3508_1.motor_pid.inner);

	PID_Calc_New(&Crane_3508_2.motor_pid.outer);
	PID_Calc_New(&Crane_3508_2.motor_pid.inner);	
	

  //获取要发送的电流
	Crane_3508_1.motor_current = Crane_3508_1.motor_pid.inner.fpU;
	Crane_3508_2.motor_current = Crane_3508_2.motor_pid.inner.fpU;
		
		
		
		
		





	}


uint16_t x1_1,x1_2;
uint16_t x2_1,x2_2;
uint16_t fit_t_1,fit_t_2;
uint16_t go_1,go_2;
	
	
	
	
	
float y1_1,y1_2;
float	y2_1,y2_2;
float target_1,target_2;
float pretarget_1,pretarget_2;
float	posi_1,posi_2;
	

void M3508_AngleSmoothTransition(uint16_t time)//input3508的符号和Crane1 3508相同，向上是正，向下是负
{
		if(pretarget_1!=Input_3508)//检测目标值有没有改变，如果改变了，更新初始点和目标点坐标
		{
			  x1_1=0;
			  x2_1=time*1000;//t单位为ms，time单位为s要乘1000
			  y1_1=Crane_3508_1.angle ;
			  y2_1=Input_3508 ;
			  go_1=1;
		}
		if(go_1)//go=1过渡中，go=0过渡完成
		{
		  posi_1=getCubicCurveY_1( x1_1, y1_1, x2_1, y2_1);//曲线拟合的posi（-p0为调机械臂时水平时刻电机反馈角度）
		}
		else
		{
			posi_1=y2_1;
		}
	
		pretarget_1=Input_3508 ;//读取当前target最为上一次的pertarget


		
		
	






		
		
		
		if(pretarget_2!= -Input_3508)//检测目标值有没有改变，如果改变了，更新初始点和目标点坐标
		{
			  x1_2=0;
			  x2_2=time*1000;//t单位为ms，time单位为s要乘1000
			  y1_2=Crane_3508_1.angle ;
			  y2_2=-Input_3508 ;
			  go_2=1;
		}
		if(go_2)//go=1过渡中，go=0过渡完成
		{
		  posi_2=getCubicCurveY_2( x1_2, y1_2, x2_2, y2_2);//曲线拟合的posi（-p0为调机械臂时水平时刻电机反馈角度）
		}
		else
		{
			posi_2=y2_2;
		}
	
		pretarget_2=-Input_3508 ;//读取当前target最为上一次的pertarget
}























































//这段代码实现了一个基于三次贝塞尔曲线原理的缓动函数，用于在两个点之间创建平滑过渡效果。
//该函数接受四个参数：x1、y1表示起始点坐标，x2、y2表示目标点坐标，t表示当前时间，返回值表示在贝塞尔曲线上的y坐标。
//函数首先计算t在[0,1]之间的归一化值，然后使用三次贝塞尔曲线的公式计算出对应的曲线值。最后，根据当前时间t与目标时间x2的关系，返回相应的y坐标值，实现平滑过渡效果。
float getCubicCurveY_1(float x1, float y1, float x2, float y2) {
	  fit_t_1++;
    float t = (fit_t_1- x1) / (x2 - x1);  // t在[0,1]之间
    
	  float curve = 1.17f * t * t * t + -3.33f* t * t+ 3.156f * t;//一个x从零增到一时y从零增到一的曲线，将这个曲线按比例映射到当前的起始点和目标点
    if (fit_t_1<=x2)
		{
       return y1 + (y2 - y1) * curve;
		}
		else if(fit_t_1>x2)//大于期望的响应时间后返回期望目标值
		{
			go_1=0;
			fit_t_1=0;
			return y2;
		}
	else
		{
			return y2;
		}
}



float getCubicCurveY_2(float x1, float y1, float x2, float y2) {
	  fit_t_2++;
    float t = (fit_t_2- x1) / (x2 - x1);  // t在[0,1]之间
    
	  float curve = 1.17f * t * t * t + -3.33f* t * t+ 3.156f * t;//一个x从零增到一时y从零增到一的曲线，将这个曲线按比例映射到当前的起始点和目标点
    if (fit_t_2<=x2)
		{
       return y1 + (y2 - y1) * curve;
		}
		else if(fit_t_2>x2)//大于期望的响应时间后返回期望目标值
		{
			go_2=0;
			fit_t_2=0;
			return y2;
		}
	else
		{
			return y2;
		}
}
