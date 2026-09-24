#include "pid.h"


void CalTD(ST_TD *pStTD)
{
	float d,d0,y,a0,a=0,fhan;
	pStTD->m_x = pStTD->m_x1 - pStTD->m_aim;
	    // 计算当前位置与目标位置的误差
	d = pStTD->m_r * pStTD->m_h;// 计算调节因子 d
	d0 = pStTD->m_h * d;// 时间相关项 d0
	y = pStTD->m_x + pStTD->m_h * pStTD->m_x2;
	// 组合当前位置误差和速度误差

	a0 = sqrt(d * d + 8 * pStTD->m_r * fabs(y));
  // 计算带有非线性特性的中间量 a0

	if(fabs(y) > d0)
		a = pStTD->m_x2 + (a0 - d) * Sgn(y) / 2;
	// 误差较大时，采用非线性调节

	else
		a = pStTD->m_x2 + y / pStTD->m_h;
 // 误差较小时，采用线性调节

	if(fabs(a) > d)
		fhan = -1 * pStTD->m_r * Sgn(a);//r?
	else
		fhan = -1 * pStTD->m_r * a / d;
// 控制速度更新的方向和幅度

	pStTD->m_x1 += pStTD->m_T * pStTD->m_x2;// 更新位置
	pStTD->m_x2 += pStTD->m_T * fhan;// 更新速度
}
//r 越大，跟踪越快
//h 越大，滤波越强，越平滑（但延迟增加）
//T 为实际采样时间，需与系统同步







































//pid计算，在此之前FB,DES已经更新过了
void PID_Calc_New(ST_PID *pStPID)
{
	
	static uint8_t forward_flag = 1; //控制前馈的标志位，消除静止震动
	static uint8_t rub_forward_flag=1;
	static uint8_t  Ki_threshold = 5; //积分分离阈值，error小于阈值才开始引入积分项，防止超调
		
    pStPID->fpPreE = pStPID->fpE;

    pStPID->fpE = pStPID->fpDes - pStPID->fpFB; // 计算当前误差
    
    //死区，消除微小抖动
    if (fabs(pStPID->fpE) <= pStPID->fpEMin)
    {       
        pStPID->fpE = 0;
    }
		
		if(fabs((float)pStPID->fpDes)<= 30 ) {rub_forward_flag=0;}
		else {rub_forward_flag=1;}
		
		if(fabs((float)pStPID->fpDes)<= 1.5 ) {forward_flag = 0;}
		else {forward_flag = 1;}
		
		
		
		// 误差限幅
    pStPID->fpE = ClipFloat(pStPID->fpE, -pStPID->fpEMax, pStPID->fpEMax);
		
		
    pStPID->fpSumE += pStPID->fpE; // 计算偏差累积
		
		if (fabs(pStPID->fpE) >= Ki_threshold){	pStPID->fpSumE = 0;}

		// 计算总误差，并且限幅在[-fpSumEMax,fpSumEMax]里
		pStPID->fpSumE = ClipFloat(pStPID->fpSumE, -pStPID->fpSumEMax, pStPID->fpSumEMax);
		
    // 计算P项输出，并且限幅在[-fpUpMax,fpUpMax]里
    pStPID->fpUp = ClipFloat(pStPID->fpKp * pStPID->fpE, -pStPID->fpUpMax, pStPID->fpUpMax);

		pStPID->fpUi = pStPID->fpKi * pStPID->fpSumE;
		
    // 计算D项输出，并且限幅在[-fpUdMax,fpUdMax]里
    pStPID->fpUd = ClipFloat(pStPID->fpKd * (pStPID->fpE - pStPID->fpPreE), -pStPID->fpUdMax, pStPID->fpUdMax);

    // 计算总输出
    pStPID->fpU = pStPID->fpUp + pStPID->fpUi + pStPID->fpUd + pStPID->fpE * pStPID->feedforward * forward_flag + pStPID->forward_rub * Sgn(pStPID->fpDes)*rub_forward_flag; 

    // PID运算总限幅
    pStPID->fpU = ClipFloat(pStPID->fpU, -pStPID->fpUMax, pStPID->fpUMax);
}
