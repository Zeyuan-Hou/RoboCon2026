#include "smc.h"

/*******************************************************************
函数名称：CalSMC(ST_SMC *pStSMC)
函数功能：滑模控制算法
备    注：
********************************************************************/
void CalSMC(ST_SMC *pStSMC)
{
         pStSMC->TD.aim = pStSMC->fpDes;        
         CalTD(&pStSMC->TD);
         pStSMC->fpE = pStSMC->TD.x1 - pStSMC->fpFB;
				 pStSMC->fpU = 1 / pStSMC->b * (pStSMC->TD.x2+ pStSMC->eps * SMC_SatFunc(pStSMC->fpE, pStSMC->dead)+ pStSMC->gain * pStSMC->fpE);
				pStSMC->fpU = Clip(pStSMC->fpU, -pStSMC->fpUMax, pStSMC->fpUMax);
}
float SMC_SatFunc(float in, float d)
{
		if(fabs(in) >= d)
				return Sgn(in);
		else
				return in / d;
}

