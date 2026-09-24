#include "StretchableArm.h"
#include "Robot.h"
#include "CANBsp.h"
#include <string.h>
#include <math.h>

//DM:RAD	LK:DEGREE
uint8_t gTestStateSA;
fp32 rad_lk,rad0_lk= 3.262,rad_dm,rad0_dm= -2.2f,t0_dm,t1_dm=3.4,k_without_KFS=0.096,b_without_KFS=1310,k_with_KFS=0.128,b_with_KFS=1560,k_TtoA=12.5;
//lk's rad,lk's original rad,dm as the same,t0:without KFS,k:torque lk,Torque to Current
void gravityCompensation_SAtest(void){
	switch(gravityCompensation_SA_state){
		case 0://without KFS
			rad_lk = rad0_lk+stretch_LK.angle*RADIAN;
			rad_dm = rad0_dm+stretch_DM.para.pos-rad_lk;
			gTorqueDM = t0_dm*cosf(rad_dm);
			g_cur_LK = k_without_KFS*(stretch_2006.angle+b_without_KFS)*cosf(rad_lk)+k_TtoA*gTorqueDM;
		break;
		case 1://with KFS
			rad_lk = rad0_lk+stretch_LK.angle*RADIAN;
			rad_dm = rad0_dm+stretch_DM.para.pos+rad_lk;
			gTorqueDM = t1_dm*cosf(rad_dm);
			g_cur_LK = k_with_KFS*(stretch_2006.angle+b_with_KFS)*cosf(rad_lk)+k_TtoA*gTorqueDM;
		break;
		
		default:
		break;
	}
	
}

void gravityCompensation_SA(uint8_t state){
    switch (state)
	{
	case 0://without KFS
		rad_lk = RAD0_LK+stretch_LK.angle*RADIAN;
		rad_dm = RAD0_DM+stretch_DM.para.pos+rad_lk;
		gTorqueDM = T2_WITHOUT_KFS_DM*cosf(rad_dm);
		g_cur_LK =K_WITHOUT_KFS*(stretch_2006.angle+B_WITHOUT_KFS)*cosf(rad_lk)+K_T_to_A*gTorqueDM;
		break;
	case 1://with KFS
		rad_lk = RAD0_LK+stretch_LK.angle*RADIAN;
		rad_dm = RAD0_DM+stretch_DM.para.pos+rad_lk;
		gTorqueDM = T2_WITH_KFS_DM*cosf(rad_dm);
		g_cur_LK = K_WITH_KFS*(stretch_2006.angle+B_WITH_KFS)*cosf(rad_lk)+K_T_to_A*gTorqueDM;
		break;
	
	default:
		break;
	}
}

