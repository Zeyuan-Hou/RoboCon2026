#include "GravityCompensation.h"
#include "ROBOT.h"
#include "CAN_Bsp.h"
#include <string.h>
#include <math.h>

// DM:RAD	A1:RAD
fp32 rad_A1, rad0_A1 = 2.025f, rad_dm, rad0_dm = -2.84f /*1.45f*/, t0_dm = 0.2f, t1_dm = 1.7, k_without_KFS = 0.62f, b_without_KFS = 900, k_with_KFS = 0.827f, b_with_KFS = 1150, K_DM = 0.148f;

void gravityCompensation(void)
{
	switch (gravityCompensation_state)
	{
	case 0: // without KFS
		rad_A1 = rad0_A1 + Motor_A1.RealPos;
		rad_dm = rad0_dm + stretch_DM.para.pos + rad_A1;
		gTorqueDM = t0_dm * cosf(rad_dm);
		gTorqueA1 = k_without_KFS * (stretch_2006.angle + b_without_KFS) / 1000.f * cosf(rad_A1) + K_DM * gTorqueDM;
		break;

	case 1: // with KFS
		rad_A1 = rad0_A1 + Motor_A1.RealPos;
		rad_dm = rad0_dm + stretch_DM.para.pos + rad_A1;
		gTorqueDM = t1_dm * cosf(rad_dm);
		gTorqueA1 = k_with_KFS * (stretch_2006.angle + b_with_KFS) / 1000.f * cosf(rad_A1) + K_DM * gTorqueDM;
		break;

	default:
		break;
	}
}
