#include "vision_relocation.h"

void Vision_Relocation(void)
{
	fp32 fpQ;
	fpQ = ConvertAngle(stRobot.stPos.fpPosQ * RADIAN_10);
	if(fabs(Vision_Data.radar_x - stRobot.stPos.fpPosX)>70 
	 ||fabs(Vision_Data.radar_y - stRobot.stPos.fpPosY)>70)
	{
		stFollowerWheel.stPos.fpPosX = Vision_Data.radar_x + (-sinf(FW_rob_Alpha + fpQ) + sinf(FW_rob_Alpha)) * FW_Rob_Len;
		stFollowerWheel.stPos.fpPosY = Vision_Data.radar_y - (-cosf(FW_rob_Alpha + fpQ) + cosf(FW_rob_Alpha)) * FW_Rob_Len;
	}
}



