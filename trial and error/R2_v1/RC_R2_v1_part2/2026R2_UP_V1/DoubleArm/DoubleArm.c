#include "DoubleArm.h"
#include "Robot.h"
#include "CANBsp.h"
#include <string.h>
#include <math.h>

//J60:RAD  DJI:DEGREE
uint8_t gTestStateDA;
fp32 radL,radL0=0.323f,radL0WithSth,radR,radR0=-0.44,radR0WithSth,gTorqueOriginal_L=5.2f,gTorqueOriginal_R=-5.f,gTorqueWithSth;
void gravityCompensation_DAtest(uint8_t state){
	switch(state){
		case 0://original
			radL=radL0+leftShoulder.position_;
			gTorqueLeft=gTorqueOriginal_L*cosf(radL);
			radR=radR0+rightShoulder.position_;
			gTorqueRight=gTorqueOriginal_R*cosf(radR);
		break;
		case 1://with Sth
			radL=radL0WithSth+leftShoulder.position_;
			gTorqueLeft=gTorqueWithSth*cosf(radL);
			radR=radR0WithSth+rightShoulder.position_;
			gTorqueRight=gTorqueWithSth*cosf(radR);
		break;
	}
}

void gravityCompensation_DA(uint8_t state){
	switch(state){
		case 0://without KFS
			radL=RAD_ORIGINAL_LEFT+leftShoulder.position_;
			gTorqueLeft=T_ORIGINAL_J60*cosf(radL);
			radR=RAD_ORIGINAL_RIGHT+rightShoulder.position_;
			gTorqueRight=T_ORIGINAL_J60*cosf(radR);
		break;
		case 1://with KFS
			radL=RAD_WITH_KFS_LEFT+leftShoulder.position_;
			gTorqueLeft=T_WITH_KFS_J60*cosf(radL);
			radR=RAD_WITH_KFS_RIGHT+rightShoulder.position_;
			gTorqueRight=T_WITH_KFS_J60*cosf(radR);
		break;
		case 2://with Weapon
			radL=RAD_WITH_WEAPON_LEFT+leftShoulder.position_;
			gTorqueLeft=T_WITH_WEAPON_J60*cosf(radL);
			radR=RAD_WITH_WEAPON_RIGHT+rightShoulder.position_;
			gTorqueRight=T_WITH_WEAPON_J60*cosf(radR);
	}
}
