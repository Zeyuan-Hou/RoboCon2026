#include "GravityCompensation.h"
fp32 C_TORQUE=1200,torque0=2800.f,rad0=-1.57f,torque1=0,rad1=0;
void GravityCompensation_weaponPlayer(u8 mode)
{
    switch (mode)
    {
    case 0:
        weapon_wrist_torque=C_TORQUE*cos(claw3508.angle*3.1415f/180.f+rad0);
        break;
    case 1:
        weapon_wrist_torque=torque0*cos(claw3508.angle*3.1415f/180.f+rad0);
        break;
    default:
        break;
    }
}
fp32 t_shoulder_0=3.4f,t_shoulder_1=6.5f,rad_shoulder_0=2.55f,rad_shoulder_1=2.55f,elbow_to_shoulder=0.f,
    t_elbow_0=-1.6f,t_elbow_1=-3.5f,rad_elbow_0=0.12f,rad_elbow_1=0.12f,wrist_to_elbow=-0.0004f,
    t_wrist_0=400,t_wrist_1=2200,rad_wrist_0=1.57f,rad_wrist_1=1.57f;   
void GravityCompensation_KFSMaster(u8 mode){
    switch (mode)
    {
    case 0:
        wrist_torque=t_wrist_0*cos(wrist3508.angle*3.1415f/180.f/2.23f+rad_wrist_0);
        elbow_torque=t_elbow_0*cos(elbowJ60.position_+rad_elbow_0)+wrist_to_elbow*(wrist_torque);
        shoulder_torque=t_shoulder_0*cos(shoulderJ60.position_+rad_shoulder_0);
        break;
    case 1:
        wrist_torque=t_wrist_1*cos(wrist3508.angle*3.1415f/180.f/2.23f+rad_wrist_1);
        elbow_torque=t_elbow_1*cos(elbowJ60.position_+rad_elbow_1)+wrist_to_elbow*wrist_torque;
        shoulder_torque=t_shoulder_1*cos(shoulderJ60.position_+rad_shoulder_1)+elbow_to_shoulder*elbow_torque;
        break;
    default:
        break;
    }

}
