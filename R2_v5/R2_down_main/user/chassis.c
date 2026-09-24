#include "chassis.h"


ST_VECTOR expect_robot_local_Velt;
float gain_whell_1=1.0f;
float gain_whell_2=1.0f;
float gain_whell_3=1.0f;
float gain_whell_4=1.0f;
//全局速度输入
void SpeedDistribute_Four_OmnidriectionalWhile(ST_VECTOR *p_nav)
{
	
/*			|
		/   |   \(LENGTH,WIDTH)
			 \|/
	-------------->
			 /|\
	  \   |   /
				|		
	四个全向轮
*/

	fp32 cos_q = cosf(robot_pos.fpPosQ - 1.57);
	fp32 sin_q = sinf(robot_pos.fpPosQ - 1.57);
	
	expect_robot_local_Velt.fpW = p_nav->fpW;
	
	if(nav.nav_state!=NAV_INIT&&up_down_state==0){
	expect_robot_local_Velt.fpX = p_nav->fpX * cos_q + p_nav->fpY * sin_q;
	expect_robot_local_Velt.fpY = p_nav->fpY * cos_q - p_nav->fpX * sin_q;}
	else {expect_robot_local_Velt.fpX = p_nav->fpX;
				expect_robot_local_Velt.fpY = p_nav->fpY;}
	
	
	
  straight_des.wheel_1 = (-	SIN_45 * expect_robot_local_Velt.fpX - SIN_45 * expect_robot_local_Velt.fpY) / R_WHEEL * RUN_GEAR_RATIO;
  straight_des.wheel_2 = (SIN_45 * expect_robot_local_Velt.fpX - SIN_45 * expect_robot_local_Velt.fpY) / R_WHEEL * RUN_GEAR_RATIO;
  straight_des.wheel_3 = (SIN_45 * expect_robot_local_Velt.fpX + SIN_45 * expect_robot_local_Velt.fpY) / R_WHEEL * RUN_GEAR_RATIO;
  straight_des.wheel_4 = (- SIN_45 * expect_robot_local_Velt.fpX + SIN_45 * expect_robot_local_Velt.fpY) / R_WHEEL * RUN_GEAR_RATIO;
	
  rotation_des.wheel_1 = -R_ROBOT * expect_robot_local_Velt.fpW / R_WHEEL * RUN_GEAR_RATIO;  
  rotation_des.wheel_2 = -R_ROBOT * expect_robot_local_Velt.fpW / R_WHEEL * RUN_GEAR_RATIO;  
  rotation_des.wheel_3 = -R_ROBOT * expect_robot_local_Velt.fpW / R_WHEEL * RUN_GEAR_RATIO; 
  rotation_des.wheel_4 = -R_ROBOT * expect_robot_local_Velt.fpW / R_WHEEL * RUN_GEAR_RATIO;  
 
	chassis_run.wheel_1.fpDes = (straight_des.wheel_1  + rotation_des.wheel_1)*gain_whell_1;
	chassis_run.wheel_4.fpDes = (straight_des.wheel_4  + rotation_des.wheel_4)*gain_whell_2;
  chassis_run.wheel_3.fpDes = (straight_des.wheel_3  + rotation_des.wheel_3)*gain_whell_3;
	chassis_run.wheel_2.fpDes = (straight_des.wheel_2  + rotation_des.wheel_2)*gain_whell_4;
				
	if(dji_run.flag_2006_V==1)
	{
		straight_des.dji_left = - p_nav->fpY / RpmToRad / R_WHEEL_2006;
		straight_des.dji_right = p_nav->fpY / RpmToRad / R_WHEEL_2006;
	}else {straight_des.dji_left = 0;straight_des.dji_right = 0;}
	
	dji_run.DJI_1.fpDes = straight_des.dji_left;
	dji_run.DJI_2.fpDes = straight_des.dji_right;
	
}
