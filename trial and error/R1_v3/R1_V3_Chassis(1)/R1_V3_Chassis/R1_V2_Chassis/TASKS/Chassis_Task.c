#include "Chassis_Task.h"
//用于将导航指定的移动速度分配到四个全向轮上








//下面创建的变量是void SpeedDistribute_Four_OmnidriectionalWhile(ST_Nav *p_nav)要使用的中间量
ST_VECTOR expect_robot_local_Velt;
chassis_run_des  straight_des,rotation_des;

void SpeedDistribute_Four_OmnidriectionalWhile(ST_Nav *p_nav)//以前为Y轴正方向，右为X轴正方向
{
/*			|
		/   |   \       (LENGTH,WIDTH)
		   \|/
	----------------->
		   /|\
	    \   |   /
			|		
	四个全向轮
*/


	fp32 fpQ;
	
	if(p_nav->nav_state == NAV_MANUAL )
	{
		fpQ = 0;
	}
	
	else
	{
		fpQ = ConvertAngle(stRobot.stPos.fpPosQ * RADIAN_10);//在自动导航下，获取车身的偏航角(°)，转化为弧度，便于利用cos计算
	}
	
	Concert_coorindnate(&p_nav->expect_robot_global_velt,&expect_robot_local_Velt, fpQ);//全局坐标系（正直角坐标系）的速度分配到局部坐标系（斜的直角坐标系）

	expect_robot_local_Velt.fpW = p_nav->expect_robot_global_velt.fpW * RADIAN ;
	expect_robot_local_Velt.type = CARTESIAN;
	
	//前Y右X
//	straight_des.leftup    = (-sin(PI/4) * expect_robot_local_Velt.fpX - sin(PI/4) * expect_robot_local_Velt.fpY)/ R_WHEEL * RUN_GEAR_RATIO;
//  straight_des.rightup   = (-sin(PI/4) * expect_robot_local_Velt.fpX + sin(PI/4) * expect_robot_local_Velt.fpY)/ R_WHEEL * RUN_GEAR_RATIO;
//	straight_des.rightdown = ( sin(PI/4) * expect_robot_local_Velt.fpX + sin(PI/4) * expect_robot_local_Velt.fpY)/ R_WHEEL * RUN_GEAR_RATIO;
//	straight_des.leftdown  = ( sin(PI/4) * expect_robot_local_Velt.fpX - sin(PI/4) * expect_robot_local_Velt.fpY)/ R_WHEEL * RUN_GEAR_RATIO;
	
	//前X左Y
straight_des.leftup    = (-sin(PI/4)*expect_robot_local_Velt.fpX + sin(PI/4)*expect_robot_local_Velt.fpY)/ R_WHEEL * RUN_GEAR_RATIO;
straight_des.rightup   = ( sin(PI/4)*expect_robot_local_Velt.fpX + sin(PI/4)*expect_robot_local_Velt.fpY)/ R_WHEEL * RUN_GEAR_RATIO;
straight_des.rightdown = ( sin(PI/4)*expect_robot_local_Velt.fpX - sin(PI/4)*expect_robot_local_Velt.fpY)/ R_WHEEL * RUN_GEAR_RATIO;
straight_des.leftdown  = (-sin(PI/4)*expect_robot_local_Velt.fpX - sin(PI/4)*expect_robot_local_Velt.fpY)/ R_WHEEL * RUN_GEAR_RATIO;
	
	rotation_des.leftup    = R_ROBOT	* expect_robot_local_Velt.fpW / R_WHEEL * RUN_GEAR_RATIO;
    rotation_des.rightup   = R_ROBOT	* expect_robot_local_Velt.fpW / R_WHEEL * RUN_GEAR_RATIO;
	rotation_des.rightdown = R_ROBOT	* expect_robot_local_Velt.fpW / R_WHEEL * RUN_GEAR_RATIO;
	rotation_des.leftdown  = R_ROBOT	* expect_robot_local_Velt.fpW / R_WHEEL * RUN_GEAR_RATIO;
 
	chassis_run.leftup.fpDes    = straight_des.leftup    + rotation_des.leftup;//最后将车身直行和车身旋转时电机所需的角速度加起来分配到电机即可
	chassis_run.rightup.fpDes   = straight_des.rightup   + rotation_des.rightup;
    chassis_run.rightdown.fpDes = straight_des.rightdown + rotation_des.rightdown;
	chassis_run.leftdown.fpDes  = straight_des.leftdown  + rotation_des.leftdown;
}
