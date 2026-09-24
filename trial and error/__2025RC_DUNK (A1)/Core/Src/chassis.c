#include "chassis.h"
/****************************************************************************************************
函数名称: fp32 Handle_OmnidriectionalWhile(ST_VECTOR *expect_robot_local_velt,ST_VECTOR *pos_motor)

函数功能: 给单个全向轮分配速度
输入参数: 
返回参数: 
备   注:	alpha是期望速度和全向轮的夹角
					
					车身的速度分别投影到四个全向轮上，可以分别拿车身Vx和Vy投影，计算公式相对简单，这里是拿总体V直接投影到轮子，意思一样
****************************************************************************************************/
fp32 Handle_OmnidriectionalWhile(ST_VECTOR *expect_robot_local_velt,ST_VECTOR *pos_motor)
{
    fp32 run_velt, alpha;
	  Covert_coordinate(expect_robot_local_velt);
	  Covert_coordinate(pos_motor); //把四个全向轮的位矢转成极坐标，角度分别是45°，135°，-45°，-135°

    alpha = (expect_robot_local_velt->fpThetha - (pos_motor->fpThetha - 90.f))*RADIAN ;//×PI/180，转换为弧度
  
    run_velt = -expect_robot_local_velt->fpLength*RUN_GEAR_RATIO*cos(alpha)/R_WHEEL; // 自研电调反馈是rad/s
		//fpLength*cos(alpha)  是求每个轮子的速度
		//轮子到电机，先除以轮子半径，求出轮子角速度，再×减速比，得到电机速度， 即*RUN_GEAR_RATIO/R_WHEEL
	  if(fabs(run_velt) < 1e-5)
	  {
	  	run_velt = 0;
	  }
	
	  return run_velt;
    
}

	

/****************************************************************************************************
函数名称: void SpeedDistribute_Four_OmnidriectionalWhile(cNav *p_nav)

函数功能: 根据全局坐标下的速度期望，给四个全向轮分配速度
输入参数: 
返回参数: 
备   注:	
				传入的角速度是弧度制

				比之前写的底盘结算更加严谨，考虑到了各种数值，以使最后的的各种速度转化到轮子上是真实所需的速度
****************************************************************************************************/
chassis_velt_t velt_w; 
ST_VECTOR expect_robot_local_Velt;
chassis_run_des  straight_des ,rotation_des;
ST_VECTOR pos_leftup	  = {-LENGTH ,WIDTH  ,0,0,0};
ST_VECTOR pos_rightup   = {LENGTH  ,WIDTH  ,0,0,0};
ST_VECTOR pos_rightdown = {LENGTH  ,-WIDTH ,0,0,0};
ST_VECTOR pos_leftdown 	= {-LENGTH ,-WIDTH ,0,0,0};//四个全向轮距离车中心的位矢，直线运动不需要该参数，只有旋转运动需要
void SpeedDistribute_Four_OmnidriectionalWhile(ST_Nav *p_nav)
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


	fp32 fpQ;
	if(p_nav->nav_state == NAV_MANUAL ||p_nav->nav_state == NAV_GLOBAL_MANUAL||p_nav->nav_state == NAV_VISION_MANUAL)
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
	
	straight_des.leftup    = (-sin_pi_4 * expect_robot_local_Velt.fpX - sin_pi_4 * expect_robot_local_Velt.fpY)/ R_WHEEL * RUN_GEAR_RATIO;
  straight_des.rightup   = (-sin_pi_4 * expect_robot_local_Velt.fpX + sin_pi_4 * expect_robot_local_Velt.fpY)/ R_WHEEL * RUN_GEAR_RATIO;
	straight_des.rightdown = ( sin_pi_4 * expect_robot_local_Velt.fpX + sin_pi_4 * expect_robot_local_Velt.fpY)/ R_WHEEL * RUN_GEAR_RATIO;
	straight_des.leftdown  = ( sin_pi_4 * expect_robot_local_Velt.fpX - sin_pi_4 * expect_robot_local_Velt.fpY)/ R_WHEEL * RUN_GEAR_RATIO;
	
	rotation_des.leftup    = R_ROBOT	* expect_robot_local_Velt.fpW / R_WHEEL * RUN_GEAR_RATIO;
  rotation_des.rightup   = R_ROBOT	* expect_robot_local_Velt.fpW / R_WHEEL * RUN_GEAR_RATIO;
	rotation_des.rightdown = R_ROBOT	* expect_robot_local_Velt.fpW / R_WHEEL * RUN_GEAR_RATIO;
	rotation_des.leftdown  = R_ROBOT	* expect_robot_local_Velt.fpW / R_WHEEL * RUN_GEAR_RATIO;
 
	chassis_run.leftup.fpDes    = straight_des.leftup    + rotation_des.leftup;//最后将车身直行和车身旋转时电机所需的角速度加起来分配到电机即可
	chassis_run.rightup.fpDes   = straight_des.rightup   + rotation_des.rightup;
  chassis_run.rightdown.fpDes = straight_des.rightdown + rotation_des.rightdown;
	chassis_run.leftdown.fpDes  = straight_des.leftdown  + rotation_des.leftdown;

	chassis_feed_forward(straight_des,rotation_des);
	chassis_friction_compensation();
}
//void SpeedDistribute_Four_OmnidriectionalWhile(ST_Nav *p_nav)
//{
///*			|
//		/   |   \(LENGTH,WIDTH)
//			 \|/
//	-------------->
//			 /|\
//	  \   |   /
//				|		
//	四个全向轮
//*/


//	fp32 fpQ;
//	if(p_nav->nav_state == NAV_MANUAL ||p_nav->nav_state == NAV_GLOBAL_MANUAL)
//	{
//		fpQ = 0;
//	}
//	else
//	{
//		fpQ = ConvertAngle(stRobot.stPos.fpPosQ * RADIAN_10);//在自动导航下，获取车身的偏航角(°)，转化为弧度，便于利用cos计算
//	}
//	
//	Concert_coorindnate(&p_nav->expect_robot_global_velt,&expect_robot_local_Velt, fpQ);//全局坐标系（正直角坐标系）的速度分配到局部坐标系（斜的直角坐标系）

//	expect_robot_local_Velt.fpW = p_nav->expect_robot_global_velt.fpW * RADIAN ;
//	expect_robot_local_Velt.type = CARTESIAN;
//	
//	straight_des.leftup    = Handle_OmnidriectionalWhile(&expect_robot_local_Velt,&pos_leftup);
//  straight_des.rightup   = Handle_OmnidriectionalWhile(&expect_robot_local_Velt,&pos_rightup);
//	straight_des.rightdown = Handle_OmnidriectionalWhile(&expect_robot_local_Velt,&pos_rightdown);
//	straight_des.leftdown  = Handle_OmnidriectionalWhile(&expect_robot_local_Velt,&pos_leftdown);
//	
//	Vector_cross(&pos_leftup,    expect_robot_local_Velt.fpW, &velt_w.leftup   );//→	  →  →
//	Vector_cross(&pos_rightup,   expect_robot_local_Velt.fpW, &velt_w.rightup  );//W × R = V
//	Vector_cross(&pos_rightdown, expect_robot_local_Velt.fpW, &velt_w.rightdown);//用向量叉乘的方法求车身旋转角速度下四个全向轮所需的线速度
//	Vector_cross(&pos_leftdown,  expect_robot_local_Velt.fpW, &velt_w.leftdown );//存到velt_v里面

//	rotation_des.leftup    = Handle_OmnidriectionalWhile(&velt_w.leftup,&pos_leftup);//上面一段代码是用于求四个轮子所需的线速度，因此可以推出，此处的Handle_OmnidriectionalWhile
//  rotation_des.rightup   = Handle_OmnidriectionalWhile(&velt_w.rightup,&pos_rightup);//函数是用于计算轮子线速度到电机角速度的分配，即电机角速度到轮子速度的逆变换
//	rotation_des.rightdown = Handle_OmnidriectionalWhile(&velt_w.rightdown,&pos_rightdown);
//	rotation_des.leftdown  = Handle_OmnidriectionalWhile(&velt_w.leftdown,&pos_leftdown);
// 
//	chassis_run.leftup.fpDes    = straight_des.leftup    + rotation_des.leftup;//最后将车身直行和车身旋转时电机所需的角速度加起来分配到电机即可
//	chassis_run.rightup.fpDes   = straight_des.rightup   + rotation_des.rightup;
//  chassis_run.rightdown.fpDes = straight_des.rightdown + rotation_des.rightdown;
//	chassis_run.leftdown.fpDes  = straight_des.leftdown  + rotation_des.leftdown;

//	chassis_feed_forward(straight_des,rotation_des);
//	chassis_friction_compensation();
//}

chassis_run_des friction_compensation_current;
void chassis_friction_compensation(void)
{
	//中间部分斜坡补偿摩擦力，防突变
  friction_compensation_current.leftup = 2400 * chassis_run.leftup.fpDes / 7.0f;
	friction_compensation_current.rightup = 2900 * chassis_run.rightup.fpDes / 7.0f;
	friction_compensation_current.rightdown = 2600 * chassis_run.rightdown.fpDes / 7.0f;
	friction_compensation_current.leftdown = 2400 * chassis_run.leftdown.fpDes / 7.0f;
	//两边饱和
	friction_compensation_current.leftup = ClipFloat(friction_compensation_current.leftup,-2400.0f,2400.0f);
	friction_compensation_current.rightup = ClipFloat(friction_compensation_current.rightup,-2900.0f,2900.0f);
	friction_compensation_current.rightdown = ClipFloat(friction_compensation_current.rightdown,-2600.0f,2600.0f);
	friction_compensation_current.leftdown = ClipFloat(friction_compensation_current.leftdown,-2400.0f,2400.0f);
	
//	//中间部分斜坡补偿摩擦力，防突变
//  friction_compensation_current.leftup = 3300 * chassis_run.leftup.fpDes / 7.0f;
//	friction_compensation_current.rightup = 3400 * chassis_run.rightup.fpDes / 7.0f;
//	friction_compensation_current.rightdown = 3000 * chassis_run.rightdown.fpDes / 7.0f;
//	friction_compensation_current.leftdown = 3000 * chassis_run.leftdown.fpDes / 7.0f;
//	//两边饱和
//	friction_compensation_current.leftup = ClipFloat(friction_compensation_current.leftup,-3300.0f,3300.0f);
//	friction_compensation_current.rightup = ClipFloat(friction_compensation_current.rightup,-3400.0f,3400.0f);
//	friction_compensation_current.rightdown = ClipFloat(friction_compensation_current.rightdown,-3000.0f,3000.0f);
//	friction_compensation_current.leftdown = ClipFloat(friction_compensation_current.leftdown,-3000.0f,3000.0f);

//	//中间部分斜坡补偿摩擦力，防突变
//	if(fabs(nav.expect_robot_global_velt.fpLength)<153)
//	{
//		friction_compensation_current.leftup = 3300 * chassis_run.leftup.fpDes / 10.0f;
//		friction_compensation_current.rightup = 3400 * chassis_run.rightup.fpDes / 10.0f;
//		friction_compensation_current.rightdown = 3000 * chassis_run.rightdown.fpDes / 10.0f;
//		friction_compensation_current.leftdown = 3000 * chassis_run.leftdown.fpDes / 10.0f;
//	}
//	else if(fabs(nav.expect_robot_global_velt.fpLength)<765)
//	{
//		friction_compensation_current.leftup = 0.0714f * chassis_run.leftup.fpDes * chassis_run.leftup.fpDes + 14.714f * chassis_run.leftup.fpDes + 3140.f;
//		friction_compensation_current.rightup = 25.f * chassis_run.rightup.fpDes + 3150.f;
//		friction_compensation_current.rightdown = -0.3571f * chassis_run.rightdown.fpDes * chassis_run.rightdown.fpDes + 40.429f * chassis_run.rightdown.fpDes + 2650.f;
//		friction_compensation_current.leftdown=0.0042f*chassis_run.leftdown.fpDes*chassis_run.leftdown.fpDes*chassis_run.leftdown.fpDes-0.4821f*chassis_run.leftdown.fpDes*chassis_run.leftdown.fpDes+29.762f*chassis_run.leftdown.fpDes+2740.f;
//	}
//	else
//	{
//		friction_compensation_current.leftup = Sgn(friction_compensation_current.leftup)*4100.f;
//		friction_compensation_current.rightup = Sgn(friction_compensation_current.rightup)*4000.0f;
//		friction_compensation_current.rightdown = Sgn(friction_compensation_current.rightdown)*3900.0f;
//		friction_compensation_current.leftdown = Sgn(friction_compensation_current.leftdown)*3600.0f;
//	}
//	//两边饱和
//	friction_compensation_current.leftup = ClipFloat(friction_compensation_current.leftup,-4100.0f,4100.0f);
//	friction_compensation_current.rightup = ClipFloat(friction_compensation_current.rightup,-4000.0f,4000.0f);
//	friction_compensation_current.rightdown = ClipFloat(friction_compensation_current.rightdown,-3900.0f,3900.0f);
//	friction_compensation_current.leftdown = ClipFloat(friction_compensation_current.leftdown,-3600.0f,3600.0f);
}

chassis_run_des feed_forward_current;
chassis_run_des pre_straight_speed_fpDes,now_straight_speed_fpDes;
float K_Feed_Forward = 3;//8;
float K1_Feed_Forward_LU = 4000;//15000;
float K1_Feed_Forward_RU = 4000;//15000;
float K1_Feed_Forward_RD = 4000;//15000;
float K1_Feed_Forward_LD = 4000;//15000;
void chassis_feed_forward(chassis_run_des straight_des,chassis_run_des rotation_des)
{
	now_straight_speed_fpDes.leftup = straight_des.leftup ;//+ rotation_des.leftup;
	now_straight_speed_fpDes.rightup = straight_des.rightup ;//+ rotation_des.rightup;
	now_straight_speed_fpDes.rightdown = straight_des.rightdown ;//+ rotation_des.rightdown;
	now_straight_speed_fpDes.leftdown = straight_des.leftdown ;//+ rotation_des.leftdown;
	
	feed_forward_current.leftup  =  ClipFloat((now_straight_speed_fpDes.leftup - pre_straight_speed_fpDes.leftup)*K1_Feed_Forward_LU + K_Feed_Forward*now_straight_speed_fpDes.leftup,-4000,4000);
	feed_forward_current.rightup  =  ClipFloat((now_straight_speed_fpDes.rightup - pre_straight_speed_fpDes.rightup)*K1_Feed_Forward_RU + K_Feed_Forward*now_straight_speed_fpDes.rightup,-4000,4000);
	feed_forward_current.rightdown  =  ClipFloat((now_straight_speed_fpDes.rightdown - pre_straight_speed_fpDes.rightdown)*K1_Feed_Forward_RD + K_Feed_Forward*now_straight_speed_fpDes.rightdown,-4000,4000);
	feed_forward_current.leftdown  =  ClipFloat((now_straight_speed_fpDes.leftdown - pre_straight_speed_fpDes.leftdown)*K1_Feed_Forward_LD + K_Feed_Forward*now_straight_speed_fpDes.leftdown,-4000,4000);
	
	pre_straight_speed_fpDes.leftup = now_straight_speed_fpDes.leftup; 
	pre_straight_speed_fpDes.rightup = now_straight_speed_fpDes.rightup; 
	pre_straight_speed_fpDes.rightdown = now_straight_speed_fpDes.rightdown; 
	pre_straight_speed_fpDes.leftdown = now_straight_speed_fpDes.leftdown; 
	
	
}


