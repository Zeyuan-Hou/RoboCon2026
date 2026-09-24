#include "auto_path.h"
#include "string.h"
// 在chassis.c中已将此状态归为局部坐标系控制并忽略yaw，故直接给全局速度赋值
void UP_DOWN_Velt_Set(ST_Nav *pNav)
{
	switch (pNav->auto_path.up_down_state)
	{
	case UP_DOWN_STATE_OFF:
		pNav->expect_robot_global_velt.fpX = 0.f;
		pNav->expect_robot_global_velt.fpY = 0.f;
		pNav->expect_robot_global_velt.fpW = 0.f;
		chassis_run.leftup.fpSumE = 0.f;
		chassis_run.rightup.fpSumE = 0.f;
		chassis_run.leftdown.fpSumE = 0.f;
		chassis_run.rightdown.fpSumE = 0.f;
		leftup_turn_motor.pid_inner.fpSumE = 0.f;
		leftup_turn_motor.pid_outer.fpSumE = 0.f;
		rightup_turn_motor.pid_inner.fpSumE = 0.f;
		rightup_turn_motor.pid_outer.fpSumE = 0.f;
		leftdown_turn_motor.pid_inner.fpSumE = 0.f;
		leftdown_turn_motor.pid_outer.fpSumE = 0.f;
		rightdown_turn_motor.pid_inner.fpSumE = 0.f;
		rightdown_turn_motor.pid_outer.fpSumE = 0.f;
		pNav->auto_path.run_time_flag = 0;
		pNav->auto_path.run_time = 0;
		break;

	case UP_APPROACH_1:
		if (pNav->auto_path.run_time_flag == 0)
		{
			pNav->auto_path.run_time = 0;
			pNav->auto_path.run_time_flag = 1;
		}
		if (!foot_down_G_feedforward_flag && !foot_up_G_feedforward_flag)
		{
			pNav->expect_robot_global_velt.fpX = 0.f;
			pNav->expect_robot_global_velt.fpY = -1200.f;
			pNav->expect_robot_global_velt.fpW = 0.f;
		}
		else
		{
			pNav->expect_robot_global_velt.fpX = 0.f;
			pNav->expect_robot_global_velt.fpY = -1200.f;
			pNav->expect_robot_global_velt.fpW = 0.f;
		}
		break;

	case UP_APPROACH_2:
		if (pNav->auto_path.run_time_flag == 0)
		{
			pNav->auto_path.run_time = 0;
			pNav->auto_path.run_time_flag = 1;
		}
		pNav->expect_robot_global_velt.fpX = 0.f;
		pNav->expect_robot_global_velt.fpY = -4600.f;
		pNav->expect_robot_global_velt.fpW = 0.f;
		break;

	case UP_UPSTAIRS:
		if (pNav->auto_path.run_time_flag == 0)
		{
			pNav->auto_path.run_time = 0;
			pNav->auto_path.run_time_flag = 1;
		}
		pNav->expect_robot_global_velt.fpX = 0.f;
		pNav->expect_robot_global_velt.fpY = -2500.f;
		pNav->expect_robot_global_velt.fpW = 0.f;
		break;

	case DOWN_APPROACH_AND_DOWNSTAIRS:
		if (pNav->auto_path.run_time_flag == 0)
		{
			pNav->auto_path.run_time = 0;
			pNav->auto_path.run_time_flag = 1;
		}
		pNav->expect_robot_global_velt.fpX = 0.f;
		pNav->expect_robot_global_velt.fpY = 900.f;
		pNav->expect_robot_global_velt.fpW = 0.f;
		break;

	default:
		break;
	}
}
float t_run, t_rotation;
float Ts = 0.001;

uint8_t flag_flag_lock = 0;
uint8_t flag_path = 0;
uint8_t P_Num, W_Num;				// 默认的初始值就是0
void SET_NAV_PATH_PERMUTATION(void) // 唯一接口，别的方式开启路径可能会由于run_time、rotation_time、P_Num、W_Num和flag_rotation不清零而出错
{
	// nav.auto_path.number_permutation = number;
	// nav.nav_state = NAV_PERMUTATION_PATH;
	nav.auto_path.run_time = 0;
	nav.auto_path.rotation_time = 0;
	nav.auto_path.run_Sumtime = 0;
	path_state = PATH_ONGOING;
	flag_permutation_path = 1;
	flag_rotation = 0;
	P_Num = 0;
	W_Num = 0;
	memset(&Path_Permuta, 0, sizeof(PATH_PERMUTATION));
	memset(&Path_Point, 0, sizeof(PATH_POINT));
	flag_point_permutation = 1;
	point_cnt = 0;
	flag_choose = 1;
	flag_zero = 1;
	nav.auto_path.pos_pid.x.fpSumE = 0;
	nav.auto_path.pos_pid.y.fpSumE = 0;
	nav.auto_path.pos_pid.w.fpSumE = 0;
}
/*************************************************************************************************************************************************
	路径排列的弊端：
	1.由于该方法的原理是根据几个给定数据来得出该条直/曲线的整体数据，其中的T是由函数自己算出来的，因此对于一段未知路径，直接套用该函数的话是无法适应于所有情况的
		比如：对于一段长度任意的直线，由于在匀加匀减过程中V和A是要自己定的，因此无论多长的路径，匀加匀减过程经过的路程是恒定的（即V^2/2A），无法做到很短的路径
					正确做法是at^2=L来求，T计算的公式不再是V/A。
	2.只有走路径，姿态角的旋转需要自己再写一个NavRotation()函数，路径和姿态角相互独立。

	用途：计算每小段路径的fpx,fpy,t，并赋值给Path_Permuta结构体
*************************************************************************************************************************************************/
void Path_Permutation(ST_VECTOR *Point_Start, ST_VECTOR *Point_Inc, ST_VECTOR *V_Start, ST_VECTOR *V_End,
					  PATH_TYPE *Path_Type, float *A, float *R, float *T, uint8_t num_module)
{
	int i;
	for (i = 0; i < num_module; i++)
	{
		if (i == 0)
		{ // 指定启动的速度和方向
			Path_Permuta.V_Start[0].fpLength = V_Start->fpLength;
			Path_Permuta.V_Start[0].fpThetha = V_Start->fpThetha;
			Path_Permuta.V_Start[0].type = V_Start->type;
			Covert_coordinate(Path_Permuta.V_Start);
			// 指定启动位置坐标
			Path_Permuta.Point_Start[0].fpX = Point_Start->fpX;
			Path_Permuta.Point_Start[0].fpY = Point_Start->fpY;
			Path_Permuta.Point_Start[0].type = Point_Start->type;
			Covert_coordinate(Path_Permuta.Point_Start);

			Path_Permuta.num_module = num_module;
		}
		if (i < num_module)
		{
			Path_Permuta.Path_Type[i] = Path_Type[i];
			Covert_coordinate(Path_Permuta.Point_Start + i);
			Covert_coordinate(Path_Permuta.V_Start + i);

			if (Path_Type[i] == LINE)
			{
				if (fabs(A[i]) <= 1e-5)											// 匀速直线，只拿路径坐标增量计算即可
				{																//!!!!!!!!!!这里是通过加速度来判断匀速和加减速的，因此在下面走匀速路线的时候不能只给Point_Inc，同时也得给A[i]置0
					Path_Permuta.Point_Inc[i].fpLength = Point_Inc[i].fpLength; // 给第i段坐标增量赋值
					Path_Permuta.Point_Inc[i].fpThetha = Point_Inc[i].fpThetha;
					Path_Permuta.Point_Inc[i].type = Point_Inc[i].type;
					Covert_coordinate(Path_Permuta.Point_Inc + i);			 // 求出位移增量的fpX和fpY，方便打点
					Path_Permuta.V_End[i].fpX = Path_Permuta.V_Start[i].fpX; // 匀速下终止速度和起始速度一样
					Path_Permuta.V_End[i].fpY = Path_Permuta.V_Start[i].fpY;
					Path_Permuta.V_End[i].type = CARTESIAN;
					Covert_coordinate(Path_Permuta.V_End + i);
					Path_Permuta.T[i] = Path_Permuta.Point_Inc[i].fpLength / Path_Permuta.V_Start[i].fpLength; // 时间等于路程除以速度，用来计算时间
				}
				else // 加速或减速直线，主要需要路径终止速度和加速度，可以依次解算出路程和时间
				{
					Path_Permuta.V_End[i].fpLength = (V_End + i)->fpLength; // 给第i段路径终止速度赋值
					Path_Permuta.V_End[i].fpThetha = (V_End + i)->fpThetha;
					Path_Permuta.V_End[i].type = (V_End + i)->type;
					Covert_coordinate(Path_Permuta.V_End + i);
					Path_Permuta.A[i] = A[i];																						   // 给第i段路径加速度赋值
					Path_Permuta.T[i] = fabs((Path_Permuta.V_End[i].fpLength - Path_Permuta.V_Start[i].fpLength) / Path_Permuta.A[i]); // 加速时间
					Path_Permuta.Point_Inc[i].fpLength = (Path_Permuta.V_End[i].fpLength * Path_Permuta.V_End[i].fpLength - Path_Permuta.V_Start[i].fpLength * Path_Permuta.V_Start[i].fpLength) /
														 (2.f * Path_Permuta.A[i]); // 计算路程
					if (Path_Permuta.Point_Inc[i].fpLength >= 0)					// 路径坐标增加时，方向和初始速度一致
					{
						Path_Permuta.Point_Inc[i].fpThetha = Path_Permuta.V_Start[i].fpThetha;
					}
					else if (Path_Permuta.Point_Inc[i].fpLength < 0) // 路径坐标减小时，方向和初始速度相反
					{
						Path_Permuta.Point_Inc[i].fpThetha = Path_Permuta.V_Start[i].fpThetha + 180.f;
					}
					Path_Permuta.Point_Inc[i].type = POLAR;
					Covert_coordinate(Path_Permuta.Point_Inc + i); // 求出位移增量的fpX和fpY，方便打点
				}
			}

			else if (Path_Type[i] == CIRCLE)							// 匀速圆弧，只需要终止速度（方向）和半径即可，可计算出时间和位移增量
			{															//************！！！！！！！这里有可能会转反，以后需要注意！！！！！**************
				Path_Permuta.V_End[i].fpLength = (V_End + i)->fpLength; // 先给终止速度赋值
				Path_Permuta.V_End[i].fpThetha = (V_End + i)->fpThetha;
				Path_Permuta.V_End[i].type = (V_End + i)->type;
				Covert_coordinate(Path_Permuta.V_End + i);																											   // 最终速度转换为笛卡尔坐标系，得到fpX和fpY
				if (Path_Permuta.V_End[i].fpThetha - Path_Permuta.V_Start[i].fpThetha > 0 && Path_Permuta.V_End[i].fpThetha - Path_Permuta.V_Start[i].fpThetha <= 180) // 这里用来判断最后是顺时针旋转还是逆时针旋转省事
				{
					Path_Permuta.R[i] = R[i];
				}
				else if (Path_Permuta.V_End[i].fpThetha - Path_Permuta.V_Start[i].fpThetha > 180)
				{
					Path_Permuta.R[i] = -R[i];
				}
				else if (Path_Permuta.V_End[i].fpThetha - Path_Permuta.V_Start[i].fpThetha < 0 && Path_Permuta.V_End[i].fpThetha - Path_Permuta.V_Start[i].fpThetha >= -180)
				{
					Path_Permuta.R[i] = -R[i];
				}
				else if (Path_Permuta.V_End[i].fpThetha - Path_Permuta.V_Start[i].fpThetha < -180)
				{
					Path_Permuta.R[i] = R[i];
				}
				Path_Permuta.T[i] = (Path_Permuta.V_End[i].fpThetha - Path_Permuta.V_Start[i].fpThetha) * RADIAN * Path_Permuta.R[i] / Path_Permuta.V_Start[i].fpLength; // 弧长除以速度即为时间
				Path_Permuta.Point_Inc[i].fpX = Path_Permuta.R[i] * (sinf(Path_Permuta.V_End[i].fpThetha * RADIAN) - sinf(Path_Permuta.V_Start[i].fpThetha * RADIAN));	 // 计算x，y方向的位移
				Path_Permuta.Point_Inc[i].fpY = Path_Permuta.R[i] * (cosf(Path_Permuta.V_Start[i].fpThetha * RADIAN) - cosf(Path_Permuta.V_End[i].fpThetha * RADIAN));
				Path_Permuta.Point_Inc[i].type = CARTESIAN;
				Covert_coordinate(Path_Permuta.Point_Inc + i);
			}

			Path_Permuta.Point_Start[i + 1].fpX = Path_Permuta.Point_Start[i].fpX + Path_Permuta.Point_Inc[i].fpX; // 每段路径过后都会更新下一段路径的起点
			Path_Permuta.Point_Start[i + 1].fpY = Path_Permuta.Point_Start[i].fpY + Path_Permuta.Point_Inc[i].fpY;
			Path_Permuta.V_Start[i + 1].fpX = Path_Permuta.V_End[i].fpX; // 每段路径结束后更新下一段的初始速度
			Path_Permuta.V_Start[i + 1].fpY = Path_Permuta.V_End[i].fpY;
		}
		nav.auto_path.run_Sumtime += Path_Permuta.T[i] * 1000;
	}
}
/*********************************************************************************************************
函数名称：void NavPosition(ST_Nav *p_nav,PATH_PERMUTATION *Path_Permuta)
函数功能：根据Path_Permuta数组计算组合路径的速度期望，位置期望
输入:     1.p_nav        输出自动路径的实时参数--在全局坐标系的速度、机器人当前状态、跑自动路径的一些参数number等
					2.Path_Permuta 输入生成路径的基本参数--具体的规划速度加速度坐标等等
备注:生成位置和速度期望，无自转

		每一段路径都会根据Path_Permutation函数（会生成T）进行计算离散化的目标速度和目标位置
**********************************************************************************************************/
void NavPosition(ST_Nav *p_nav, PATH_PERMUTATION *Path_Permuta)
{
	float alpha;
	t_run = Ts * p_nav->auto_path.run_time; // t_run单位就是秒了

	if (Path_Permuta->Path_Type[P_Num] == LINE)
	{
		if (fabs(Path_Permuta->A[P_Num]) <= 1e-5) // 匀速直线
		{										  // 将路径拆分成一段一段的，之后用P_Num标志，每段结束有标志位这样来卡一下路径速度
			p_nav->auto_path.pos_pid.x.fpDes = Path_Permuta->Point_Start[P_Num].fpX + Path_Permuta->V_Start[P_Num].fpX * t_run;
			p_nav->auto_path.pos_pid.y.fpDes = Path_Permuta->Point_Start[P_Num].fpY + Path_Permuta->V_Start[P_Num].fpY * t_run;
			p_nav->auto_path.auto_path_vel.fpVx = Path_Permuta->V_Start[P_Num].fpX;
			p_nav->auto_path.auto_path_vel.fpVy = Path_Permuta->V_Start[P_Num].fpY;
		}
		else // 加速直线，每一段的打点位S0+Vt+0.5at2
		{
			p_nav->auto_path.pos_pid.x.fpDes = Path_Permuta->Point_Start[P_Num].fpX + Path_Permuta->V_Start[P_Num].fpX * t_run +
											   Path_Permuta->A[P_Num] * cosf(Path_Permuta->V_End[P_Num].fpThetha * RADIAN) * pow(t_run, 2.f) / 2.f;

			p_nav->auto_path.pos_pid.y.fpDes = Path_Permuta->Point_Start[P_Num].fpY + Path_Permuta->V_Start[P_Num].fpY * t_run +
											   Path_Permuta->A[P_Num] * sinf(Path_Permuta->V_End[P_Num].fpThetha * RADIAN) * pow(t_run, 2.f) / 2.f;

			p_nav->auto_path.auto_path_vel.fpVx = Path_Permuta->V_Start[P_Num].fpX + Path_Permuta->A[P_Num] * t_run * cosf(Path_Permuta->V_End[P_Num].fpThetha * RADIAN);
			p_nav->auto_path.auto_path_vel.fpVy = Path_Permuta->V_Start[P_Num].fpY + Path_Permuta->A[P_Num] * t_run * sinf(Path_Permuta->V_End[P_Num].fpThetha * RADIAN);
		}
	}
	if (Path_Permuta->Path_Type[P_Num] == CIRCLE)
	{
		alpha = Path_Permuta->V_End[P_Num].fpThetha - Path_Permuta->V_Start[P_Num].fpThetha;

		if (alpha > 180) // 处理顺时针还是逆时针旋转
			alpha -= 360;
		if (alpha < -180)
			alpha += 360;

		p_nav->auto_path.pos_pid.x.fpDes = Path_Permuta->Point_Start[P_Num].fpX + Path_Permuta->R[P_Num] *
																					  (sinf((Path_Permuta->V_Start[P_Num].fpThetha + alpha * t_run / Path_Permuta->T[P_Num]) * RADIAN) -
																					   sinf(Path_Permuta->V_Start[P_Num].fpThetha * RADIAN)); // 每两点间的坐标增加向量
		p_nav->auto_path.pos_pid.y.fpDes = Path_Permuta->Point_Start[P_Num].fpY + Path_Permuta->R[P_Num] *
																					  (cosf(Path_Permuta->V_Start[P_Num].fpThetha * RADIAN) -
																					   cosf((Path_Permuta->V_Start[P_Num].fpThetha + alpha * t_run / Path_Permuta->T[P_Num]) * RADIAN));
		p_nav->auto_path.auto_path_vel.fpVx = Path_Permuta->V_Start[P_Num].fpLength *
											  cosf((Path_Permuta->V_Start[P_Num].fpThetha + alpha * t_run / Path_Permuta->T[P_Num]) * RADIAN); // 匀速圆周下每一点的速度都是总速度的cos，sin
		p_nav->auto_path.auto_path_vel.fpVy = Path_Permuta->V_Start[P_Num].fpLength *
											  sinf((Path_Permuta->V_Start[P_Num].fpThetha + alpha * t_run / Path_Permuta->T[P_Num]) * RADIAN);
	}
}

/*******************************************自转路径路径排列*****************************************************/
void Rotation_Permutation(float *Rotation_Start, float *W_Start, float *Rotation_Inc, float *W_End, float *A_W, float *T_W, uint8_t num_modele_w)
{
	int i;
	for (i = 0; i < num_modele_w; i++)
	{
		if (i == 0)
		{
			Path_Permuta.Rotation_Start[0] = Rotation_Start[0];
			Path_Permuta.W_Start[0] = W_Start[0];

			Path_Permuta.num_module_w = num_modele_w;
		}
		if (i < num_modele_w)
		{
			if (fabs(A_W[i]) <= 1e-5)
			{
				Path_Permuta.Rotation_Inc[i] = Rotation_Inc[i];
				Path_Permuta.W_End[i] = Path_Permuta.W_Start[i];
				Path_Permuta.T_W[i] = Path_Permuta.Rotation_Inc[i] / Path_Permuta.W_Start[i];
			}
			else
			{
				Path_Permuta.W_End[i] = W_End[i];
				Path_Permuta.A_W[i] = A_W[i];
				Path_Permuta.T_W[i] = (Path_Permuta.W_End[i] - Path_Permuta.W_Start[i]) / Path_Permuta.A_W[i];
				Path_Permuta.Rotation_Inc[i] = (Path_Permuta.W_End[i] * Path_Permuta.W_End[i] - Path_Permuta.W_Start[i] * Path_Permuta.W_Start[i]) / (2.f * Path_Permuta.A_W[i]);
			}
			Path_Permuta.W_Start[i + 1] = Path_Permuta.W_End[i];
			Path_Permuta.Rotation_Start[i + 1] = Path_Permuta.Rotation_Start[i] + Path_Permuta.Rotation_Inc[i];
		}
	}
}

/*********************************************************************************************************
函数名称：void NavRotation(ST_Nav *p_nav,PATH_PERMUTATION *Path_Permuta)
函数功能：根据Path_Permuta数组设置组合路径的转速，角度期望
输入:     1.p_nav        输出自动路径的实时参数
					2.Path_Permuta 输入生成路径的基本参数
备注:自转

		自旋进行打点拟合
**********************************************************************************************************/
void NavRotation(ST_Nav *p_nav, PATH_PERMUTATION *Path_Permuta)
{
	t_rotation = Ts * p_nav->auto_path.rotation_time;

	if (t_rotation < Path_Permuta->T_W[W_Num])
	{
		if (flag_path == 0)
		{
			p_nav->auto_path.pos_pid.x.fpDes = p_nav->auto_path.pos_pid.x.fpFB;
			p_nav->auto_path.pos_pid.y.fpDes = p_nav->auto_path.pos_pid.y.fpFB;
		}
		if (fabs(Path_Permuta->A_W[W_Num]) > 1e-5)
		{
			p_nav->auto_path.pos_pid.w.fpDes = Path_Permuta->Rotation_Start[W_Num] + Path_Permuta->W_Start[W_Num] * t_rotation + Path_Permuta->A_W[W_Num] * pow(t_rotation, 2) / 2.f;
			p_nav->auto_path.auto_path_vel.fpW = Path_Permuta->W_Start[W_Num] + Path_Permuta->A_W[W_Num] * t_rotation;
		}
		else
		{
			p_nav->auto_path.pos_pid.w.fpDes = Path_Permuta->Rotation_Start[W_Num] + Path_Permuta->W_Start[W_Num] * t_rotation;
			p_nav->auto_path.auto_path_vel.fpW = Path_Permuta->W_Start[W_Num];
		}
	}
	if (W_Num == Path_Permuta->num_module_w)
	{
		p_nav->auto_path.pos_pid.w.fpDes = Path_Permuta->Rotation_Start[W_Num - 1] + Path_Permuta->Rotation_Inc[W_Num - 1]; // 去掉这一句之后角度会莫名其妙归零，所以留着
		p_nav->auto_path.auto_path_vel.fpW = 0;
	}
}
/*********************************************************************************************************
函数名称：void CheckPathEnd(ST_Nav *p_nav,PATH_PERMUTATION *Path_Permuta)
函数功能：判断路径状态（是否结束），给标志位赋值
输入:     1.p_nav        输出自动路径的实时参数
					2.Path_Permuta 输入生成路径的基本参数
备注:

		监测路径状态是否走完
**********************************************************************************************************/
void CheckPathEnd(ST_Nav *p_nav, PATH_PERMUTATION *Path_Permuta)
{
	if (P_Num < Path_Permuta->num_module && t_run >= Path_Permuta->T[P_Num]) // 总路径还没走完，该分路径已走完
	{
		P_Num++;							   // 进行下一路径
		p_nav->auto_path.run_time = 0;		   // 下一路径初始化
		p_nav->auto_path.pos_pid.x.fpSumE = 0; // 开启路径之前先清零积分项
		p_nav->auto_path.pos_pid.y.fpSumE = 0;
	}
	if (W_Num < Path_Permuta->num_module_w && t_rotation >= Path_Permuta->T_W[W_Num]) // 总路径还没走完，该分路径已走完
	{
		W_Num++; // 进行下一路径
		p_nav->auto_path.rotation_time = 0;
		p_nav->auto_path.pos_pid.w.fpSumE = 0;
	}
	if (W_Num == Path_Permuta->num_module_w && flag_zero == 1)
	{
		t_rotation = 0;
		flag_zero = 0;
	}
	if (P_Num == Path_Permuta->num_module_w && flag_zero == 1)
	{
		t_run = 0;
		flag_zero = 0;
	}
	if (P_Num == Path_Permuta->num_module) // 最后一段减速路径
	{
		if (flag_rotation == 0 || (flag_rotation == 1 && W_Num == Path_Permuta->num_module_w)) // flag_rotation未用上
		{
			if (t_run < 0.36f)
			{
				p_nav->auto_path.pos_pid.x.fpDes = Path_Permuta->Point_Start[P_Num - 1].fpX + Path_Permuta->Point_Inc[P_Num - 1].fpX;
				p_nav->auto_path.pos_pid.y.fpDes = Path_Permuta->Point_Start[P_Num - 1].fpY + Path_Permuta->Point_Inc[P_Num - 1].fpY;

				p_nav->auto_path.auto_path_vel.fpVx = 0;
				p_nav->auto_path.auto_path_vel.fpVy = 0;

				//				p_nav->auto_path.pos_pid.w.fpDes = Path_Permuta->Rotation_Start[W_Num -1] + Path_Permuta->Rotation_Inc[W_Num -1];
				//				p_nav->auto_path.auto_path_vel.fpW = 0;
			}
			if (t_rotation < 0.36f&&flag_rotation == 1)
			{
				p_nav->auto_path.pos_pid.w.fpDes = Path_Permuta->Rotation_Start[W_Num - 1] + Path_Permuta->Rotation_Inc[W_Num - 1];
				p_nav->auto_path.auto_path_vel.fpW = 0;
			}
			else
			{
				P_Num = 0;
				W_Num = 0;
				p_nav->auto_path.run_time = 0;
				p_nav->auto_path.rotation_time = 0;
				p_nav->auto_path.run_Sumtime = 0;
				path_state = PATH_END;
				nav.nav_state = NAV_LOCK;
				flag_lock = 1; // 最后进入停止导航
			}
		}
	}
}
/*********************************************************************************************************
函数名称：void path_permutation_choose(ST_Nav *p_nav)
函数功能：选择路径组合
输入:     组合路径序号：nav.auto_path.number_permutation
备注:
**********************************************************************************************************/
void path_permutation_choose(ST_Nav *p_nav)
{
	switch (p_nav->auto_path.number_permutation)
	{
	case 0: // 前往武器架
		Path_Permutation_Set_1_1(&Path_Permuta, p_nav);
		break;
	case 1: // 前往梅林2
		Path_Permutation_Set_1_2(&Path_Permuta, p_nav);
		//		Rotation_Permutation_Set_2(&Path_Permuta,p_nav);
		//		flag_rotation=1;
		break;
	case 2:
		flag_path = 0;
		Rotation_Permutation_Set_1(&Path_Permuta, p_nav); // 顺时针90度
		flag_rotation = 1;
		break;
	case 3:
		flag_path = 0;
		Rotation_Permutation_Set_2(&Path_Permuta, p_nav); // 逆时针90度
		flag_rotation = 1;
		break;
	case 4: // 前往梅林1
		Path_Permutation_Set_1_0(&Path_Permuta, p_nav);
		// Rotation_Permutation_Set_1(&Path_Permuta,p_nav);
		break;
	case 5: // 前往梅林3
		Path_Permutation_Set_1_3(&Path_Permuta, p_nav);
		// Rotation_Permutation_Set_1(&Path_Permuta,p_nav);
		break;
	case 6: // 梅林1到2
		Path_Permutation_Set_1to2(&Path_Permuta, p_nav);
		break;
	case 7: // 梅林3到2
		Path_Permutation_Set_3to2(&Path_Permuta, p_nav);
		break;
	case 8:
		Rotation_Permutation_Set_3(&Path_Permuta, p_nav); // 转180度
		flag_rotation = 1;
		break;
	case 9: // 梅林10到3区
		Path_Permutation_Set_3_10(&Path_Permuta, p_nav);
		break;
	case 10: // 梅林12到3区
		Path_Permutation_Set_3_12(&Path_Permuta, p_nav);
		break;
	case 11:
		Path_Permutation_Set_3_retry;
		break;
	default:
		break;
	}
}
/*********************************************************************************************************
函数名称：void Path_Permutation_Set(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav)
函数功能：输入路径参数
输入:     Path_Permuta 输入生成路径的基本参数
备注:
		 ************************************************************************
		 **路径参数要求: 直线匀速    参数: 位置增量(极坐标)                       **
		 **             直线加减速  参数: 终止速度(极),加速度(区分正负)           **
		 **             圆弧        参数: 终止速度(极),半径(为正)                **
		 ************************************************************************

		 Point_Start,V_Start只需付一次初值，如果初值为0可以默认初始化:


				 不同路径的参数，共八个路径Num_module，第八个路径为最后减速
					Point_Start,Point_Inc,V_Start,V_End,Path_Type,A,R,T,num_module,P_Num
					所有路径均需Point_Start、V_Start和Path_Type，直线匀速要Point_Inc，直线匀加减速要V_End和A，圆弧需要V_End和R
					路径时间均由Path_Permutation函数算出。
**********************************************************************************************************/
void Path_Permutation_Set_1_1(PATH_PERMUTATION *Path_Permuta, ST_Nav *p_nav) // 一区到武器架
{
	ST_VECTOR Point_Start[5]; // 路程起始坐标
	ST_VECTOR Point_Inc[5];	  // 坐标增量
	ST_VECTOR V_Start[5];	  // 速度起始点
	ST_VECTOR V_End[5];		  // 速度终止点
	PATH_TYPE Path_Type[5];	  // 10段路径各自的类型：直线/圆弧/（贝赛尔）（没写）
	float A[5];				  // 10段各自的加速度
	float R[5];				  // 10段各自的半径（直线给0）
	float T[5];				  // 10段各自的时间
	uint8_t num_module = 2;

	if (P_Num == 0) // 刚开始跑组合路径
	{
		Point_Start[0].fpX = p_nav->auto_path.pos_pid.x.fpFB; // 立刻读取反馈作为x,y起始坐标
		Point_Start[0].fpY = p_nav->auto_path.pos_pid.y.fpFB;
		Point_Start[0].type = CARTESIAN;
	}
	// 每一段大路径的首尾速度均为0，路径切换时也不会发生速度突变
	// 1.斜线匀加速
	// 起始、终止速度和加速度（刚开始启动的时候比较特殊需要给一个总初值，之后会自动拿上一次中止当下一次的start）
	//	V_Start[0].fpLength = 0.001;
	//	V_Start[0].fpThetha = 61.69;
	//	V_Start[0].type = POLAR;
	//
	//	V_End[0].fpLength = 1200;//1500;//
	//	V_End[0].fpThetha = 61.69;
	//	V_End[0].type = POLAR;
	//	Path_Type[0] = LINE;
	//	A[0] = 1200;//2000;
	//
	//	//2.直线匀速
	//	//极坐标位置增量
	//	Point_Inc[1].fpLength = 6;//2600;//
	//  Point_Inc[1].fpThetha =61.69;
	//	Point_Inc[1].type = POLAR;
	//	Path_Type[1] = LINE;
	//	A[1] = 0;
	//
	//	//2.斜线匀减速
	//	//极坐标位置增量
	//	V_End[2].fpLength = 0.001;//1500;//
	//	V_End[2].fpThetha = 61.69;
	//	V_End[2].type = POLAR;
	//	Path_Type[2] = LINE;
	//	A[2] = -1200;//-2300;

	// 1.斜线匀加速
	// 起始、终止速度和加速度（刚开始启动的时候比较特殊需要给一个总初值，之后会自动拿上一次中止当下一次的start）
	V_Start[0].fpLength = 0.001;
	V_Start[0].fpThetha = -29;
	V_Start[0].type = POLAR;

	V_End[0].fpLength = 950; // 1500;//
	V_End[0].fpThetha = -29;
	V_End[0].type = POLAR;
	Path_Type[0] = LINE;
	A[0] = 950; // 2000;

	//	// 2.直线匀速
	//	// 极坐标位置增量
	//	Point_Inc[1].fpLength = 125; // 2600;//
	//	Point_Inc[1].fpThetha = -29;
	//	Point_Inc[1].type = POLAR;
	//	Path_Type[1] = LINE;
	//	A[1] = 0;

	// 3.斜线匀减速
	// 极坐标位置增量
	V_End[1].fpLength = 0.001; // 1500;//
	V_End[1].fpThetha = -29;
	V_End[1].type = POLAR;
	Path_Type[1] = LINE;
	A[1] = -950; //-2300;

	// 赋好值以后只需要调用初始化函数
	Path_Permutation(Point_Start, Point_Inc, V_Start, V_End, Path_Type, A, R, T, num_module); // 将上述路径赋值给Path_Permuta结构体并计算时间
}

void Path_Permutation_Set_1_2(PATH_PERMUTATION *Path_Permuta, ST_Nav *p_nav) // 一区到梅林2
{
	ST_VECTOR Point_Start[10]; // 路程起始坐标
	ST_VECTOR Point_Inc[10];   // 坐标增量
	ST_VECTOR V_Start[10];	   // 速度起始点
	ST_VECTOR V_End[10];	   // 速度终止点
	PATH_TYPE Path_Type[10];   // 10段路径各自的类型：直线/圆弧/（贝赛尔）（没写）
	float A[10];			   // 10段各自的加速度
	float R[10];			   // 10段各自的半径（直线给0）
	float T[10];			   // 10段各自的时间
	uint8_t num_module = 3;

	if (P_Num == 0) // 刚开始跑组合路径
	{
		Point_Start[0].fpX = p_nav->auto_path.pos_pid.x.fpFB; // 立刻读取反馈作为x,y起始坐标
		Point_Start[0].fpY = p_nav->auto_path.pos_pid.y.fpFB;
		Point_Start[0].type = CARTESIAN;
	}
	// 每一段大路径的首尾速度均为0，路径切换时也不会发生速度突变

	// 1.斜线匀加速
	// 起始、终止速度和加速度（刚开始启动的时候比较特殊需要给一个总初值，之后会自动拿上一次中止当下一次的start）
	V_Start[0].fpLength = 0.001;
	V_Start[0].fpThetha = -180;
	V_Start[0].type = POLAR;

	V_End[0].fpLength = 1500; // 1500;//
	V_End[0].fpThetha = -180;
	;
	V_End[0].type = POLAR;
	Path_Type[0] = LINE;
	A[0] = 1500; // 2000;

	// 2.斜线匀速
	// 极坐标位置增量
	Point_Inc[1].fpLength = 1065; // 2600;//
	Point_Inc[1].fpThetha = -180;
	Point_Inc[1].type = POLAR;
	Path_Type[1] = LINE;
	A[1] = 0;

	// 3.斜线减速
	V_End[2].fpLength = 0.001;
	V_End[2].fpThetha = -180;
	V_End[2].type = POLAR;
	A[2] = -1500;
	Path_Type[2] = LINE;
	// 赋好值以后只需要调用初始化函数
	Path_Permutation(Point_Start, Point_Inc, V_Start, V_End, Path_Type, A, R, T, num_module); // 将上述路径赋值给Path_Permuta结构体并计算时间
}
void Path_Permutation_Set_1_3(PATH_PERMUTATION *Path_Permuta, ST_Nav *p_nav) // 一区到梅林3
{
	ST_VECTOR Point_Start[10]; // 路程起始坐标
	ST_VECTOR Point_Inc[10];   // 坐标增量
	ST_VECTOR V_Start[10];	   // 速度起始点
	ST_VECTOR V_End[10];	   // 速度终止点
	PATH_TYPE Path_Type[10];   // 10段路径各自的类型：直线/圆弧/（贝赛尔）（没写）
	float A[10];			   // 10段各自的加速度
	float R[10];			   // 10段各自的半径（直线给0）
	float T[10];			   // 10段各自的时间
	uint8_t num_module = 2;

	if (P_Num == 0) // 刚开始跑组合路径
	{
		Point_Start[0].fpX = p_nav->auto_path.pos_pid.x.fpFB; // 立刻读取反馈作为x,y起始坐标
		Point_Start[0].fpY = p_nav->auto_path.pos_pid.y.fpFB;
		Point_Start[0].type = CARTESIAN;
	}
	// 每一段大路径的首尾速度均为0，路径切换时也不会发生速度突变

	// 1.直线匀加速
	// 起始、终止速度和加速度（刚开始启动的时候比较特殊需要给一个总初值，之后会自动拿上一次中止当下一次的start）
	V_Start[0].fpLength = 0.001;
	V_Start[0].fpThetha = -180;
	V_Start[0].type = POLAR;

	V_End[0].fpLength = 1300; // 1500;//
	V_End[0].fpThetha = -180;
	V_End[0].type = POLAR;
	Path_Type[0] = LINE;
	A[0] = 1300; // 2000;

	// 2.直线减速
	V_End[1].fpLength = 0.001;
	V_End[1].fpThetha = -180;
	V_End[1].type = POLAR;
	A[1] = -1300;
	Path_Type[1] = LINE;
	// 赋好值以后只需要调用初始化函数
	Path_Permutation(Point_Start, Point_Inc, V_Start, V_End, Path_Type, A, R, T, num_module); // 将上述路径赋值给Path_Permuta结构体并计算时间
}
void Path_Permutation_Set_1_0(PATH_PERMUTATION *Path_Permuta, ST_Nav *p_nav) // 一区到梅林1
{
	ST_VECTOR Point_Start[10]; // 路程起始坐标
	ST_VECTOR Point_Inc[10];   // 坐标增量
	ST_VECTOR V_Start[10];	   // 速度起始点
	ST_VECTOR V_End[10];	   // 速度终止点
	PATH_TYPE Path_Type[10];   // 10段路径各自的类型：直线/圆弧/（贝赛尔）（没写）
	float A[10];			   // 10段各自的加速度
	float R[10];			   // 10段各自的半径（直线给0）
	float T[10];			   // 10段各自的时间
	uint8_t num_module = 3;

	if (P_Num == 0) // 刚开始跑组合路径
	{
		Point_Start[0].fpX = p_nav->auto_path.pos_pid.x.fpFB; // 立刻读取反馈作为x,y起始坐标
		Point_Start[0].fpY = p_nav->auto_path.pos_pid.y.fpFB;
		Point_Start[0].type = CARTESIAN;
	}
	// 每一段大路径的首尾速度均为0，路径切换时也不会发生速度突变

	// 1.斜线匀加速
	// 起始、终止速度和加速度（刚开始启动的时候比较特殊需要给一个总初值，之后会自动拿上一次中止当下一次的start）
	V_Start[0].fpLength = 0.001;
	V_Start[0].fpThetha = -180;
	V_Start[0].type = POLAR;

	V_End[0].fpLength = 1500; // 1500;//
	V_End[0].fpThetha = -180;
	;
	V_End[0].type = POLAR;
	Path_Type[0] = LINE;
	A[0] = 1500; // 2000;

	// 2.斜线匀速
	// 极坐标位置增量
	Point_Inc[1].fpLength = 2265; // 2600;//
	Point_Inc[1].fpThetha = -180;
	Point_Inc[1].type = POLAR;
	Path_Type[1] = LINE;
	A[1] = 0;

	// 3.斜线减速
	V_End[2].fpLength = 0.001;
	V_End[2].fpThetha = -180;
	V_End[2].type = POLAR;
	A[2] = -1500;
	Path_Type[2] = LINE;
	// 赋好值以后只需要调用初始化函数
	Path_Permutation(Point_Start, Point_Inc, V_Start, V_End, Path_Type, A, R, T, num_module); // 将上述路径赋值给Path_Permuta结构体并计算时间
}
void Path_Permutation_Set_3to2(PATH_PERMUTATION *Path_Permuta, ST_Nav *p_nav) // 梅林3到2
{
	ST_VECTOR Point_Start[5]; // 路程起始坐标
	ST_VECTOR Point_Inc[5];	  // 坐标增量
	ST_VECTOR V_Start[5];	  // 速度起始点
	ST_VECTOR V_End[5];		  // 速度终止点
	PATH_TYPE Path_Type[5];	  // 10段路径各自的类型：直线/圆弧/（贝赛尔）（没写）
	float A[5];				  // 10段各自的加速度
	float R[5];				  // 10段各自的半径（直线给0）
	float T[5];				  // 10段各自的时间
	uint8_t num_module = 2;

	if (P_Num == 0) // 刚开始跑组合路径
	{
		Point_Start[0].fpX = p_nav->auto_path.pos_pid.x.fpFB; // 立刻读取反馈作为x,y起始坐标
		Point_Start[0].fpY = p_nav->auto_path.pos_pid.y.fpFB;
		Point_Start[0].type = CARTESIAN;
	}
	// 每一段大路径的首尾速度均为0，路径切换时也不会发生速度突变
	// 1.直线匀加速
	// 起始、终止速度和加速度（刚开始启动的时候比较特殊需要给一个总初值，之后会自动拿上一次中止当下一次的start）
	V_Start[0].fpLength = 0.001;
	V_Start[0].fpThetha = -180;
	V_Start[0].type = POLAR;

	V_End[0].fpLength = 1200; // 1500;//
	V_End[0].fpThetha = -180;
	V_End[0].type = POLAR;
	Path_Type[0] = LINE;
	A[0] = 1200; // 2000;

	// 2.直线匀减速
	// 极坐标位置增量
	V_End[1].fpLength = 0.001; // 1500;//
	V_End[1].fpThetha = -180;
	V_End[1].type = POLAR;
	Path_Type[1] = LINE;
	A[1] = -1200; // 2000;
	// 赋好值以后只需要调用初始化函数
	Path_Permutation(Point_Start, Point_Inc, V_Start, V_End, Path_Type, A, R, T, num_module); // 将上述路径赋值给Path_Permuta结构体并计算时间
}
void Path_Permutation_Set_1to2(PATH_PERMUTATION *Path_Permuta, ST_Nav *p_nav) // 梅林1到2
{
	ST_VECTOR Point_Start[5]; // 路程起始坐标
	ST_VECTOR Point_Inc[5];	  // 坐标增量
	ST_VECTOR V_Start[5];	  // 速度起始点
	ST_VECTOR V_End[5];		  // 速度终止点
	PATH_TYPE Path_Type[5];	  // 10段路径各自的类型：直线/圆弧/（贝赛尔）（没写）
	float A[5];				  // 10段各自的加速度
	float R[5];				  // 10段各自的半径（直线给0）
	float T[5];				  // 10段各自的时间
	uint8_t num_module = 2;

	if (P_Num == 0) // 刚开始跑组合路径
	{
		Point_Start[0].fpX = p_nav->auto_path.pos_pid.x.fpFB; // 立刻读取反馈作为x,y起始坐标
		Point_Start[0].fpY = p_nav->auto_path.pos_pid.y.fpFB;
		Point_Start[0].type = CARTESIAN;
	}
	// 每一段大路径的首尾速度均为0，路径切换时也不会发生速度突变
	// 1.直线匀加速
	// 起始、终止速度和加速度（刚开始启动的时候比较特殊需要给一个总初值，之后会自动拿上一次中止当下一次的start）
	V_Start[0].fpLength = 0.001;
	V_Start[0].fpThetha = 0;
	V_Start[0].type = POLAR;

	V_End[0].fpLength = 1200; // 1500;//
	V_End[0].fpThetha = 0;
	V_End[0].type = POLAR;
	Path_Type[0] = LINE;
	A[0] = 1200; // 2000;

	// 2.直线匀减速
	// 极坐标位置增量
	V_End[1].fpLength = 0.001; // 1500;//
	V_End[1].fpThetha = 0;
	V_End[1].type = POLAR;
	Path_Type[1] = LINE;
	A[1] = -1200; // 2000;
	// 赋好值以后只需要调用初始化函数
	Path_Permutation(Point_Start, Point_Inc, V_Start, V_End, Path_Type, A, R, T, num_module); // 将上述路径赋值给Path_Permuta结构体并计算时间
}
void Path_Permutation_Set_3_10(PATH_PERMUTATION *Path_Permuta, ST_Nav *p_nav) // 梅林10到3区
{
	ST_VECTOR Point_Start[10]; // 路程起始坐标
	ST_VECTOR Point_Inc[10];   // 坐标增量
	ST_VECTOR V_Start[10];	   // 速度起始点
	ST_VECTOR V_End[10];	   // 速度终止点
	PATH_TYPE Path_Type[10];   // 10段路径各自的类型：直线/圆弧/（贝赛尔）（没写）
	float A[10];			   // 10段各自的加速度
	float R[10];			   // 10段各自的半径（直线给0）
	float T[10];			   // 10段各自的时间
	uint8_t num_module = 7;

	if (P_Num == 0) // 刚开始跑组合路径
	{
		Point_Start[0].fpX = p_nav->auto_path.pos_pid.x.fpFB; // 立刻读取反馈作为x,y起始坐标
		Point_Start[0].fpY = p_nav->auto_path.pos_pid.y.fpFB;
		Point_Start[0].type = CARTESIAN;
	}
	// 每一段大路径的首尾速度均为0，路径切换时也不会发生速度突变

	// 1.直线匀加速
	// 起始、终止速度和加速度（刚开始启动的时候比较特殊需要给一个总初值，之后会自动拿上一次中止当下一次的start）
	V_Start[0].fpLength = 0.001;
	V_Start[0].fpThetha = -180;
	V_Start[0].type = POLAR;

	V_End[0].fpLength = 1200; // 1500;//
	V_End[0].fpThetha = -180;
	V_End[0].type = POLAR;
	Path_Type[0] = LINE;
	A[0] = 1200; // 2000;

	// 2.圆弧
	// 终止速度和半径
	V_End[1].fpLength = 1200; // 1500;//
	V_End[1].fpThetha = -90;
	R[1] = 580;
	V_End[1].type = POLAR;
	Path_Type[1] = CIRCLE;

	// 3.直线匀加速
	V_Start[2].fpLength = 1200;
	V_Start[2].fpThetha = -90;
	V_Start[2].type = POLAR;

	V_End[2].fpLength = 1500; // 1500;//
	V_End[2].fpThetha = -90;
	V_End[2].type = POLAR;
	Path_Type[2] = LINE;
	A[2] = 1500; // 2000;

	// 4.直线匀速
	Point_Inc[3].fpLength = 1450; // 2600;//
	Point_Inc[3].fpThetha = -90;
	Point_Inc[3].type = POLAR;
	Path_Type[3] = LINE;
	A[3] = 0;

	// 5.圆弧
	// 终止速度和半径
	V_End[4].fpLength = 1500; // 1500;//
	V_End[4].fpThetha = 10;
	R[4] = 500;
	V_End[4].type = POLAR;
	Path_Type[4] = CIRCLE;

	//	//6.斜线匀速
	
	V_Start[5].fpLength = 1500;
	V_Start[5].fpThetha = 10;
	V_Start[5].type = POLAR;

	V_End[5].fpLength = 2500; // 1500;//
	V_End[5].fpThetha = 10;
	V_End[5].type = POLAR;
	Path_Type[5] = LINE;
	A[5] = 2000; // 2000;
	
//	Point_Inc[5].fpLength = 2200; // 2600;//
//	Point_Inc[5].fpThetha = 10;
//	Point_Inc[5].type = POLAR;
//	Path_Type[5] = LINE;
//	A[5] = 0;

	// 7.斜线匀减速
	// 极坐标位置增量
	V_End[6].fpLength = 0.001; // 1500;//r
	V_End[6].fpThetha = 10;
	V_End[6].type = POLAR;
	Path_Type[6] = LINE;
	A[6] = -3000; // 2000;

	// 赋好值以后只需要调用初始化函数
	Path_Permutation(Point_Start, Point_Inc, V_Start, V_End, Path_Type, A, R, T, num_module); // 将上述路径赋值给Path_Permuta结构体并计算时间
}

void Path_Permutation_Set_3_12(PATH_PERMUTATION *Path_Permuta, ST_Nav *p_nav) // 梅林12到3区
{
	ST_VECTOR Point_Start[10]; // 路程起始坐标
	ST_VECTOR Point_Inc[10];   // 坐标增量
	ST_VECTOR V_Start[10];	   // 速度起始点
	ST_VECTOR V_End[10];	   // 速度终止点
	PATH_TYPE Path_Type[10];   // 10段路径各自的类型：直线/圆弧/（贝赛尔）（没写）
	float A[10];			   // 10段各自的加速度
	float R[10];			   // 10段各自的半径（直线给0）
	float T[10];			   // 10段各自的时间
	uint8_t num_module = 7;

	if (P_Num == 0) // 刚开始跑组合路径
	{
		Point_Start[0].fpX = p_nav->auto_path.pos_pid.x.fpFB; // 立刻读取反馈作为x,y起始坐标
		Point_Start[0].fpY = p_nav->auto_path.pos_pid.y.fpFB;
		Point_Start[0].type = CARTESIAN;
	}
	// 每一段大路径的首尾速度均为0，路径切换时也不会发生速度突变

	// 1.直线匀加速
	// 起始、终止速度和加速度（刚开始启动的时候比较特殊需要给一个总初值，之后会自动拿上一次中止当下一次的start）
	V_Start[0].fpLength = 0.001;
	V_Start[0].fpThetha = -180;
	V_Start[0].type = POLAR;

	V_End[0].fpLength = 2000; // 1500;//
	V_End[0].fpThetha = -180;
	V_End[0].type = POLAR;
	Path_Type[0] = LINE;
	A[0] = 2000; // 2000;

	// 2.直线匀速
	Point_Inc[1].fpLength = 2000; // 2600;//
	Point_Inc[1].fpThetha = -180;
	Point_Inc[1].type = POLAR;
	Path_Type[1] = LINE;
	A[1] = 0;

	// 3.圆弧
	// 终止速度和半径
	V_End[2].fpLength = 1500; // 1500;//
	V_End[2].fpThetha = -90;
	R[2] = 580;
	V_End[2].type = POLAR;
	Path_Type[2] = CIRCLE;

//	// 4.直线匀加速
//	V_Start[3].fpLength = 1200;
//	V_Start[3].fpThetha = -90;
//	V_Start[3].type = POLAR;

//	V_End[3].fpLength = 1500; // 1500;//
//	V_End[3].fpThetha = -90;
//	V_End[3].type = POLAR;
//	Path_Type[3] = LINE;
//	A[3] = 1500; // 2000;

	// 5.直线匀速
	Point_Inc[3].fpLength = 1820; // 2600;//
	Point_Inc[3].fpThetha = -90;
	Point_Inc[3].type = POLAR;
	Path_Type[3] = LINE;
	A[3] = 0;

	// 6.圆弧
	// 终止速度和半径
	V_End[4].fpLength = 2000; // 1500;//
	V_End[4].fpThetha = 10;
	R[4] = 500;
	V_End[4].type = POLAR;
	Path_Type[4] = CIRCLE;

	// 7.斜线匀速
	
	V_Start[5].fpLength = 2000;
	V_Start[5].fpThetha = 10;
	V_Start[5].type = POLAR;

	V_End[5].fpLength = 2500; // 1500;//
	V_End[5].fpThetha = 10;
	V_End[5].type = POLAR;
	Path_Type[5] = LINE;
	A[5] = 2000; // 2000;
	
//	Point_Inc[6].fpLength = 3000; // 2600;//
//	Point_Inc[6].fpThetha = 10;
//	Point_Inc[6].type = POLAR;
//	Path_Type[6] = LINE;
//	A[6] = 0;

	// 8.斜线匀减速
	// 极坐标位置增量
	V_End[6].fpLength = 0.001; // 1500;//
	V_End[6].fpThetha = 10;
	V_End[6].type = POLAR;
	Path_Type[6] = LINE;
	A[6] = -3000; // 2000;

	// 赋好值以后只需要调用初始化函数
	Path_Permutation(Point_Start, Point_Inc, V_Start, V_End, Path_Type, A, R, T, num_module); // 将上述路径赋值给Path_Permuta结构体并计算时间
}
void Path_Permutation_Set_3_retry(PATH_PERMUTATION *Path_Permuta, ST_Nav *p_nav) // 梅林3重试
{
	ST_VECTOR Point_Start[5]; // 路程起始坐标
	ST_VECTOR Point_Inc[5];	  // 坐标增量
	ST_VECTOR V_Start[5];	  // 速度起始点
	ST_VECTOR V_End[5];		  // 速度终止点
	PATH_TYPE Path_Type[5];	  // 10段路径各自的类型：直线/圆弧/（贝赛尔）（没写）
	float A[5];				  // 10段各自的加速度
	float R[5];				  // 10段各自的半径（直线给0）
	float T[5];				  // 10段各自的时间
	uint8_t num_module = 3;

	if (P_Num == 0) // 刚开始跑组合路径
	{
		Point_Start[0].fpX = p_nav->auto_path.pos_pid.x.fpFB; // 立刻读取反馈作为x,y起始坐标
		Point_Start[0].fpY = p_nav->auto_path.pos_pid.y.fpFB;
		Point_Start[0].type = CARTESIAN;
	}
	// 每一段大路径的首尾速度均为0，路径切换时也不会发生速度突变
	// 1.直线匀加速
	// 起始、终止速度和加速度（刚开始启动的时候比较特殊需要给一个总初值，之后会自动拿上一次中止当下一次的start）
	V_Start[0].fpLength = 0.001;
	V_Start[0].fpThetha = 0;
	V_Start[0].type = POLAR;

	V_End[0].fpLength = 1200; // 1500;//
	V_End[0].fpThetha = 0;
	V_End[0].type = POLAR;
	Path_Type[0] = LINE;
	A[0] = 1200; // 2000;
	
	Point_Inc[1].fpLength = 1800; // 2600;//
	Point_Inc[1].fpThetha = 0;
	Point_Inc[1].type = POLAR;
	Path_Type[1] = LINE;
	A[1] = 0;

	// 2.直线匀减速
	// 极坐标位置增量
	V_End[2].fpLength = 0.001; // 1500;//
	V_End[2].fpThetha = 0;
	V_End[2].type = POLAR;
	Path_Type[2] = LINE;
	A[2] = -1200; // 2000;
	// 赋好值以后只需要调用初始化函数
	Path_Permutation(Point_Start, Point_Inc, V_Start, V_End, Path_Type, A, R, T, num_module); // 将上述路径赋值给Path_Permuta结构体并计算时间
}
/*********************************************************************************************************
函数名称：void Rotation_Permutation_Set(PATH_PERMUTATION *Path_Permuta,ST_Nav *p_nav)
函数功能：输入路径参数
输入:     Rotation_Permuta 输入生成路径的基本参数
备注:
		 ************************************************************************
		 **路径参数要求: 匀速自转    参数: 角度增量(degree)                      **
		 **             匀加/减速自传  参数:终止速度(degree/s),加速度(区分正负) 	 **
		 ************************************************************************

		 Rotation_Start,W_Start只需付一次初值，如果初值为0可以默认初始化:


				 不同路径的参数，共八个路径Num_module，第八个路径为最后减速
					Rotation_Start,W_Start,Rotation_Inc,W_End,A_W,T_W,num_module_w
					匀加速需要W_Start,W_End,A_W,匀速需要Rotation_Inc且A_W=0
					路径时间均由Rotation_Permutation函数算出。
**********************************************************************************************************/
void Rotation_Permutation_Set_1(PATH_PERMUTATION *Path_Permuta, ST_Nav *p_nav)
{
	float Rotation_Start[10], W_Start[10], Rotation_Inc[10], W_End[10], A_W[10], T_W[10];
	uint8_t num_module_w = 3;

	if (W_Num == 0)
	{
		Rotation_Start[0] = p_nav->auto_path.pos_pid.w.fpFB; // 立刻读取角度反馈
	}

	// 1.匀加速旋转
	W_Start[0] = 0.001;

	W_End[0] = 80;
	A_W[0] = 80;

	// 2.匀速旋转
	Rotation_Inc[1] = 10;
	A_W[1] = 0;

	// 3.匀减速旋转
	W_End[2] = 0.001;
	A_W[2] = -80;

	Rotation_Permutation(Rotation_Start, W_Start, Rotation_Inc, W_End, A_W, T_W, num_module_w);
}
void Rotation_Permutation_Set_2(PATH_PERMUTATION *Path_Permuta, ST_Nav *p_nav)
{
	float Rotation_Start[10], W_Start[10], Rotation_Inc[10], W_End[10], A_W[10], T_W[10];
	uint8_t num_module_w = 3;

	if (W_Num == 0)
	{
		Rotation_Start[0] = p_nav->auto_path.pos_pid.w.fpFB; // 立刻读取角度反馈
	}

	// 1.匀加速旋转
	W_Start[0] = 0.001;

	W_End[0] = -80;
	A_W[0] = -80;

	// 2.匀速旋转
	Rotation_Inc[1] = -10;
	A_W[1] = 0;

	// 3.匀减速旋转
	W_End[2] = 0.001;
	A_W[2] = 80;

	Rotation_Permutation(Rotation_Start, W_Start, Rotation_Inc, W_End, A_W, T_W, num_module_w);
}
void Rotation_Permutation_Set_3(PATH_PERMUTATION *Path_Permuta, ST_Nav *p_nav)
{
	float Rotation_Start[10], W_Start[10], Rotation_Inc[10], W_End[10], A_W[10], T_W[10];
	uint8_t num_module_w = 3;

	if (W_Num == 0)
	{
		Rotation_Start[0] = p_nav->auto_path.pos_pid.w.fpFB; // 立刻读取角度反馈
	}

	// 1.匀加速旋转
	W_Start[0] = 0.001;

	W_End[0] = 100;
	A_W[0] = 100;

	// 2.匀速旋转
	Rotation_Inc[1] = 80;
	A_W[1] = 0;

	// 3.匀减速旋转
	W_End[2] = 0.001;
	A_W[2] = -100;

	Rotation_Permutation(Rotation_Start, W_Start, Rotation_Inc, W_End, A_W, T_W, num_module_w);
}

// 微调start

void path_get_block(ST_Nav *p_nav)
{
	switch (p_nav->auto_path.get_block)
	{
	case 1: // 武馆--->梅林1 未改
		Path_Point.point[0].fpX = -2267.f;
		Path_Point.point[0].fpY = -997.f;
		Path_Point.point[0].fpW = 90.f; // 偏航角 °
		break;
	case 2: // 武馆--->梅林2
		Path_Point.point[0].fpX = -1280.f;
		Path_Point.point[0].fpY = -643.f;
		Path_Point.point[0].fpW = 90.f; // 偏航角 °
		Path_Point.point_num = 1;
		break;
	case 3: // 武馆--->梅林3  未改
		Path_Point.point[0].fpX = 127.f;
		Path_Point.point[0].fpY = -1057.f;
		Path_Point.point[0].fpW = 90.f; // 偏航角 °
		Path_Point.point_num = 1;
		break;
	case 4: // 三区二层左
		Path_Point.point[0].fpX = 1447.f;
		Path_Point.point[0].fpY = -7525.f;
		Path_Point.point[0].fpW = 180.f; // 偏航角 °
		Path_Point.point_num = 1;
		break;
	case 5: // 三区二层中
		Path_Point.point[0].fpX = 844.f;
		Path_Point.point[0].fpY = -9075.f;
		Path_Point.point[0].fpW = 180.f; // 偏航角 °
		Path_Point.point_num = 1;
		break;
	case 6: // 三区二层右
		Path_Point.point[0].fpX = 1840.f;
		Path_Point.point[0].fpY = -8434.f;
		Path_Point.point[0].fpW = 180.f; // 偏航角 °
		Path_Point.point_num = 1;
		break;
	case 25: // 梅林2--->梅林5
		Path_Point.point[0].fpX = -1330.f;
		Path_Point.point[0].fpY = -1840.f;
		Path_Point.point[0].fpW = 90.f;
		Path_Point.point_num = 1;
		break;

	case 23: // 梅林2--->梅林3
		Path_Point.point[0].fpX = -1485.f;
		Path_Point.point[0].fpY = -1968.f;
		Path_Point.point[0].fpW = 180.f;
		Path_Point.point_num = 1;
		break;

	case 21: // 梅林2--->梅林1
		Path_Point.point[0].fpX = -1180.f;
		Path_Point.point[0].fpY = -1996.f;
		Path_Point.point[0].fpW = 0.f;
		Path_Point.point_num = 1;
		break;

	case 58: // 梅林5--->梅林8

		Path_Point.point[0].fpX = -1376.f;
		Path_Point.point[0].fpY = -3035.f;
		Path_Point.point[0].fpW = 90.f;
		Path_Point.point_num = 1;
		break;

	case 14: // 梅林1--->梅林4
		Path_Point.point[0].fpX = -2498.f;
		Path_Point.point[0].fpY = -1914.f;
		Path_Point.point[0].fpW = 90.f;
		Path_Point.point_num = 1;
		break;

	case 36: // 梅林3--->梅林6

		Path_Point.point[0].fpX = -96.f;
		Path_Point.point[0].fpY = -1880.f;
		Path_Point.point[0].fpW = 90.f;
		Path_Point.point_num = 1;
		break;

	case 54: // 梅林5--->梅林4

		Path_Point.point[0].fpX = -1320.f;
		Path_Point.point[0].fpY = -3202.f;
		Path_Point.point[0].fpW = 0.f;
		Path_Point.point_num = 1;
		break;

	case 56: // 梅林5--->梅林6    // 54和56起始点相同
		Path_Point.point[0].fpX = -1516.f;
		Path_Point.point[0].fpY = -3170.f;
		Path_Point.point[0].fpW = 180.f;
		Path_Point.point_num = 1;
		break;

	case 47: // 梅林4--->梅林7

		Path_Point.point[0].fpX = -2538.f;
		Path_Point.point[0].fpY = -3008.f;
		Path_Point.point[0].fpW = 90.f;
		Path_Point.point_num = 1;
		break;

	case 45: // 梅林4--->梅林5

		Path_Point.point[0].fpX = -2710.f;
		Path_Point.point[0].fpY = -3140.f;
		Path_Point.point[0].fpW = 180.f;
		Path_Point.point_num = 1;
		break;

	case 69: // 梅林6--->梅林9

		Path_Point.point[0].fpX = -121.f;
		Path_Point.point[0].fpY = -3195.f;
		Path_Point.point[0].fpW = 90.f;
		Path_Point.point_num = 1;
		break;

	case 65: // 梅林6--->梅林5

		Path_Point.point[0].fpX = -117.f;
		Path_Point.point[0].fpY = -3231.f;
		Path_Point.point[0].fpW = 0.f;
		Path_Point.point_num = 1;
		break;

	case 70: // 梅林7--->梅林10

		Path_Point.point[0].fpX = -2571.f;
		Path_Point.point[0].fpY = -4321.f;
		Path_Point.point[0].fpW = 90.f;
		Path_Point.point_num = 1;
		break;
	case 78: // 梅林7--->梅林8

		Path_Point.point[0].fpX = -2729.f;
		Path_Point.point[0].fpY = -4361.f;
		Path_Point.point[0].fpW = 180.f;
		Path_Point.point_num = 1;
		break;

	case 87: // 梅林8--->梅林7

		Path_Point.point[0].fpX = -1359.f;
		Path_Point.point[0].fpY = -4401.f;
		Path_Point.point[0].fpW = 0.f;
		Path_Point.point_num = 1;
		break;

	case 89: // 梅林8--->梅林9

		Path_Point.point[0].fpX = -1450.f;
		Path_Point.point[0].fpY = -4396.f;
		Path_Point.point[0].fpW = 180.f;
		Path_Point.point_num = 1;
		break;

	case 81: // 梅林8--->梅林11

		Path_Point.point[0].fpX = -1395.f;
		Path_Point.point[0].fpY = -4367.f;
		Path_Point.point[0].fpW = 90.f;
		Path_Point.point_num = 1;
		break;

	case 92: // 梅林9--->梅林12

		Path_Point.point[0].fpX = -162.f;
		Path_Point.point[0].fpY = -4393.f;
		Path_Point.point[0].fpW = 90.f;
		Path_Point.point_num = 1;
		break;

	case 98: // 梅林9--->梅林8

		Path_Point.point[0].fpX = -86.f;
		Path_Point.point[0].fpY = -4425.f;
		Path_Point.point[0].fpW = 0.f;
		Path_Point.point_num = 1;
		break;

	case 101: // 梅林10--->梅林11

		Path_Point.point[0].fpX = -2772.f;
		Path_Point.point[0].fpY = -5562.f;
		Path_Point.point[0].fpW = 180.f;
		Path_Point.point_num = 1;
		break;

	case 121: // 梅林12--->梅林11

		Path_Point.point[0].fpX = -2288.f;
		Path_Point.point[0].fpY = -5729.f;
		Path_Point.point[0].fpW = 0.f;
		Path_Point.point_num = 1;
		break;

	case 112: // 梅林11--->梅林12

		Path_Point.point[0].fpX = -1484.f;
		Path_Point.point[0].fpY = -5606.f;
		Path_Point.point[0].fpW = 180.f;
		Path_Point.point_num = 1;
		break;

	case 110: // 梅林11--->梅林10

		Path_Point.point[0].fpX = -1392.f;
		Path_Point.point[0].fpY = -5607.f;
		Path_Point.point[0].fpW = 0.f;
		Path_Point.point_num = 1;
		break;

	default:
		break;
	}
}
void path_point_choose(ST_Nav *p_nav)
{

	switch (p_nav->auto_path.number_point)
	{
	case 0: // 启动区
		Path_Point.point[0].fpX = 380.f;
		Path_Point.point[0].fpY = 151.f;
		Path_Point.point[0].fpW = 0.f; // 偏航角 °
		Path_Point.point_num = 1;
		break;

	case 1: // 启动区--->武馆
Path_Point.point[0].fpX = 1258.89697f;
        Path_Point.point[0].fpY = -444.071228f;
        Path_Point.point[0].fpW = -1.6f; // 偏航角 °
        Path_Point.point_num = 1;
		break;

	case 2: // 武馆--->梅林2

		Path_Point.point[0].fpX = -1214.f;
		Path_Point.point[0].fpY = -467.f;
		Path_Point.point[0].fpW = 90.f; // 偏航角 °
		Path_Point.point_num = 1;
		break;
	
	case 3: // 三区重试

		Path_Point.point[0].fpX = -4066.f;
		Path_Point.point[0].fpY = -9656.f;
		Path_Point.point[0].fpW = 180.f; // 偏航角 °
		Path_Point.point_num = 1;
		break;

	case 25: // 梅林2--->梅林5

		Path_Point.point[0].fpX = -1360.f;
		Path_Point.point[0].fpY = -1721.f;
		Path_Point.point[0].fpW = 90.f;
		Path_Point.point_num = 1;
		break;

	case 23: // 梅林2--->梅林3

		Path_Point.point[0].fpX = -1628.f;
		Path_Point.point[0].fpY = -1988.f;
		Path_Point.point[0].fpW = 180.f;
		Path_Point.point_num = 1;
		break;

	case 21: // 梅林2--->梅林1

		Path_Point.point[0].fpX = -1042.f;
		Path_Point.point[0].fpY = -2034.f;
		Path_Point.point[0].fpW = 0.f;
		Path_Point.point_num = 1;
		break;

	case 14: // 梅林1--->梅林4

		Path_Point.point[0].fpX = -2510.f;
		Path_Point.point[0].fpY = -2275.f;
		Path_Point.point[0].fpW = 270.f;
		Path_Point.point_num = 1;
		break;

	case 36: // 梅林3--->梅林6

		Path_Point.point[0].fpX = -74.f;
		Path_Point.point[0].fpY = -1697.f;
		Path_Point.point[0].fpW = 90.f;
		Path_Point.point_num = 1;
		break;

	case 54: // 梅林5--->梅林4

		Path_Point.point[0].fpX = -1656.f;
		Path_Point.point[0].fpY = -3217.f;
		Path_Point.point[0].fpW = 180.f;
		Path_Point.point_num = 1;
		break;

	case 56: // 梅林5--->梅林6  //    54和56起始点相同
		Path_Point.point[0].fpX = -1656.f;
		Path_Point.point[0].fpY = -3217.f;
		Path_Point.point[0].fpW = 180.f;
		Path_Point.point_num = 1;
		break;

	case 47: // 梅林4--->梅林7

		Path_Point.point[0].fpX = -2550.f;
		Path_Point.point[0].fpY = -2862.f;
		Path_Point.point[0].fpW = 90.f;
		Path_Point.point_num = 1;
		break;

	case 45: // 梅林4--->梅林5 无

		Path_Point.point[0].fpX = -2240.f;
		Path_Point.point[0].fpY = -1730.f;
		Path_Point.point[0].fpW = 0.f;
		Path_Point.point_num = 1;
		break;

	case 69: // 梅林6--->梅林9

		Path_Point.point[0].fpX = -201.f;
		Path_Point.point[0].fpY = -3569.f;
		Path_Point.point[0].fpW = 270.f;
		Path_Point.point_num = 1;
		break;

	case 75: // 梅林7中心无
		Path_Point.point[0].fpX = -2600.f;
		Path_Point.point[0].fpY = -4380.f;
		Path_Point.point[0].fpW = 180.f;
		Path_Point.point_num = 1;
		break;

	case 76: // 梅林7中心2 无
		Path_Point.point[0].fpX = -2600.f;
		Path_Point.point[0].fpY = -4380.f;
		Path_Point.point[0].fpW = -90.f;
		Path_Point.point_num = 1;
		break;

	case 70: // 梅林7--->梅林10

		Path_Point.point[0].fpX = -2609.f;
		Path_Point.point[0].fpY = -4695.f;
		Path_Point.point[0].fpW = 270.f;
		Path_Point.point_num = 1;
		break;

	case 58: // 梅林5--->梅林8

		Path_Point.point[0].fpX = -1336.f;
		Path_Point.point[0].fpY = -2879.f;
		Path_Point.point[0].fpW = 90.f;
		Path_Point.point_num = 1;
		break;

	case 85: // 梅林8中心 无
		Path_Point.point[0].fpX = -1124.f;
		Path_Point.point[0].fpY = -4440.f;
		Path_Point.point[0].fpW = 90.f;
		Path_Point.point_num = 1;
		break;

	case 86: // 梅林8中心2 无
		Path_Point.point[0].fpX = -1124.f;
		Path_Point.point[0].fpY = -4440.f;
		Path_Point.point[0].fpW = 180.f;
		Path_Point.point_num = 1;
		break;

	case 87: // 梅林8--->梅林7
		Path_Point.point[0].fpX = -1705.f;
		Path_Point.point[0].fpY = -4391.f;
		Path_Point.point[0].fpW = 180.f;
		Path_Point.point_num = 1;
		break;

	case 89: // 梅林8--->梅林9   0

		Path_Point.point[0].fpX = -1091.f;
		Path_Point.point[0].fpY = -4410.f;
		Path_Point.point[0].fpW = 0.f;
		Path_Point.point_num = 1;
		break;

	case 81: // 梅林8--->梅林11

		Path_Point.point[0].fpX = -1401.f;
		Path_Point.point[0].fpY = -4710.f;
		Path_Point.point[0].fpW = 270.f;
		Path_Point.point_num = 1;
		break;

	case 92: // 梅林9--->梅林12

		Path_Point.point[0].fpX = -201.f;
		Path_Point.point[0].fpY = -4774.f;
		Path_Point.point[0].fpW = 270.f;
		Path_Point.point_num = 1;
		break;

	case 100: // 梅林10--->对抗区

		Path_Point.point[0].fpX = -2634.f;
		Path_Point.point[0].fpY = -5901.f;
		Path_Point.point[0].fpW = 270.f;
		Path_Point.point_num = 1;
		break;

	case 120: // 梅林12--->对抗区

		Path_Point.point[0].fpX = -265.f;
		Path_Point.point[0].fpY = -5970.f;
		Path_Point.point[0].fpW = 270.f;
		Path_Point.point_num = 1;
		break;

	case 112: // 梅林11--->梅林12   0

		Path_Point.point[0].fpX = -1127.f;
		Path_Point.point[0].fpY = -5610.f;
		Path_Point.point[0].fpW = 0.f;
		Path_Point.point_num = 1;
		break;

	case 110: // 梅林11--->梅林10   180

		Path_Point.point[0].fpX = -1741.f;
		Path_Point.point[0].fpY = -5591.f;
		Path_Point.point[0].fpW = 180.f;
		Path_Point.point_num = 1;
		break;

	default:
		break;
	}
}

// void Point_Set_1(PATH_POINT *p_point){ // 依次设置目标点(global_x, global_y, yaw)
//     p_point->point.fpX = 0.f;
//     p_point->point.fpY = 0.f; // mm
//     p_point->point.fpW = 0.f; // 偏航角 °
//     p_point->point_num = 1;
// }

// void Point_Set_3(PATH_POINT *p_point)
// {								  // 依次设置目标点(global_x, global_y, yaw)
// 	p_point->point[0].fpW = -3.f; // mm
// 	p_point->flag_only_position[0] = 0;
// 	p_point->flag_only_yaw[0] = 1;

// 	p_point->point[1].fpY = -820.f; // °
// 	p_point->flag_only_position[1] = 3;
// 	p_point->flag_only_yaw[1] = 0;

// 	p_point->point[2].fpX = 1197.f;
// 	p_point->flag_only_position[2] = 2;
// 	p_point->flag_only_yaw[2] = 0;

// 	p_point->point_num = 3;
// }

float dis, delta, dis_x, dis_y;
void Point_to_Point(PATH_POINT *p_point)
{
//	if (nav.auto_path.number_point>10)
//	{
		if (fabs(nav.auto_path.pos_pid.w.fpFB - p_point->point[point_cnt].fpW) > 180.f)
		{
			if (nav.auto_path.pos_pid.w.fpFB > p_point->point[point_cnt].fpW)
			{
				p_point->point[point_cnt].fpW += 360.f;
			}
			else
			{
				p_point->point[point_cnt].fpW -= 360.f;
			}
		}
//		if (fabs(nav.auto_path.pos_pid.w.fpFB - p_point->point[point_cnt].fpW) > 70.f)
//		{
//			Point_Permutation(p_point);
//		}
//	}

	// 1、给一小段路径的目标位置赋值
	if (p_point->flag_only_position[point_cnt])
	{
		nav.auto_path.pos_pid.td_x.aim = p_point->point[point_cnt].fpX;
		nav.auto_path.pos_pid.td_y.aim = p_point->point[point_cnt].fpY;
		nav.auto_path.pos_pid.td_w.aim = nav.auto_path.pos_pid.w.fpFB;
	}
	else if (p_point->flag_only_position[point_cnt] == 2)
	{
		nav.auto_path.pos_pid.td_x.aim = p_point->point[point_cnt].fpX;
		nav.auto_path.pos_pid.td_y.aim = nav.auto_path.pos_pid.y.fpFB;
		nav.auto_path.pos_pid.td_w.aim = nav.auto_path.pos_pid.w.fpFB; // 只走x
	}
	else if (p_point->flag_only_position[point_cnt] == 3)
	{
		nav.auto_path.pos_pid.td_x.aim = nav.auto_path.pos_pid.x.fpFB;
		nav.auto_path.pos_pid.td_y.aim = p_point->point[point_cnt].fpY;
		nav.auto_path.pos_pid.td_w.aim = nav.auto_path.pos_pid.w.fpFB; // 只走y
	}
	else if (p_point->flag_only_yaw[point_cnt])
	{
		nav.auto_path.pos_pid.td_x.aim = nav.auto_path.pos_pid.x.fpFB;
		nav.auto_path.pos_pid.td_y.aim = nav.auto_path.pos_pid.y.fpFB;
		nav.auto_path.pos_pid.td_w.aim = p_point->point[point_cnt].fpW; // 只转向
	}
	else if (!p_point->flag_only_position[point_cnt] && !p_point->flag_only_yaw[point_cnt])
	{
		// nav.auto_path.pos_pid.x.fpDes = p_point->point[point_cnt].fpX;
		// nav.auto_path.pos_pid.y.fpDes = p_point->point[point_cnt].fpY;
		// nav.auto_path.pos_pid.w.fpDes = p_point->point[point_cnt].fpW;
		nav.auto_path.pos_pid.td_x.aim = p_point->point[point_cnt].fpX;
		nav.auto_path.pos_pid.td_y.aim = p_point->point[point_cnt].fpY;
		nav.auto_path.pos_pid.td_w.aim = p_point->point[point_cnt].fpW;
	}
	
	static uint32_t cnt=0;

	// 2、判断是否到达一小段路径的目标位置

	dis = sqrtf(powf(nav.auto_path.pos_pid.x.fpFB - nav.auto_path.pos_pid.td_x.aim, 2.f) +
				powf(nav.auto_path.pos_pid.y.fpFB - nav.auto_path.pos_pid.td_y.aim, 2.f));
	dis_x = fabs(nav.auto_path.pos_pid.x.fpFB - nav.auto_path.pos_pid.td_x.aim);
	dis_y = fabs(nav.auto_path.pos_pid.y.fpFB - nav.auto_path.pos_pid.td_y.aim);
	delta = fabs(nav.auto_path.pos_pid.w.fpFB - nav.auto_path.pos_pid.td_w.aim);
	if(delta>360.f){delta-=360.f;}
	else if(delta<-360.f){delta+=360.f;}
	
	nav.auto_path.pos_pid.x.fpDes = nav.auto_path.pos_pid.td_x.aim;
	nav.auto_path.pos_pid.y.fpDes = nav.auto_path.pos_pid.td_y.aim;
	nav.auto_path.pos_pid.w.fpDes = nav.auto_path.pos_pid.td_w.aim;
	PID_Calc(&nav.auto_path.pos_pid.x, nav.auto_path.pos_pid.x.fpDes, nav.auto_path.pos_pid.x.fpFB);
	PID_Calc(&nav.auto_path.pos_pid.y, nav.auto_path.pos_pid.y.fpDes, nav.auto_path.pos_pid.y.fpFB);
	PID_Calc_Angle(&nav.auto_path.pos_pid.w);
	if(fabs(nav.auto_path.pos_pid.x.fpE)>=50) nav.auto_path.pos_pid.x.fpSumE=0;
	if(fabs(nav.auto_path.pos_pid.y.fpE)>=50) nav.auto_path.pos_pid.y.fpSumE=0;
	if(fabs(nav.auto_path.pos_pid.w.fpE)>=5) nav.auto_path.pos_pid.w.fpSumE=0;

	nav.expect_robot_global_velt.fpX = nav.auto_path.pos_pid.x.fpU;
	nav.expect_robot_global_velt.fpY = nav.auto_path.pos_pid.y.fpU;
	nav.expect_robot_global_velt.fpW = nav.auto_path.pos_pid.w.fpU / 180.f * PI;
	
	if (nav.auto_path.number_point == 1)
	{
		if (dis_x < 4.f && dis_y < 4.f && delta < 1.f && !flag_point_end) cnt++;
		if(cnt>500)
		{
			cnt=0;
			if (point_cnt < p_point->point_num - 1) // 非最后一段路径，直接在NAV_POINT_TO_POINT状态下切换到下一段路径
			{
				point_cnt++;
				nav.auto_path.pos_pid.x.fpSumE = 0;
				nav.auto_path.pos_pid.y.fpSumE = 0;
				nav.auto_path.pos_pid.w.fpSumE = 0;
			}
			else if (point_cnt == p_point->point_num - 1) // 最后一段路径
			{
				flag_point_end = 1;
				point_tim = 0;
			}
		}
	}
	else
	{
		
		if (dis< 25.f && delta < 2.5f && !flag_point_end)
		{
			if (point_cnt < p_point->point_num - 1) // 非最后一段路径，直接在NAV_POINT_TO_POINT状态下切换到下一段路径
			{
				point_cnt++;
				nav.auto_path.pos_pid.x.fpSumE = 0;
				nav.auto_path.pos_pid.y.fpSumE = 0;
				nav.auto_path.pos_pid.w.fpSumE = 0;
			}
			else if (point_cnt == p_point->point_num - 1) // 最后一段路径
			{
				flag_point_end = 1;
				point_tim = 0;
			}
		}
	}
	if (flag_point_end) // 最后一段路径，给0.3s精调，然后切换到NAV_LOCK状态
	{
		flag_lock = 1;
		nav.nav_state = NAV_LOCK;
		nav.auto_path.pos_pid.x.fpSumE = 0;
		nav.auto_path.pos_pid.y.fpSumE = 0;
		nav.auto_path.pos_pid.w.fpSumE = 0;
		nav.auto_path.pos_pid.x.fpU = 0;
    nav.auto_path.pos_pid.y.fpU = 0;
    nav.auto_path.pos_pid.w.fpU = 0;
		flag_point_end = 0;
	    // flag_point_to_point = 1;
	}

	// 3、计算速度输出
//	CalTD(&nav.auto_path.pos_pid.td_x);
//	CalTD(&nav.auto_path.pos_pid.td_y);
//	CalTD(&nav.auto_path.pos_pid.td_w);
	
}

void Point_Permutation(PATH_POINT *p_point)
{
	p_point->point[point_cnt + 2] = p_point->point[point_cnt];

	// 第一段只变XY
	p_point->point[point_cnt].fpX = (nav.auto_path.pos_pid.x.fpFB + p_point->point[point_cnt].fpX) / 2;
	p_point->point[point_cnt].fpY = (nav.auto_path.pos_pid.y.fpFB + p_point->point[point_cnt].fpY) / 2;
	p_point->point[point_cnt].fpW = nav.auto_path.pos_pid.w.fpFB;

	// 第二段只变W
	p_point->point[point_cnt + 1].fpX = p_point->point[point_cnt].fpX;
	p_point->point[point_cnt + 1].fpY = p_point->point[point_cnt].fpY;
	p_point->point[point_cnt + 1].fpW = p_point->point[point_cnt + 2].fpW;

	p_point->point_num += 2;
}
