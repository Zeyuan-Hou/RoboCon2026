#include "action.h"
// 进入之前使nav.nav_state=NAV_PERMUTATION_PATH;flag_permutation_path=1;
uint8_t flag_iinit = 1;
uint8_t up_stairs_flag = 1;
void Action_choose(void)
{
	switch (part_over_flag)
	{
	case 0:
		Part0_INIT();
		break;
	case 1:
		Part1_action();
		break;
	case 2:
		Part2_123_action();
		break;
	case 3:
		Part2_action();
		break;
	case 4:
		Part2_to_3();
		break;
	case 5:
		Part3_action();
		break;
	case 30:
		Part3_begin();
		break;
	case 31:
		Part3_begin_2();
		break;
	case 20:
		Part2_begin();
		break;
	case 6:
		Part_3_to_R1();
		break;
	case 7:
		Part_3_Retry();
		break;
	default:
		break;
	}
}
void Part0_INIT(void)
{
	if (part_over_flag == 0)
	{
		if (flag_iinit)
		{
			flag_route_plan=1;
			nav.nav_state = CHASSIS_INIT;
			foot.foot_state = FOOT_INIT;
			flag_iinit = 0;
			up_d_state = D_READY_GET_HEAD;	 // 准备取武器头  2
		}
		if (nav.nav_state == CHASSIS_INIT_DONE)
		{
			// nav.nav_state=NAV_PERMUTATION_PATH;
			if (leftup_turn_motor.ControlLoop_State == MULTIPLE_LOOP &&
				rightup_turn_motor.ControlLoop_State == MULTIPLE_LOOP &&
				leftdown_turn_motor.ControlLoop_State == MULTIPLE_LOOP &&
				rightdown_turn_motor.ControlLoop_State == MULTIPLE_LOOP)
				// part_over_flag=1;
				flag_path_1 = 0;
		}
	}
}

void Part1_action(void) // 一区
{
	if (part_over_flag == 1)
	{
		switch (flag_path_1)
		{
		case 0:
			if (flag_choose)
			{
				flag_point_to_point = 1;
				nav.auto_path.number_point = 0;
				nav.nav_state = NAV_POINT_TO_POINT;
				flag_choose = 0;
			}
			if (nav.nav_state == NAV_LOCK) // 路径完成
			{
				flag_choose = 1;
				flag_path_1 = 1;
			}
			break;
		case 1:

			judge_123 = vision_data_recieve.flag_123;
			block_state.vision_judge_123 = vision_data_recieve.flag_123;
			block_state.vision_block_num = vision_data_recieve.count_num;
			flag_path_1 = 2;

			break;
		case 2:
			if (flag_choose)
			{
				flag_point_to_point = 1;
				nav.auto_path.number_point = 1;
				nav.nav_state = NAV_POINT_TO_POINT;
				flag_choose = 0;
			}
			if (nav.nav_state == NAV_LOCK)
			{
				osDelay(1000);
				flag_path_1 = 3;
			}
			break;
		case 3:
			
			if (flag_D_ready_get_head == 10) // 准备完成
			{
				up_d_state = D_GET_LEFT; // 取武器头3
				flag_path_1 = 4;
			}
			break;
		case 4:
			if (flag_D_get_right == 10) // 取完
			{
				up_d_state = D_GET_BLOCK;
				// part_over_flag=2;  //进入二区前置任务
				// judge_123=0;	//改！！！
			}
			break;
		default:
			break;
		}
	}
}
void Part2_begin(void)
{
	if (part_over_flag == 20)
	{
		switch (flag_2_begin)
		{
		case 1:
			up_s_state = S_INIT;
			up_d_state = D_GET_BLOCK;
			judge_123=vision_data_recieve.flag_123;
			block_state.vision_judge_123 = vision_data_recieve.flag_123;
			block_state.vision_block_num = vision_data_recieve.count_num;
			if (flag_S_init == 10 && flag_D_get_block == 10)
			{
				 part_over_flag=2;
				//judge_123 = 5;
			}
			break;
		default:
			break;
		}
	}
}
void Part2_123_action(void) // 上台阶前取123方块
{
	if (part_over_flag == 2)
	{
		switch (judge_123)
		{
		case 0: // 123无方块，直接到二
			if (flag_choose)
			{
				flag_point_to_point = 1;
				nav.auto_path.number_point = 2;
				nav.nav_state = NAV_POINT_TO_POINT;
				flag_choose = 0;
			}
			if (nav.nav_state == NAV_LOCK)
			{
				flag_choose = 1;
				judge_123 = 4;
			}
			break;
		case 1: // 1有方块，到1
			if (flag_choose)
			{
				nav.auto_path.number_permutation = 4;
				nav.nav_state = NAV_PERMUTATION_PATH;
				flag_choose = 0;
			}
			if (nav.nav_state == NAV_LOCK)
			{
				judge_123 = 4;
				flag_choose = 1;
			}
			break;
		case 2: // 2有方块，到2
			if (flag_choose)
			{
				flag_point_to_point = 1;
				nav.auto_path.number_point = 2;
				nav.nav_state = NAV_POINT_TO_POINT;
				flag_choose = 0;
			}
			if (nav.nav_state == NAV_LOCK)
			{
				flag_choose = 1;
				judge_123 = 4;
			}
			break;
		case 3: // 3有方块，到3
			if (flag_choose)
			{
				nav.auto_path.number_permutation = 5;
				nav.nav_state = NAV_PERMUTATION_PATH;
				flag_choose = 0;
			}
			if (nav.nav_state == NAV_LOCK)
			{
				flag_choose = 1;
				judge_123 = 4;
			}
			break;
		case 4:
			if (flag_choose)
			{
				up_s_state = S_INIT;
//				nav.auto_path.number_permutation = 2; // 逆时针
//				nav.nav_state = NAV_PERMUTATION_PATH;
				flag_choose = 0;
			}
			if (flag_S_init==10) // 微调取方块或进二区
			{
				flag_choose = 1;
				judge_123 = 5;
			}
			break;
		case 5:
			if (block_state.vision_judge_123 == 0)
			{
				flag_part_2 = 2;
				part_over_flag = 3;
			}
			else
			{
				if (flag_choose)
				{
					flag_point_block = 1;
					nav.auto_path.get_block = (vision_data_recieve.flag_123);
					nav.nav_state = NAV_POINT_TO_POINT;
					flag_choose = 0;
					judge_123 = 6;
				}
			}
			break;
		case 6:
			if (nav.nav_state == NAV_LOCK) // 取块
			{
				if (block_state.vision_judge_123 == 2)
				{
					up_d_state = D_GET_BLOCK;
					up_s_state = S_GET_BLOCK_UP;
					if (flag_S_get_block_up == 10 || flag_S_get_block_top == 10)
					{
						judge_123 = 7;
					}
				}
				else if (block_state.vision_judge_123)
				{
					if (up_stairs_flag)
					{
						foot.foot_state = FOOT_TEST_STAND_PREPARE; // 8
						up_stairs_flag = 0;
					}
					up_d_state = D_GET_BLOCK;
					up_s_state = S_GET_BLOCK_TOP;
					if (flag_S_get_block_up == 10 || flag_S_get_block_top == 10)
					{
						judge_123 = 7;
						foot.foot_state = FOOT_TEST_SIT; // 6
					}
				}
			}
			break;
		case 7: // 存块 进入二区任务
			up_s_state = S_GIVE_D;
			if (flag_S_give_D == 10)
			{
				flag_part_2 = 2;
				part_over_flag = 3; // 进入二区任务
				block_state.num++;
				block_state.vision_block_num++;
				block_state.pass_block++;
			}
			break;
		default:
			break;
		}
	}
}
int i = 0;
void Part2_action(void) // 二区
{
	if (i < vision_data_recieve.I_max && part_over_flag == 3)//vision_data_recieve.I_max
	{
		switch (flag_part_2)
		{
		case 0:								// 判断面前是否有块，有则微调
			if (!vision_data_recieve.judge) // 面前无块
			{
				flag_part_2 = 2;
			}
			else if (vision_data_recieve.judge) // 面前有块 微调
			{
				if (flag_choose)
				{
					flag_point_block = 1;
					nav.auto_path.get_block = (vision_data_recieve.id * 10 + vision_data_recieve.next_id % 10);
					nav.nav_state = NAV_POINT_TO_POINT;
					flag_choose = 0;
				}
				if (nav.nav_state == NAV_LOCK)
				{
					flag_part_2 = 1;
				}
			}
			break;
		case 1: // 判断如何取
			if ((vision_data_recieve.z2 - vision_data_recieve.z1) > 150)
			{
				up_s_state = S_GET_BLOCK_UP;
			}
			else if ((vision_data_recieve.z2 - vision_data_recieve.z1) < -150)
			{
				up_s_state = S_GET_BLOCK_DOWN;
			}
			if (flag_S_get_block_up == 10 || flag_S_get_block_down == 10)
			{
				flag_part_2 = 2;
			}
			break;
		case 2: // 上台阶前微调
			if (flag_choose)
			{
				flag_point_to_point = 1;
				if (i == 0)
				{
					nav.auto_path.number_point = 2;
				}
				else
				{
					nav.auto_path.number_point = (vision_data_recieve.id * 10 + vision_data_recieve.next_id % 10);
				}
				nav.nav_state = NAV_POINT_TO_POINT;
				flag_choose = 0;
			}
			if (nav.nav_state == NAV_LOCK)
			{

				flag_part_2 = 3;
			}
			break;
		case 3:															 // 判断上台阶还是下台阶
			if ((vision_data_recieve.z2 - vision_data_recieve.z1) > 150) // 上台阶
			{
				if (block_state.num > 0)
				{
					foot_up_G_feedforward_flag = 1;
				}
				if (up_stairs_flag)
				{
					foot.foot_state = FOOT_UP_PREPARE; // 2
					up_stairs_flag = 0;
				}
				if (foot.foot_state == FOOT_CLEAR) // 1
				{
					up_stairs_flag = 1;
					if(i==0)
					{
							flag_part_2 = 4;
					}
					else if(vision_data_recieve.judge==1)
					{
							flag_part_2 = 8;
					}
					else
					{
							flag_part_2 = 10;
					}
					i++;
				}
			}
			else if ((vision_data_recieve.z2 - vision_data_recieve.z1) < 150) // 下台阶
			{
				if (block_state.num > 1)
				{
					foot_down_G_feedforward_flag = 1;
				}
				if (up_stairs_flag)
				{
					foot.foot_state = FOOT_DOWN_PREPARE; // 2
					up_stairs_flag = 0;
				}
				if (foot.foot_state == FOOT_CLEAR) // 1
				{
					up_stairs_flag = 1;
					if(i==0)
					{
							flag_part_2 = 4;
					}
					else if(vision_data_recieve.judge==1)
					{
							flag_part_2 = 8;
					}
					else
					{
							flag_part_2 = 10;
					}
					i++;
				}
			}
			break;
		case 8:																			// 向前取如何存块
			if (block_state.vision_block_num == 1 || block_state.vision_block_num == 2) // 总路上块数1或2
			{
				if (block_state.num == 0) // 第一个块双臂存
				{
					up_s_state = S_GIVE_D;
				}
				else if (block_state.num == 1) // 第二个块单臂存
				{
					flag_part_2 = 10;
				}
			}
			else if (block_state.vision_block_num == 3) // 总路上块数为3
			{
				if (block_state.vision_judge_123 == 0) // 123无块
				{
					if (block_state.pass_block == 0) // 第一个块直接扔
					{
						up_s_state = S_THROW_BLOCK_BACK;
					}
					else if (block_state.pass_block == 1) // 第二个块双臂存
					{
						up_s_state = S_GIVE_D;
					}
					else if (block_state.pass_block == 2) // 第三个块单臂存
					{
						flag_part_2 = 10;
					}
				}
				else if (block_state.vision_judge_123 != 0) // 手中一定有1个块
				{
					if (block_state.pass_block == 1) // 第二个块转身扔
					{
						if (flag_choose)
						{
							nav.auto_path.number_permutation = 8;
							nav.nav_state = NAV_PERMUTATION_PATH;
							flag_choose = 0;
						}
						if (nav.nav_state == NAV_LOCK)
						{
							flag_part_2 = 9;
						}
					}
					else if (block_state.pass_block == 2) // 第三个块单臂存
					{
						flag_part_2 = 10;
					}
				}
			}
			else if (block_state.vision_block_num == 4) // 总路上块数为4
			{
				if (block_state.vision_judge_123 == 0) // 123无块
				{
					if (block_state.pass_block < 2) // 12个块直接扔
					{
						up_s_state = S_THROW_BLOCK_BACK;
					}
					else if (block_state.pass_block == 2) // 第3个双臂存
					{
						up_s_state = S_GIVE_D;
					}
					else if (block_state.pass_block == 3) // 第4个单臂存
					{
						flag_part_2 = 10;
					}
				}
				else if (block_state.vision_judge_123 != 0) // 手中一定有1个块
				{
					if (block_state.pass_block == 1 || block_state.pass_block == 2) // 23个块转身扔
					{
						if (flag_choose)
						{
							nav.auto_path.number_permutation = 8;
							nav.nav_state = NAV_PERMUTATION_PATH;
							flag_choose = 0;
						}
						if (nav.nav_state == NAV_LOCK)
						{
							flag_part_2 = 9;
						}
					}
					else if (block_state.pass_block == 3) // 第4个块单臂存
					{
						flag_part_2 = 10;
					}
				}
			}
			if (flag_S_give_D == 10)//有问题
			{
				block_state.pass_block++;
				block_state.num++;
				flag_part_2 = 10;
			}
			else if (flag_S_throw_block_back == 10)
			{
				block_state.pass_block++;
				flag_part_2 = 10;
			}
			break;
		case 9: // 转身扔块
			up_s_state = S_THROW_BLOCK_FRONT;
			osDelay(2000);
			flag_part_2 = 12;
			break;
		case 12:
			if (flag_choose)
			{
				nav.auto_path.number_permutation = 8;
				nav.nav_state = NAV_PERMUTATION_PATH;
				flag_choose = 0;
			}
			if (nav.nav_state == NAV_LOCK)
			{
				block_state.pass_block++;
				flag_part_2 = 10;
			}
			break;
		case 10: // 
				flag_part_2 = 4;
			break;
		case 4:																												 // 是否取左右方块,取则微调
			if (block_state.vision_block_num == 2 || block_state.vision_block_num == 3 || block_state.vision_block_num == 4) // 路上总块数>=2,不取
			{
				if (i > 1)
				{
					flag_vision_update++;
					osDelay(20);
				}
				flag_part_2 = 0;
			}
			else if ((block_state.vision_block_num == 0 && block_state.num == 0) || block_state.vision_block_num == 1) // 路上总块数<2,取第一个块
			{
				if (fabs(vision_data_recieve.y3 * 1000 - vision_data_recieve.y1) < 550) // 此刻取
				{
					if (flag_choose)
					{
						flag_point_block = 1;
						nav.auto_path.get_block = (vision_data_recieve.id * 10 + vision_data_recieve.extra_id1 % 10);
						nav.nav_state = NAV_POINT_TO_POINT;
						flag_choose = 0;
					}
					if (nav.nav_state == NAV_LOCK)
					{
						flag_part_2 = 5;
					}
				}
				else
				{
					if (i > 1)
					{
						flag_vision_update++;
						osDelay(20);
					}

					flag_part_2 = 0;
				}
			}
			else if (block_state.vision_block_num == 0 && block_state.num == 1) // 取第二个块
			{
				if (fabs(vision_data_recieve.y4 * 1000 - vision_data_recieve.y1) < 550) // 此刻取
				{
					if (flag_choose)
					{
						flag_point_block = 1;
						nav.auto_path.get_block = (vision_data_recieve.id * 10 + vision_data_recieve.extra_id2 % 10);
						nav.nav_state = NAV_POINT_TO_POINT;
						flag_choose = 0;
					}
					if (nav.nav_state == NAV_LOCK)
					{
						flag_part_2 = 5;
					}
				}
				else
				{
					if (i > 1)
					{
						flag_vision_update++;
						osDelay(20);
					}
					flag_part_2 = 0;
				}
			}
			break;
		case 5: // 取块
			if ((vision_data_recieve.z3 - vision_data_recieve.z1) > 150)
			{
				up_s_state = S_GET_BLOCK_UP;
			}
			else if ((vision_data_recieve.z3 - vision_data_recieve.z1) < -150)
			{
				up_s_state = S_GET_BLOCK_DOWN;
			}
			if (flag_S_get_block_up == 10 || flag_S_get_block_down == 10)
			{
				flag_part_2 = 6;
			}
			break;
		case 6:						  //	左右取如何存方块
			if (block_state.num == 1) // 第二个块单臂存
			{
				if (i > 1)
				{
					flag_vision_update++;
					osDelay(20);
				}
				block_state.num++;
				flag_part_2 = 0;
			}
			else if (block_state.num == 0) // 第一个块双臂存
			{
				up_s_state = S_GIVE_D;
				if (flag_S_give_D == 10)
				{
					if (i > 1)
					{
						flag_vision_update++;
						osDelay(20);
					}
					block_state.num++;
					// block_state.pass_block++;
					flag_part_2 = 0;
				}
			}

			break;
		default:
			break;
		}
	}
	if (i == 5)
	{
		//		flag_vision_update++;
		osDelay(1000);
		part_over_flag = 4;
		flag_part_2_to_3 = 1;
	}
}
void Part2_to_3(void)
{
	if (part_over_flag == 4)
	{
		switch (flag_part_2_to_3)
		{
		case 1:
			if (flag_choose)
			{
				flag_point_to_point = 1;
				nav.auto_path.number_point = (vision_data_recieve.id * 10);
				flag_part_3 = vision_data_recieve.id;
				nav.nav_state = NAV_POINT_TO_POINT;
				flag_choose = 0;
			}
			if (nav.nav_state == NAV_LOCK)
			{
				flag_part_2_to_3 = 2;
			}
			break;
		case 2:
			if (up_stairs_flag)
			{
				foot.foot_state = FOOT_DOWN_PREPARE; // 4
				up_stairs_flag = 0;
			}
			if (foot.foot_state == FOOT_CLEAR) // 1
			{
				up_stairs_flag = 1;
				// part_over_flag=5;
				flag_part_2_to_3 = 3;
			}
			break;
		default:
			break;
		}
	}
}

void Part3_action(void) // 3区状态机
{
	if (part_over_flag == 5)
	{
		switch (flag_part_3)
		{
		case 10: // 到9宫格前
			if (flag_choose)
			{
				nav.auto_path.number_permutation = 9;
				nav.nav_state = NAV_PERMUTATION_PATH;
				flag_choose = 0;
			}
			if (nav.nav_state == NAV_LOCK)
			{
				flag_choose = 1;
				flag_part_3 = 3;
			}
			break;
		case 12: // 到9宫格前
			if (flag_choose)
			{
				nav.auto_path.number_permutation = 10;
				nav.nav_state = NAV_PERMUTATION_PATH;
				flag_choose = 0;
			}
			if (nav.nav_state == NAV_LOCK)
			{
				flag_choose = 1;
				flag_part_3 = 3;
			}
			break;
		case 1: // 逆时针转90度，单臂面向9宫格
			if (flag_choose)
			{
				nav.auto_path.number_permutation = 3;
				nav.nav_state = NAV_PERMUTATION_PATH;
				flag_choose = 0;
			}
			if (nav.nav_state == NAV_LOCK)
			{
				flag_choose = 1;
				flag_part_3 = 3;
			}
			break;
		case 3: // 放中层中方块
			if (flag_choose)
			{
				flag_point_block = 1;
				nav.auto_path.get_block = 5;
				nav.nav_state = NAV_POINT_TO_POINT;
				flag_choose = 0;
			}
			if (nav.nav_state == NAV_LOCK)
			{
				flag_part_3 = 6;
				
			}
			break;
		case 6:
			up_s_state = S_PUT_BLOCK_MIDDLE;
			if (flag_S_put_block_middle == 10)
				{
					flag_part_3 = 7;
				}
			break;
		case 5: //
			if (flag_S_get_D_block == 10)
			{
				flag_part_3 = 4;
			}
			break;
		case 4: // 放中层右方块
			if (flag_choose)
			{
				flag_point_block = 1;
				nav.auto_path.get_block = 6;
				nav.nav_state = NAV_POINT_TO_POINT;
				flag_choose = 0;
			}
			if (nav.nav_state == NAV_LOCK)
			{
				up_s_state = S_PUT_BLOCK_MIDDLE;
				if (flag_S_put_block_middle == 10)
				{
					flag_part_3 = 6;
				}
			}
			break;
		case 2: // 放中层左方块
			if (flag_choose)
			{
				flag_point_block = 1;
				nav.auto_path.get_block = 4;
				nav.nav_state = NAV_POINT_TO_POINT;
				flag_choose = 0;
			}
			if (nav.nav_state == NAV_LOCK)
			{
				up_s_state = S_PUT_BLOCK_MIDDLE;
				if (flag_S_put_block_middle == 10)
				{
					flag_part_3 = 5;
				}
			}
			break;
		default:

			break;
		}
	}
}
void Part3_begin(void)
{
	if (part_over_flag == 30)
	{
		switch (flag_part_3_begin)
		{
		case 1:
			up_s_state = S_INIT;
			up_d_state = D_GET_BLOCK;
			if (flag_S_init == 10 && flag_D_get_block == 10)
			{
				flag_part_3 = 10;
				part_over_flag = 5;
			}
			break;
		}
	}
}
void Part3_begin_2(void)
{
	if (part_over_flag == 31)
	{
		switch (flag_part_3_begin_2)
		{
		case 1:
			up_s_state = S_INIT;
			up_d_state = D_GET_BLOCK;
			if (flag_S_init == 10 && flag_D_get_block == 10)
			{
				flag_part_3 = 12;
				part_over_flag = 5;
			}
			break;
		}
	}
}
void Part_3_to_R1(void)
{
	if (part_over_flag == 6)
	{
		switch (flag_3_to_R1)
		{
		case 1:
			up_s_state = S_PUT_BLOCK_MIDDLE;
			if (flag_S_put_block_middle == 10)
			{
				flag_3_to_R1 = 2;
			}
			break;
		}
	}
}



void Part_3_Retry(void)
{
	if (part_over_flag == 7)
	{
		switch (flag_part_3_retry)
		{
		case 1: // 三区启动区微调
			if (flag_choose)
			{
				flag_point_to_point = 1;
				nav.auto_path.number_point = 3;  //测
				nav.nav_state = NAV_POINT_TO_POINT;
				flag_choose = 0;
			}
			
			if (nav.nav_state == NAV_LOCK)
			{
				flag_part_3_retry = 0;
			}
			break;
		case 0:
			up_s_state=S_INIT;
			if (flag_S_init == 10)
			{
				flag_part_3_retry = 4;
			}
			break;
		case 4:
			up_d_state=D_3_INIT;
		  if(flag_D_3_init == 10)
		  {
				flag_part_3_retry = 2;
		  }
			break;
		case 2: //初始化大小臂，使存两块
			up_s_state=S_GET_BLOCK_UP;    //改
			
			if (flag_S_get_block_up == 10)
			{
				flag_part_3_retry = 5;
			}
			break;
		case 5:
			if (flag_choose)
			{
				nav.auto_path.number_permutation = 10;
				nav.nav_state = NAV_PERMUTATION_PATH;
				flag_choose = 0;
			}
			if (nav.nav_state == NAV_LOCK)
			{
				flag_choose = 1;
				flag_part_3_retry = 3;
			}
			break;
		case 3: //切回到正常状态机
			switch(flag_3_choose)
			{
				case 1://还未放过块
					flag_part_3 = 3;
					part_over_flag = 5;
					break;
				case 2://已经放过1块
					flag_part_3 = 4;
					part_over_flag = 5;
					break;
			}
			break;
			
		}
	}
}
