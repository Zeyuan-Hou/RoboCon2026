#include "State_Machine.h"

// 电机目标位置矩阵
extern uint8_t mode;
uint8_t DM = 0, M2006 = 1, A1 = 2, J60 = 3, M3508 = 4;

uint8_t init_idle = 0, init_for_400 = 1, init_temporary = 2, init_for_mid_square = 3, init_for_top_square = 4;
float init_pos[5][5] = {{-3.58, 50, 0.25, 0.02, 93}, {-4.03, 3, 0.77, 0.02, 5}, {-2.53, 120, 0.34, 0.02, 5}, {-3.5, 450, 0.48, 0.02, 5}, {-4.35, 3, -0.15, 0.02, 5}};

uint8_t horizontal = 0, vertical = 1;
float get_weapon_pos[2][5] = {{0, 0, 0, 0, 92}, {0, 0, 0, 0, 178}};

uint8_t down_200 = 1, up_200 = 2, up_400 = 3, ground = 4;
uint8_t constant = 0, left = 1, forward = 2, right = 3;
float get_kfs_pos[5][4][5] = {
    {{0, 0, 0, 0, 10}, {0, 0, 0, -1.56, 0}, {0, 0, 0, 0.02, 0}, {0, 0, 0, 1.584, 0}},
    {{0, 0, -0.2, 0, 0}, {-4.05, 720, 2.07, -1.56, 0}, {-4.07, 505, 2.07, 0.02, 0}, {-4.05, 720, 2.07, 1.584, 0}},
    {{0, -240, 0, 0, 0}, {-2.58, 460, 1.84, -1.56, 0}, {-2.56, 250, 1.86, 0.02, 0}, {-2.56, 460, 1.84, 1.584, 0}},
    {{0, -240, 0, 0, 0}, {0, 0, 0, 0, 0}, {-2.86, 440, 1.533, 0.02, 0}, {0, 0, 0, 0, 0}},
    {{0, 0, -0.2, 0, 0}, {-4.37, 430, 1.73, -1.56, 0}, {0, 0, 0, 0, 0}, {-4.37, 430, 1.73, 1.584, 0}}};
float throw_kfs_pos[5] = {-1.45, 300, 0.00, 0.02, 0};
float store_kfs_pos[5] = {-1.05, 20, 0.33, 0.02, 0};

float place_mid_kfs_pos_near[5] = {-3.68, 580, 0.575, 0.02, 0};
float place_mid_kfs_pos_far[5] = {-3.43, 700, 0.86, 0.02, 0};
float get_kfs_behind_pos[5] = {-0.82, 250, 0.20, 0.02, 0};
float get_kfs_from_r1_pos_left[5] = {-1.865, 760, -0.53, 1.42, 0};
float get_kfs_from_r1_pos_right[5] = {-1.865, 760, -0.53, -1.40, 0};
float init_for_400_left[5] = {-2.6, 3, 1.12, 1.584, 0};
float init_for_400_right[5] = {-2.6, 3, 1.12, -1.56, 0};
float place_top_kfs_pos_near[2][5] = {{-3.65, 525, 0.02, 0.02, 0}, {-3.79, 820, 0.495, 0.02, 0}};
float place_top_kfs_pos_far[2][5] = {{-3.72, 525, 0.3, 0.02, 0}, {-3.68, 820, 0.49, 0.02, 0}};

void StateMachine()
{
    switch (ace.act_state)
    {
    case INIT:
        Init_Task();
        break;

    case INIT_PICK:
        Init_Pick_Task();
        break;

    case SILENT: // 对于需要重复执行的动作，如GET_WEAPON、COLLECT_KFS，执行完进入SILENT缓冲状态
        ace.set_flag = 0;
        ace.tim = 0;
        break;

    case IDLE: // 对于非重复执行的动作，执行完进入IDLE状态，等待下一次指令；SILENT状态接收到下板的解除指令进入IDLE状态
        Recover_Task();
        break;

    case GET_WEAPON:
        Get_Weapon_Task();
        break;

    case COMBINE_PREPARE:
        Combine_Prepare_Task();
        break;

    case COMBINE_END:
        Combine_End_Task();
        break;

    case RE_GET_WEAPON: // 只有单项赛需要取多个武器头对接需要用到
        Re_Get_Weapon_Task();
        break;

    case COLLECT_PREPARE:
        Collect_Prepare_Task();
        break;

    case COLLECT_KFS:
        Collect_KFS_Task();
        break;

    case PULL_KFS_TEMPORARY:
        Pull_KFS_Temporary_Task();
        break;

    case PULL_KFS_HOLD:
        Pull_KFS_Hold_Task();
        break;

    case THROW_KFS:
        Throw_KFS_Task();
        break;

    case STORE_KFS:
        Store_KFS_Task();
        break;

    case PLACE_MID_KFS_NEAR:
        Place_Mid_KFS_Task_Near();
        break;

    case PLACE_MID_KFS_FAR:
        Place_Mid_KFS_Task_Far();
        break;

    case GET_KFS_BEHIND:
        if (flag_stand)
        {
            index_stand = 1;
            ace.act_state = TRANSFER_FROM_TOP;
            flag_stand = 0;
        }
        else
        {
            Get_KFS_Behind_Task();
        }
        break;

    case PLACE_TOP_KFS_NEAR:
        if (flag_stand)
        {
            index_stand = 2;
            ace.act_state = TRANSFER_FROM_TOP;
            flag_stand = 0;
        }
        else
        {
            Place_Top_KFS_Task_Near();
        }
        break;

    case PLACE_TOP_KFS_FAR:
        if (flag_stand)
        {
            index_stand = 3;
            ace.act_state = TRANSFER_FROM_TOP;
            flag_stand = 0;
        }
        else
        {
            Place_Top_KFS_Task_Far();
        }
        break;

    case GET_KFS_FROM_R1:
        Get_KFS_From_R1_Task();
        break;

    case GET_KFS_FROM_R1_BACK:
        Get_KFS_From_R1_Back_Task();
        break;

    case TRANSFER_FOR_400:
        Transfer_For_400_Task();
        break;

    case TRANSFER_FOR_TEMPORARY:
        Transfer_For_Temporary_Task();
        break;

    case TRANSFER_FOR_TOP:
        Transfer_For_Top_Task();
        break;

    case TRANSFER_FROM_TOP:
        Transfer_From_Top_Task();
        break;

    case INIT_FOR_3:
        Init_For_3_Task();
        break;

    case TRIAL_LEFT:
        flag_trial_orient = 1;
        Trial_Task();
        break;

    case TRIAL_MID:
        flag_trial_orient = 2;
        Trial_Task();
        break;

    case TRIAL_RIGHT:
        flag_trial_orient = 3;
        Trial_Task();
        break;

    case ALL_PUSH:
        All_Push_Task();
        break;

    default:
        break;
    }
}

void Init_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.set_flag = 1;
    }
    if (ace.tim < 800)
    {
        ace.tim++;
        G_ff.flag = 0;
        m3508_ctrl_flag = 1;
        ace.real_time_target_pos[DM] = linear_target_curve(ace.init_pos[DM], init_pos[init_idle][DM], ace.tim - 300, 500);
        ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], init_pos[init_idle][M2006], ace.tim - 300, 500);
        ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], init_pos[init_idle][A1], ace.tim, 800);
        ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], init_pos[init_idle][J60], ace.tim, 800);
        ace.real_time_target_pos[M3508] = sin_target_curve(ace.init_pos[M3508], init_pos[init_idle][M3508], ace.tim, 800);
    }
    else
    {
        ace.set_flag = 0;
        ace.act_state = IDLE;
    }
}

void Init_Pick_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.set_flag = 1;
    }
    if (ace.tim < 800)
    {
        ace.tim++;
        G_ff.flag = 0;
        m3508_ctrl_flag = 1;
        ace.real_time_target_pos[DM] = linear_target_curve(ace.init_pos[DM], init_pos[init_idle][DM], ace.tim - 300, 500);
        ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], init_pos[init_idle][M2006], ace.tim - 300, 500);
        ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], init_pos[init_idle][A1], ace.tim, 800);
        ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], init_pos[init_idle][J60], ace.tim, 800);
        ace.real_time_target_pos[M3508] = sin_target_curve(ace.init_pos[M3508], get_weapon_pos[vertical][M3508], ace.tim, 800);
        if (ace.tim >= 600)
        {
            Clamp_Down();
            Clamp_Close();
        }
    }
    else
    {
        ace.set_flag = 0;
        ace.act_state = IDLE;
    }
}

void Get_Weapon_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.set_flag = 1;
    }
    if (ace.tim < 150)
    {
        ace.tim++;
        ace.real_time_target_pos[M3508] = linear_target_curve(ace.init_pos[M3508], get_weapon_pos[horizontal][M3508], ace.tim, 50);
    }
    else if (ace.tim >= 150 && ace.tim < 900)
    {
        ace.tim++;
        Clamp_Close();
        ace.real_time_target_pos[M3508] = linear_target_curve(get_weapon_pos[horizontal][M3508], get_weapon_pos[vertical][M3508], ace.tim - 400, 500);
    }
    else
    {
        ace.set_flag = 0;
        if (airOperator.LimitSwitch[0]) // 夹头成功
        {
            ace.act_state = COMBINE_PREPARE;
        }
        else // 夹头失败
        {
            ace.cplt_state.weapon_cplt_flag = 2;
            ace.last_act_state = GET_WEAPON;
            ace.act_state = SILENT;
        }
    }
}

void Combine_Prepare_Task() // 夹爪气缸下降
{
    if (ace.set_flag == 0)
    {
        ace.tim = 0;
        ace.set_flag = 1;
    }
    if (ace.tim < 200)
    {
        ace.tim++;
        Clamp_Down();
    }
    else
    {
        if (airOperator.LimitSwitch[0])
        {
            ace.cplt_state.weapon_cplt_flag = 1;
            m3508_ctrl_flag = 2;
        }
        else
        {
            ace.cplt_state.weapon_cplt_flag = 3;
            ace.last_act_state = GET_WEAPON;
        }
        ace.set_flag = 0;
        ace.act_state = SILENT;
    }
}

void Combine_End_Task() // 立即张开夹爪
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.set_flag = 1;
        m3508_ctrl_flag = 1;
    }
    if (ace.tim < 300)
    {
        ace.tim++;
        Clamp_Open();
        m3508_ctrl_flag = 0;
    }
    else
    {
        ace.set_flag = 0;
        ace.act_state = IDLE;
    }
}

void Re_Get_Weapon_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.set_flag = 1;
    }
    if (ace.tim < 300)
    {
        ace.tim++;
        Clamp_Open();
        Clamp_Up();
        m3508_ctrl_flag = 1;
        ace.real_time_target_pos[M3508] = sin_target_curve(ace.init_pos[M3508], init_pos[init_idle][M3508], ace.tim, 300);
    }
    else
    {
        ace.set_flag = 0;
        ace.act_state = IDLE;
    }
}

void Collect_Prepare_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        if (flag_tic_tac_toe == 0)
        {
            ace.cplt_state.prepare_kfs_cplt_flag = 1;
        }
        ace.set_flag = 1;
    }
    if (ace.tim < 800)
    {
        if ((ace.kfs.next_action == 0 || ace.kfs.kfs_height == 0 || ace.kfs.kfs_orientation == 0 || communicate_flag == 0) && ace.tim >= 10)
        {
            ace.tim = 0;
        }
        else
        {
            ace.tim++;
        }
        if (ace.tim >= 15)
        {
            kfs_param_flag = 0;
        }
        m3508_ctrl_flag = 1;
        ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], get_kfs_pos[ace.kfs.kfs_height][ace.kfs.kfs_orientation][DM] + get_kfs_pos[ace.kfs.kfs_height][constant][DM], ace.tim - 100, 700);
        ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], get_kfs_pos[ace.kfs.kfs_height][ace.kfs.kfs_orientation][M2006] + get_kfs_pos[ace.kfs.kfs_height][constant][M2006], ace.tim, 800);
        ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], get_kfs_pos[ace.kfs.kfs_height][ace.kfs.kfs_orientation][A1] + get_kfs_pos[ace.kfs.kfs_height][constant][A1], ace.tim - 100, 700);
        ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], get_kfs_pos[ace.kfs.kfs_height][ace.kfs.kfs_orientation][J60] + get_kfs_pos[ace.kfs.kfs_height][constant][J60], ace.tim, 100);
        ace.real_time_target_pos[M3508] = sin_target_curve(ace.init_pos[M3508], get_kfs_pos[0][0][M3508], ace.tim, 800);
    }
    else
    {
        ace.set_flag = 0;
        m3508_ctrl_flag = 0;
        if (flag_tic_tac_toe == 0)
        {
            ace.cplt_state.prepare_kfs_cplt_flag = 2;
            ace.act_state = IDLE;
        }
        else if (flag_tic_tac_toe == 1)
        {
            ace.act_state = COLLECT_KFS;
        }
    }
}

void Collect_KFS_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.cplt_state.collect_kfs_cplt_flag = 1;
        ace.set_flag = 1;
    }
    if (ace.tim < 800)
    {
        ace.tim++;
        Collect_KFS_Pull();
        ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], get_kfs_pos[ace.kfs.kfs_height][ace.kfs.kfs_orientation][M2006], ace.tim, 450);
        ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], get_kfs_pos[ace.kfs.kfs_height][ace.kfs.kfs_orientation][A1], ace.tim, 450);
    }
    else 
    {
        ace.set_flag = 0;
        if (flag_tic_tac_toe == 0)
        {
            if (ace.kfs.next_action == 1 || ace.kfs.next_action == 2)
            {
                
                ace.act_state = PULL_KFS_TEMPORARY;
            }
            else if (ace.kfs.next_action == 3)
            {
                ace.act_state = PULL_KFS_HOLD;
            }
        }
        else if (flag_tic_tac_toe == 1)
        {
            ace.act_state = PULL_KFS_TEMPORARY;
        }
    }
}

void Pull_KFS_Temporary_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.cplt_state.collect_kfs_cplt_flag = 1;
        kfs_param_flag = 1;
        ace.set_flag = 1;
        G_ff.flag = 1;
    }
    else if(ace.tim < 1100)
    {
        ace.tim++;
        if (ace.tim >= 300)
        {
            ace.cplt_state.collect_kfs_cplt_flag = 3;
        }
        ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], init_pos[init_temporary][DM], ace.tim, 900);
        ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], init_pos[init_temporary][M2006], ace.tim, 700);
        ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], init_pos[init_temporary][A1], ace.tim, 1100);
        ace.real_time_target_pos[J60] = linear_target_curve(ace.init_pos[J60], init_pos[init_temporary][J60], ace.tim - 750, 350);
    }
    else
    {
        if (flag_tic_tac_toe == 0)
        {
            if (ace.kfs.next_action == 1)
            {
                ace.set_flag = 0;
                ace.act_state = THROW_KFS;
            }
            else if (ace.kfs.next_action == 2)
            {
                ace.set_flag = 0;
                ace.act_state = STORE_KFS;
            }
        }
        else if (flag_tic_tac_toe == 1)
        {
            ace.set_flag = 0;
            ace.cplt_state.collect_kfs_cplt_flag = 2;
            ace.act_state = SILENT;
        }
    }
}

void Pull_KFS_Hold_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.cplt_state.collect_kfs_cplt_flag = 1;
        kfs_param_flag = 1;
        ace.set_flag = 1;
        G_ff.flag = 1;
    }
    else if (ace.tim < 1100)
    {
        ace.tim++;
        if (ace.tim >= 300)
        {
            ace.cplt_state.collect_kfs_cplt_flag = 3;
        }
        if (ace.tim < 600)
        {
            ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], init_pos[init_idle][DM] - 0.35, ace.tim, 600);
            ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], init_pos[init_idle][M2006] + 270, ace.tim, 500);
        }
        else
        {
            ace.real_time_target_pos[DM] = sin_target_curve(init_pos[init_idle][DM] - 0.35, init_pos[init_idle][DM], ace.tim - 900, 200);
            ace.real_time_target_pos[M2006] = sin_target_curve(init_pos[init_idle][M2006] + 270, init_pos[init_idle][M2006], ace.tim - 800, 300);
        }
        ace.real_time_target_pos[A1] = sin_target_curve(get_kfs_pos[ace.kfs.kfs_height][ace.kfs.kfs_orientation][A1], init_pos[init_idle][A1], ace.tim, 1100);
        ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], init_pos[init_idle][J60], ace.tim - 725, 375);
    }
    else
    {
        ace.set_flag = 0;
        ace.cplt_state.collect_kfs_cplt_flag = 2;
        ace.act_state = SILENT;
    }
}

void Throw_KFS_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.set_flag = 1;
        G_ff.flag = 1;
    }
    if (ace.tim < 600)
    {
        ace.tim++;
        ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], throw_kfs_pos[DM], ace.tim, 600);
        ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], throw_kfs_pos[M2006], ace.tim, 600);
        ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], throw_kfs_pos[A1], ace.tim, 600);
        ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], throw_kfs_pos[J60], ace.tim, 600);
    }
    else if (ace.tim >= 600 && ace.tim < 750)
    {
        ace.tim++;
        Collect_KFS_Push();
    }
    else if (ace.tim >= 750 && ace.tim < 1100)
    {
        ace.tim++;
        G_ff.flag = 0;
        Collect_KFS_IDLE();
        ace.real_time_target_pos[DM] = sin_target_curve(throw_kfs_pos[DM], init_pos[init_idle][DM], ace.tim - 750, 350);
        ace.real_time_target_pos[M2006] = sin_target_curve(throw_kfs_pos[M2006], init_pos[init_idle][M2006], ace.tim - 750, 350);
        ace.real_time_target_pos[A1] = sin_target_curve(throw_kfs_pos[A1], init_pos[init_idle][A1], ace.tim - 750, 350);
        ace.real_time_target_pos[J60] = sin_target_curve(throw_kfs_pos[J60], init_pos[init_idle][J60], ace.tim - 750, 350);
    }
    else if (ace.tim >= 1100)
    {
        ace.set_flag = 0;
        ace.cplt_state.collect_kfs_cplt_flag = 2;
        ace.act_state = SILENT;
    }
}

void Store_KFS_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.set_flag = 1;
        G_ff.flag = 1;
    }
    if (ace.tim < 450)
    {
        ace.tim++;
        ace.real_time_target_pos[DM] = linear_target_curve(ace.init_pos[DM], store_kfs_pos[DM] - 0.05, ace.tim, 450);
        ace.real_time_target_pos[M2006] = linear_target_curve(ace.init_pos[M2006], store_kfs_pos[M2006] + 320, ace.tim, 350);
        ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], store_kfs_pos[A1], ace.tim, 450);
        ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], store_kfs_pos[J60], ace.tim, 150);
    }
    else if (ace.tim >= 450 && ace.tim < 1050)
    {
        ace.tim++;
        Store_KFS_Pull();
        ace.real_time_target_pos[DM] = linear_target_curve(store_kfs_pos[DM] - 0.05, store_kfs_pos[DM] + 0.15, ace.tim - 450, 250);
        ace.real_time_target_pos[M2006] = linear_target_curve(store_kfs_pos[M2006] + 320, store_kfs_pos[M2006], ace.tim - 450, 450);
    }
    else if (ace.tim >= 1050 && ace.tim < 1150)
    {
        ace.tim++;
        Collect_KFS_Push();
    }
    else if (ace.tim >= 1150 && ace.tim < 1450)
    {
        ace.tim++;
        G_ff.flag = 0;
        Collect_KFS_IDLE();
        ace.real_time_target_pos[DM] = sin_target_curve(store_kfs_pos[DM] + 0.15, init_pos[init_idle][DM], ace.tim - 1150, 300);
        ace.real_time_target_pos[M2006] = sin_target_curve(store_kfs_pos[M2006], init_pos[init_idle][M2006], ace.tim - 1150, 300);
        ace.real_time_target_pos[A1] = sin_target_curve(store_kfs_pos[A1], init_pos[init_idle][A1], ace.tim - 1150, 300);
        ace.real_time_target_pos[J60] = sin_target_curve(store_kfs_pos[J60], init_pos[init_idle][J60], ace.tim - 1150, 300);
    }
    else if (ace.tim >= 1450)
    {
        ace.set_flag = 0;
        ace.cplt_state.collect_kfs_cplt_flag = 2;
        ace.act_state = SILENT;
    }
}

void Place_Mid_KFS_Task_Near()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.set_flag = 1;
        ace.cplt_state.place_kfs_cplt_flag = 1;
        G_ff.flag = 1;
    }
    if (ace.tim < 1050)
    {
        ace.tim++;
        ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], place_mid_kfs_pos_near[DM], ace.tim, 600);
        ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], place_mid_kfs_pos_near[M2006], ace.tim - 400, 400);
        ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], place_mid_kfs_pos_near[A1], ace.tim - 400, 400);
    }
    else if (ace.tim >= 1050 && ace.tim < 1250)
    {
        ace.tim++;
        Collect_KFS_Push();
    }
    else if (ace.tim >= 1250 && ace.tim < 1800)
    {
        ace.tim++;
        G_ff.flag = 0;
        Collect_KFS_IDLE();
        ace.real_time_target_pos[DM] = sin_target_curve(place_mid_kfs_pos_near[DM], init_pos[init_temporary][DM], ace.tim - 1400, 400);
        ace.real_time_target_pos[M2006] = sin_target_curve(place_mid_kfs_pos_near[M2006], init_pos[init_temporary][M2006], ace.tim - 1550, 250);
        ace.real_time_target_pos[A1] = sin_target_curve(place_mid_kfs_pos_near[A1], init_pos[init_temporary][A1], ace.tim - 1400, 400);
    }
    else
    {
        ace.set_flag = 0;
        ace.cplt_state.place_kfs_cplt_flag = 2;
        ace.act_state = SILENT;
    }
}

void Place_Mid_KFS_Task_Far()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.set_flag = 1;
        ace.cplt_state.place_kfs_cplt_flag = 1;
        G_ff.flag = 1;
    }
    if (ace.tim < 900)
    {
        ace.tim++;
        ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], place_mid_kfs_pos_far[DM], ace.tim - 200, 600);
        ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], place_mid_kfs_pos_far[M2006], ace.tim - 400, 450);
        ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], place_mid_kfs_pos_far[A1], ace.tim, 450);
    }
    else if (ace.tim >= 900 && ace.tim < 1075)
    {
        ace.tim++;
        Collect_KFS_Push();
    }
    else if (ace.tim >= 1075 && ace.tim < 1600)
    {
        ace.tim++;
        G_ff.flag = 0;
        Collect_KFS_IDLE();
        ace.real_time_target_pos[DM] = sin_target_curve(place_mid_kfs_pos_far[DM], init_pos[init_temporary][DM], ace.tim - 1100, 500);
        ace.real_time_target_pos[M2006] = sin_target_curve(place_mid_kfs_pos_far[M2006], init_pos[init_temporary][M2006], ace.tim - 1200, 400);
        ace.real_time_target_pos[A1] = sin_target_curve(place_mid_kfs_pos_far[A1], init_pos[init_temporary][A1], ace.tim - 1100, 500);
    }
    else
    {
        ace.set_flag = 0;
        ace.cplt_state.place_kfs_cplt_flag = 2;
        ace.act_state = SILENT;
    }
}

void Get_KFS_Behind_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.set_flag = 1;
        ace.cplt_state.get_kfs_cplt_flag = 1;
    }
    if (ace.tim < 400)
    {
        ace.tim++;
        ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], init_pos[init_temporary][DM], ace.tim, 400);
        ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], init_pos[init_temporary][M2006], ace.tim, 400);
        ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], init_pos[init_temporary][A1], ace.tim, 400);
        ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], init_pos[init_temporary][J60], ace.tim - 200, 200);
    }
    else if (ace.tim >= 400 && ace.tim < 700)
    {
        ace.tim++;
        ace.real_time_target_pos[DM] = sin_target_curve(init_pos[init_temporary][DM], get_kfs_behind_pos[DM], ace.tim - 400, 300);
        ace.real_time_target_pos[M2006] = sin_target_curve(init_pos[init_temporary][M2006], get_kfs_behind_pos[M2006], ace.tim - 400, 300);
        ace.real_time_target_pos[A1] = sin_target_curve(init_pos[init_temporary][A1], get_kfs_behind_pos[A1] + 0.25, ace.tim - 400, 300);
    }
    else if (ace.tim >= 700 && ace.tim < 1200)
    {
        ace.tim++;
        Collect_KFS_Pull();
        ace.real_time_target_pos[A1] = sin_target_curve(get_kfs_behind_pos[A1] + 0.25, get_kfs_behind_pos[A1], ace.tim - 700, 400);
    }
    else if (ace.tim >= 1200 && ace.tim < 1250)
    {
        ace.tim++;
        Store_KFS_Push();
    }
    else if (ace.tim >= 1250 && ace.tim < 1800)
    {
        ace.tim++;
        if (ace.tim < 1325)
        {
            Store_KFS_Push();
            ace.real_time_target_pos[M2006] = sin_target_curve(get_kfs_behind_pos[M2006], get_kfs_behind_pos[M2006] + 150, ace.tim - 1250, 75);
        }
        else
        {
            Store_KFS_IDLE();
            ace.real_time_target_pos[M2006] = sin_target_curve(get_kfs_behind_pos[M2006] + 100, init_pos[init_temporary][M2006], ace.tim - 1400, 300);
        }
        G_ff.flag = 1;
        ace.real_time_target_pos[DM] = linear_target_curve(get_kfs_behind_pos[DM], init_pos[init_temporary][DM], ace.tim - 1250, 550);
        ace.real_time_target_pos[A1] = linear_target_curve(get_kfs_behind_pos[A1], init_pos[init_temporary][A1] - 0.03f, ace.tim - 1250, 550);
    }
    else
    {
        ace.set_flag = 0;
        ace.cplt_state.get_kfs_cplt_flag = 2;
        ace.act_state = SILENT;
    }
}

void Get_KFS_From_R1_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.cplt_state.get_kfs_cplt_flag = 1;
        ace.set_flag = 1;
        r1_orient_flag = 0;
    }
    if (ace.tim < 1100)
    {
        ace.tim++;
        Collect_KFS_Pull();
        if (ace.r1_orient == 0)
        {
            ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], get_kfs_from_r1_pos_left[DM], ace.tim - 350, 750);
            ace.real_time_target_pos[M2006] = linear_target_curve(ace.init_pos[M2006], get_kfs_from_r1_pos_left[M2006] - 400, ace.tim - 700, 400);
            ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], get_kfs_from_r1_pos_left[A1], ace.tim - 350, 750);
            ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], get_kfs_from_r1_pos_left[J60], ace.tim, 350);
        }
        else if (ace.r1_orient == 1)
        {
            ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], get_kfs_from_r1_pos_right[DM], ace.tim - 350, 750);
            ace.real_time_target_pos[M2006] = linear_target_curve(ace.init_pos[M2006], get_kfs_from_r1_pos_right[M2006] - 400, ace.tim - 700, 400);
            ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], get_kfs_from_r1_pos_right[A1], ace.tim - 350, 750);
            ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], get_kfs_from_r1_pos_right[J60], ace.tim, 350);
        }
    }
    else if (ace.tim >= 1100 && ace.tim < 1400)
    {
        ace.tim++;
        if (ace.r1_orient == 0)
        {
            ace.real_time_target_pos[M2006] = linear_target_curve(get_kfs_from_r1_pos_left[M2006] - 400, get_kfs_from_r1_pos_left[M2006], ace.tim - 1100, 300);
        }
        else if (ace.r1_orient == 1)
        {
            ace.real_time_target_pos[M2006] = linear_target_curve(get_kfs_from_r1_pos_right[M2006] - 400, get_kfs_from_r1_pos_right[M2006], ace.tim - 1100, 300);
        }
    }
    else if (ace.tim >= 1400)
    {
        ace.set_flag = 0;
        ace.act_state = IDLE;
        r1_orient_flag = 1;
    }
}

void Get_KFS_From_R1_Back_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.cplt_state.get_kfs_cplt_flag = 1;
        ace.set_flag = 1;
    }
    if (ace.tim < 1500)
    {
        ace.tim++;
        G_ff.flag = 1;
        ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], init_pos[init_temporary][DM], ace.tim, 400);
        ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], init_pos[init_temporary][M2006], ace.tim, 400);
        ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], init_pos[init_temporary][A1], ace.tim, 1200);
        ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], init_pos[init_temporary][J60], ace.tim - 1200, 300);
    }
    else
    {
        ace.set_flag = 0;
        ace.cplt_state.get_kfs_cplt_flag = 2;
        ace.act_state = SILENT;
    }
}

void Place_Top_KFS_Task_Near()
{
    // if (ace.set_flag == 0)
    // {
    //     Get_Init_Pos();
    //     ace.tim = 0;
    //     ace.set_flag = 1;
    //     G_ff.flag = 1;
    //     ace.cplt_state.place_kfs_cplt_flag = 1;
    // }
    // if (ace.tim < 700)
    // {
    //     ace.tim++;
    //     ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], place_top_kfs_pos_near[0][DM], ace.tim - 250, 450);
    //     ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], place_top_kfs_pos_near[0][M2006], ace.tim, 450);
    //     ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], place_top_kfs_pos_near[0][A1], ace.tim, 700);
    //     ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], place_top_kfs_pos_near[0][J60], ace.tim, 700);
    // }
    // else if (ace.tim >= 700 && ace.tim < 1300)
    // {
    //     ace.tim++;
    //     ace.real_time_target_pos[DM] = sin_target_curve(place_top_kfs_pos_near[0][DM], place_top_kfs_pos_near[1][DM], ace.tim - 700, 400);
    //     ace.real_time_target_pos[M2006] = sin_target_curve(place_top_kfs_pos_near[0][M2006], place_top_kfs_pos_near[1][M2006], ace.tim - 950, 350);
    //     ace.real_time_target_pos[A1] = sin_target_curve(place_top_kfs_pos_near[0][A1], place_top_kfs_pos_near[1][A1], ace.tim - 700, 400);
    // }
    // else if (ace.tim >= 1300 && ace.tim < 1475)
    // {
    //     ace.tim++;
    //     Collect_KFS_Push();
    // }
    // else if (ace.tim >= 1475 && ace.tim < 2100)
    // {
    //     ace.tim++;
    //     G_ff.flag = 0;
    //     Collect_KFS_IDLE();
    //     ace.real_time_target_pos[DM] = sin_target_curve(place_top_kfs_pos_near[1][DM], init_pos[init_temporary][DM], ace.tim - 1600, 500);
    //     ace.real_time_target_pos[M2006] = sin_target_curve(place_top_kfs_pos_near[1][M2006], init_pos[init_temporary][M2006], ace.tim - 1600, 500);
    //     ace.real_time_target_pos[A1] = sin_target_curve(place_top_kfs_pos_near[1][A1], init_pos[init_temporary][A1], ace.tim - 1600, 500);
    // }
    // else
    // {
    //     ace.set_flag = 0;
    //     ace.cplt_state.place_kfs_cplt_flag = 2;
    //     ace.act_state = SILENT;
    // }
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.set_flag = 1;
        G_ff.flag = 1;
        ace.cplt_state.place_kfs_cplt_flag = 1;
    }
    if (ace.tim < 900)
    {
        ace.tim++;
        ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], place_top_kfs_pos_far[1][DM], ace.tim, 600);
        ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], place_top_kfs_pos_far[1][M2006], ace.tim - 200, 600);
        ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], place_top_kfs_pos_far[1][A1], ace.tim - 400, 400);
    }
    else if (ace.tim >= 900 && ace.tim < 1100)
    {
        ace.tim++;
        Collect_KFS_Push();
    }
    else if (ace.tim >= 1100 && ace.tim < 1700)
    {
        ace.tim++;
        G_ff.flag = 0;
        Collect_KFS_IDLE();
        ace.real_time_target_pos[DM] = sin_target_curve(place_top_kfs_pos_far[1][DM], init_pos[init_temporary][DM], ace.tim - 1200, 500);
        ace.real_time_target_pos[M2006] = sin_target_curve(place_top_kfs_pos_far[1][M2006], init_pos[init_temporary][M2006], ace.tim - 1300, 400);
        ace.real_time_target_pos[A1] = sin_target_curve(place_top_kfs_pos_far[1][A1], init_pos[init_temporary][A1], ace.tim - 1200, 500);
    }
    else
    {
        ace.set_flag = 0;
        ace.cplt_state.place_kfs_cplt_flag = 2;
        ace.act_state = SILENT;
    }
}

void Place_Top_KFS_Task_Far()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.set_flag = 1;
        G_ff.flag = 1;
        ace.cplt_state.place_kfs_cplt_flag = 1;
    }
    if (ace.tim < 900)
    {
        ace.tim++;
        ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], place_top_kfs_pos_far[1][DM], ace.tim, 600);
        ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], place_top_kfs_pos_far[1][M2006], ace.tim - 200, 600);
        ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], place_top_kfs_pos_far[1][A1], ace.tim - 400, 400);
    }
    else if (ace.tim >= 900 && ace.tim < 1100)
    {
        ace.tim++;
        Collect_KFS_Push();
    }
    else if (ace.tim >= 1100 && ace.tim < 1700)
    {
        ace.tim++;
        G_ff.flag = 0;
        Collect_KFS_IDLE();
        ace.real_time_target_pos[DM] = sin_target_curve(place_top_kfs_pos_far[1][DM], init_pos[init_temporary][DM], ace.tim - 1200, 500);
        ace.real_time_target_pos[M2006] = sin_target_curve(place_top_kfs_pos_far[1][M2006], init_pos[init_temporary][M2006], ace.tim - 1300, 400);
        ace.real_time_target_pos[A1] = sin_target_curve(place_top_kfs_pos_far[1][A1], init_pos[init_temporary][A1], ace.tim - 1200, 500);
    }
    else
    { 
        ace.set_flag = 0;
        ace.cplt_state.place_kfs_cplt_flag = 2;
        ace.act_state = SILENT;
    }
}

void Init_For_3_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.set_flag = 1;
        ace.tim = 0;
    }
    if (ace.tim < 800)
    {
        ace.tim++;
        ace.cplt_state.weapon_cplt_flag = 0;
        ace.cplt_state.collect_kfs_cplt_flag = 0;
        ace.cplt_state.place_kfs_cplt_flag = 0;
        ace.cplt_state.get_kfs_cplt_flag = 0;
        ace.cplt_state.prepare_kfs_cplt_flag = 0;
        ace.cplt_state.transfer_cplt_flag = 0;
        Collect_KFS_Pull();
        Store_KFS_Pull();
        m3508_ctrl_flag = 0;
        ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], init_pos[init_idle][DM], ace.tim - 300, 500);
        ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], init_pos[init_idle][M2006], ace.tim - 300, 500);
        ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], init_pos[init_idle][A1], ace.tim, 800);
        ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], init_pos[init_idle][J60], ace.tim, 800);
    }
    else
    {
        ace.set_flag = 0;
        ace.act_state = IDLE;
        G_ff.flag = 1;
    }
}

void Recover_Task()//
{
    switch (ace.last_act_state)
    {
    case GET_WEAPON: // 夹头失败后进入，将夹爪打开，回到水平位置，准备下一次GET_WEAPON
        if (ace.set_flag == 0)
        {
            Get_Init_Pos();
            ace.tim = 0;
            ace.set_flag = 1;
        }
        if (ace.tim < 500)
        {
            ace.tim++;
            Clamp_Open();
            Clamp_Up();
            ace.real_time_target_pos[M3508] = sin_target_curve(ace.init_pos[M3508], init_pos[init_idle][M3508], ace.tim - 300, 200);
        }
        else
        {
            ace.set_flag = 0;
            ace.last_act_state = 0;
        }
        break;

    case TRANSFER_FOR_400:
        if (ace.set_flag == 0)
        {
            Get_Init_Pos();
            ace.set_flag = 1;
            ace.cplt_state.transfer_cplt_flag = 1;
            ace.tim = 0;
        }
        if (ace.tim < 500)
        {
            ace.tim++;
            ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], init_pos[init_idle][DM], ace.tim, 500);
            ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], init_pos[init_idle][M2006], ace.tim, 500);
            ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], init_pos[init_idle][A1], ace.tim, 500);
            ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], init_pos[init_idle][J60], ace.tim, 500);
        }
        else
        {
            ace.tim++;
            ace.last_act_state = 0;
            ace.set_flag = 0;
            ace.cplt_state.transfer_cplt_flag = 2;
        }
        break;

    default: // cplt_flag置0，相当于重置下板状态
        ace.set_flag = 0;
        ace.tim = 0;
        break;
    }
}

void Transfer_For_400_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.tim = 0;
        ace.cplt_state.transfer_cplt_flag = 1;
        ace.set_flag = 1;
    }
    if (ace.tim < 500)
    {
        ace.tim++;
        ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], init_pos[init_for_400][DM], ace.tim, 500);
        ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], init_pos[init_for_400][M2006], ace.tim, 500);
        ace.real_time_target_pos[A1] = linear_target_curve(ace.init_pos[A1], init_pos[init_for_400][A1], ace.tim, 500);
        ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], init_pos[init_for_400][J60], ace.tim, 500);
    }
    else
    {
        ace.set_flag = 0;
        ace.act_state = SILENT;
        ace.last_act_state = TRANSFER_FOR_400;
    }
}

void Transfer_For_Temporary_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.set_flag = 1;
        ace.tim = 0;
    }
    if (ace.tim < 500)
    {
        ace.tim++;
        ace.real_time_target_pos[DM] = sin_target_curve(ace.init_pos[DM], init_pos[init_temporary][DM], ace.tim, 500);
        ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], init_pos[init_temporary][M2006], ace.tim, 500);
        ace.real_time_target_pos[A1] = sin_target_curve(ace.init_pos[A1], init_pos[init_temporary][A1], ace.tim, 500);
        ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], init_pos[init_temporary][J60], ace.tim, 500);
    }
    else
    {
        ace.set_flag = 0;
        ace.act_state = IDLE;
    }
}

void Transfer_For_Top_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.set_flag = 1;
        ace.tim = 0;
        r1_orient_flag = 0;
    }
    if (ace.tim < 850)
    {
        ace.tim++;
        if (ace.r1_orient == 0)
        {
            ace.real_time_target_pos[DM] = linear_target_curve(ace.init_pos[DM], init_for_400_left[DM], ace.tim, 800);
            ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], init_for_400_left[M2006], ace.tim, 800);
            ace.real_time_target_pos[A1] = linear_target_curve(ace.init_pos[A1], init_for_400_left[A1], ace.tim - 275, 575);
            ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], init_for_400_left[J60], ace.tim, 300);
        }
        else if (ace.r1_orient == 1)
        {
            ace.real_time_target_pos[DM] = linear_target_curve(ace.init_pos[DM], init_for_400_right[DM], ace.tim, 800);
            ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], init_for_400_right[M2006], ace.tim, 800);
            ace.real_time_target_pos[A1] = linear_target_curve(ace.init_pos[A1], init_for_400_right[A1], ace.tim - 275, 575);
            ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], init_for_400_right[J60], ace.tim, 300);
        }
    }
    else
    {
        ace.set_flag = 0;
        r1_orient_flag = 1;
        flag_stand = 1;
        ace.act_state = IDLE;
    }
}

void Transfer_From_Top_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.set_flag = 1;
        ace.tim = 0;
    }
    if (ace.tim < 650)
    {
        ace.tim++;
        ace.real_time_target_pos[DM] = linear_target_curve(ace.init_pos[DM], init_pos[init_temporary][DM], ace.tim, 500);
        ace.real_time_target_pos[M2006] = sin_target_curve(ace.init_pos[M2006], init_pos[init_temporary][M2006], ace.tim, 500);
        ace.real_time_target_pos[A1] = linear_target_curve(ace.init_pos[A1], init_pos[init_temporary][A1], ace.tim, 500);
        ace.real_time_target_pos[J60] = sin_target_curve(ace.init_pos[J60], init_pos[init_temporary][J60], ace.tim - 400, 250);
    }
    else
    {
        ace.set_flag = 0;
        if (index_stand == 0)
        {
            ace.cplt_state.transfer_cplt_flag = 2;
            ace.act_state = SILENT;
        }
        else if (index_stand == 1)
        {
            index_stand = 0;
            ace.act_state = GET_KFS_BEHIND;
        }
        else if (index_stand == 2)
        {
            index_stand = 0;
            ace.act_state = PLACE_TOP_KFS_NEAR;
        }
        else if (index_stand == 3)
        {
            index_stand = 0;
            ace.act_state = PLACE_TOP_KFS_FAR;
        }
    }
}

void Trial_Task()
{
    if (ace.set_flag == 0)
    {
        Get_Init_Pos();
        ace.set_flag = 1;
        ace.tim = 0;
    }
    if (ace.tim < 1500)
    {
        ace.tim++;
        mode = 1;
        ace.real_time_target_pos[DM] = linear_target_curve(ace.init_pos[DM], init_pos[init_temporary][DM], ace.tim, 1200);
        ace.real_time_target_pos[M2006] = linear_target_curve(ace.init_pos[M2006], init_pos[init_temporary][M2006], ace.tim, 1200);
        ace.real_time_target_pos[A1] = linear_target_curve(ace.init_pos[A1], init_pos[init_temporary][A1], ace.tim, 1200);
        ace.real_time_target_pos[J60] = linear_target_curve(ace.init_pos[J60], get_kfs_pos[0][flag_trial_orient][J60], ace.tim - 1100, 400);
    }
    else
    {
        mode = 10;
        flag_trial_orient = 0;
        ace.set_flag = 0;
        ace.act_state = IDLE;
    }
}

void All_Push_Task()
{
    if (ace.set_flag == 0)
    {
        ace.set_flag = 1;
        ace.tim = 0;
    }
    if (ace.tim < 10)
    {
        ace.tim++;
        Store_KFS_IDLE();
        Collect_KFS_IDLE();
    }
    else if (ace.tim >= 10 && ace.tim < 190)
    {
        ace.tim++;
        Collect_KFS_Push();
        Store_KFS_Push();
    }
    else if (ace.tim >= 190 && ace.tim < 200)
    {
        ace.tim++;
        Store_KFS_IDLE();
        Collect_KFS_IDLE();
    }
    else
    {
        ace.set_flag = 0;
        ace.act_state = IDLE;
    }
}

void Get_Init_Pos() // 第一次进入任务时，记录电机反馈位置，作为后续动作的初始位置
{
    ace.init_pos[DM] = obs.dm_pos;
    ace.init_pos[M2006] = obs.m2006_pos;
    ace.init_pos[A1] = obs.a1_pos;
    ace.init_pos[J60] = obs.j60_pos;
    ace.init_pos[M3508] = obs.m3508_pos;
}
