#include "StateMachine.h"
#include "Robot.h"
#include "MathAlgorithm.h"

// 夹爪：0 张开， 1 闭合
// 夹爪上下位移：
// 气泵：1 工作， 0 停止
// 伸缩臂电磁阀：0 吸气， 1 吐气

void myStateMachine(void)
{
    if (actionCpltFlag == 0)
    {
        switch (actionFlag)
        {
        case INIT_ACTION:
            switch (stateFlag)
            {
            case STAND_BY:
                sm_cnt++;
                if (sm_cnt < 1000)
                {
                    if (init_mode == 0)
                    {
                        if (sm_cnt < 500)
                        {
                            airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                        }
                        else
                        {
                            airOperator.airOperatorTxBuf[0] = 1; // 夹爪
                        }
                        airOperator.airOperatorTxBuf[1] = 0; // 伸缩臂气泵
                        airOperator.airOperatorTxBuf[2] = 0; // 储存气泵
                        Gimbal_J60_CMD.position_ = STANDBY_J60 + (-STANDBY_J60 + READY_TO_GET_STRAIGHT_KFS_J60) * sm_cnt / 1000;
                        Motor_A1.TargetPos = STANDBY_A1 + (READY_TO_GET_KFS_A1 - STANDBY_A1) * sm_cnt / 1000;
                        stretch_2006.outerTarget = STANDBY_S2006 + (READY_TO_GET_KFS_S2006 - STANDBY_S2006) * sm_cnt / 1000;
                        stretch_DM.ctrl.pos_set = STANDBY_DM + (READY_TO_GET_KFS_DM - STANDBY_DM) * sm_cnt / 1000;
                        //											Gimbal_J60_CMD.position_ = READY_TO_GET_STRAIGHT_KFS_J60+(STANDBY_J60-READY_TO_GET_STRAIGHT_KFS_J60)*sm_cnt/1000;
                        //											Motor_A1.TargetPos = STANDBY_A1+(GET_KFS_III_A1-STANDBY_A1)*sm_cnt/1000;
                        //											stretch_2006.outerTarget = STANDBY_S2006+(GET_KFS_III_S2006-STANDBY_S2006)*sm_cnt/1000;
                        //											stretch_DM.ctrl.pos_set = STANDBY_DM+(GET_KFS_III_DM-STANDBY_DM)*sm_cnt/1000;
                        // move_2006.outerTarget = STANDBY_M2006 + (GET_WEAPON_HEAD_M2006 - STANDBY_M2006) * sm_cnt / 1000;
                        DJI_3508.outerTarget = STANDBY_3508 + (GET_WEAPON_HEAD_3508 - STANDBY_3508) * sm_cnt / 1000;
                    }
                    else if (init_mode == 1)
                    {
                        if (sm_cnt < 500)
                        {
                            airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                        }
                        else
                        {
                            airOperator.airOperatorTxBuf[0] = 1; // 夹爪
                        }
                        airOperator.airOperatorTxBuf[1] = 0; // 伸缩臂气泵
                        airOperator.airOperatorTxBuf[2] = 0; // 储存气泵
                        Gimbal_J60_CMD.position_ = STANDBY_J60 + (-STANDBY_J60 + READY_TO_GET_STRAIGHT_KFS_J60) * sm_cnt / 1000;
                        Motor_A1.TargetPos = STANDBY_A1 + (HOLD_KFS_A1 - STANDBY_A1) * sm_cnt / 1000;
                        stretch_2006.outerTarget = STANDBY_S2006 + (HOLD_KFS_S2006 - STANDBY_S2006) * sm_cnt / 1000;
                        stretch_DM.ctrl.pos_set = STANDBY_DM + (HOLD_KFS_DM - STANDBY_DM) * sm_cnt / 1000;
                        // move_2006.outerTarget = STANDBY_M2006;
                        DJI_3508.outerTarget = STANDBY_3508;
                    }
                    else if (init_mode == 2)
                    {
                        if (sm_cnt < 500)
                        {
                            airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                        }
                        else
                        {
                            airOperator.airOperatorTxBuf[0] = 1; // 夹爪
                        }
                        airOperator.airOperatorTxBuf[1] = 0; // 伸缩臂气泵
                        airOperator.airOperatorTxBuf[2] = 1; // 储存气泵
                        Gimbal_J60_CMD.position_ = STANDBY_J60 + (-STANDBY_J60 + READY_TO_GET_STRAIGHT_KFS_J60) * sm_cnt / 1000;
                        Motor_A1.TargetPos = STANDBY_A1 + (HOLD_KFS_A1 - STANDBY_A1) * sm_cnt / 1000;
                        stretch_2006.outerTarget = STANDBY_S2006 + (HOLD_KFS_S2006 - STANDBY_S2006) * sm_cnt / 1000;
                        stretch_DM.ctrl.pos_set = STANDBY_DM + (HOLD_KFS_DM - STANDBY_DM) * sm_cnt / 1000;
                        // move_2006.outerTarget = STANDBY_M2006;
                        DJI_3508.outerTarget = STANDBY_3508;
                    }
                }
                else
                {
                    actionCpltFlag = 1;
                    if (init_mode == 0)
                    {
                        stateFlag = INIT;
                    }
                    else
                    {
                        stateFlag = HOLDING_KFS;
                    }
                    sm_cnt = 0;
                }
                break;
            case WAIT_FOR_COMBINE:
                sm_cnt++;
                if (sm_cnt < 2000)
                {
                    if (sm_cnt < 1250)
                    {
                        airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                    }
                    else
                    {
                        airOperator.airOperatorTxBuf[0] = 1; // 夹爪
                    }
                    airOperator.airOperatorTxBuf[1] = 0; // 伸缩臂气泵
                    airOperator.airOperatorTxBuf[3] = 0; // 吸气
                    airOperator.airOperatorTxBuf[2] = 0; // 储存气泵
                    airOperator.airOperatorTxBuf[4] = 0; // 吸气
                    Gimbal_J60_CMD.position_ = READY_TO_GET_STRAIGHT_KFS_J60;
                    Motor_A1.TargetPos = READY_TO_GET_KFS_A1;
                    stretch_2006.outerTarget = READY_TO_GET_KFS_S2006;
                    stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM;
                    // move_2006.outerTarget = WAIT_FOR_COMBINE_M2006 + (GET_WEAPON_HEAD_M2006 - WAIT_FOR_COMBINE_M2006) * rampSignalFP(sm_cnt - 750, 1250);
                    DJI_3508.outerTarget = WAIT_FOR_COMBINE_3508 + (GET_WEAPON_HEAD_3508 - WAIT_FOR_COMBINE_3508) * rampSignalFP(sm_cnt, 750);
                }
                else
                {
                    actionCpltFlag = 1;
                    stateFlag = INIT;
                    sm_cnt = 0;
                }
                break;
            default:
                actionCpltFlag = 1;
                break;
            }
            break;
        case DEINIT:
            switch (stateFlag)
            {
            case INIT:
                sm_cnt++;
                if (sm_cnt < 1000)
                {
                    if (sm_cnt < 500)
                    {
                        airOperator.airOperatorTxBuf[0] = 1; // 夹爪
                    }
                    else
                    {
                        airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                    }
                    airOperator.airOperatorTxBuf[1] = 0; // 伸缩臂气泵
                    airOperator.airOperatorTxBuf[3] = 0; // 吸气
                    airOperator.airOperatorTxBuf[2] = 0; // 储存气泵
                    airOperator.airOperatorTxBuf[4] = 0; // 吸气
                    Gimbal_J60_CMD.position_ = STANDBY_J60 + (READY_TO_GET_STRAIGHT_KFS_J60 - STANDBY_J60) * sm_cnt / 1000;
                    Motor_A1.TargetPos = READY_TO_GET_KFS_A1 + (STANDBY_A1 - READY_TO_GET_KFS_A1) * sm_cnt / 1000;
                    stretch_2006.outerTarget = READY_TO_GET_KFS_S2006 + (STANDBY_S2006 - READY_TO_GET_KFS_S2006) * sm_cnt / 1000;
                    stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM + (STANDBY_DM - READY_TO_GET_KFS_DM) * sm_cnt / 1000;
                    // move_2006.outerTarget = GET_WEAPON_HEAD_M2006 + (STANDBY_M2006 - GET_WEAPON_HEAD_M2006) * sm_cnt / 1000;
                    DJI_3508.outerTarget = GET_WEAPON_HEAD_3508 + (STANDBY_3508 - GET_WEAPON_HEAD_3508) * sm_cnt / 1000;
                }
                else
                {
                    actionCpltFlag = 1;
                    stateFlag = STAND_BY;
                    sm_cnt = 0;
                }
                break;

            default:
                actionCpltFlag = 1;
                break;
            }
            break;
        case GET_WEAPON_HEAD_FOR_COMBINE:
            switch (stateFlag)
            {
            case INIT:

                if (sm_cnt < 1800)
                {
                    sm_cnt++;
                    airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                    airOperator.airOperatorTxBuf[1] = 0; // 伸缩臂气泵
                    airOperator.airOperatorTxBuf[2] = 0; // 储存气泵
                    airOperator.airOperatorTxBuf[3] = 0; // 吸气
                    airOperator.airOperatorTxBuf[4] = 0; // 吸气

                    Gimbal_J60_CMD.position_ = READY_TO_GET_STRAIGHT_KFS_J60;
                    Motor_A1.TargetPos = READY_TO_GET_KFS_A1;
                    stretch_2006.outerTarget = READY_TO_GET_KFS_S2006;
                    stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM;

                    //												if(sm_cnt>900){
                    //													if(airOperator.LimitSwitch[0]==0){
                    //														sm_cnt=4001;
                    //														noWeapon=1;
                    //													}
                    //												}

                    // move_2006.outerTarget = GET_WEAPON_HEAD_M2006 + (WAIT_FOR_COMBINE_M2006 - GET_WEAPON_HEAD_M2006) * rampSignalFP(sm_cnt - 900, 900);
                    //												move_2006.outerTarget = GET_WEAPON_HEAD_M2006;
                    DJI_3508.outerTarget = GET_WEAPON_HEAD_3508 + 90 * rampSignalFP(sm_cnt - 400, 500);
                    if (sm_cnt > 600)
                    {
                        chassisMoveFlag = 1;
                    }
                }
                else if (sm_cnt >= 1800 && sm_cnt < 2400)
                {
                    sm_cnt++;
                    DJI_3508.outerTarget = GET_WEAPON_HEAD_3508 + 90 + (WAIT_FOR_COMBINE_3508 - GET_WEAPON_HEAD_3508 - 90) * rampSignalFP(sm_cnt - 1800, 600);
                    //												 DJI_3508.outerTarget = GET_WEAPON_HEAD_3508+90-90*rampSignalFP(sm_cnt-1800,600);
                }
                else if (sm_cnt >= 2400 && sm_cnt < 3000)
                {
                    actionCpltFlag = 1;
                    chassisMoveFlag = 0;
                    stateFlag = WAIT_FOR_COMBINE;
                    noWeapon = 0;
                    sm_cnt = 0;
                }
                //											else if(sm_cnt>4000){
                //												airOperator.airOperatorTxBuf[0]=1;//夹爪
                //												move_2006.outerTarget = GET_WEAPON_HEAD_M2006;
                //												DJI_3508.outerTarget = GET_WEAPON_HEAD_3508;
                //												if(completeActionFlag==1){
                //													sm_cnt=0;
                //												}
                //										}
                break;
            default:
                actionCpltFlag = 1;
                break;
            }
            break;
        case ENDING_COMBINE:
            switch (stateFlag)
            {
            case WAIT_FOR_COMBINE:
                sm_cnt++;
                if (sm_cnt <= 3000)
                {
                    if (sm_cnt < 1200)
                    {
                        airOperator.airOperatorTxBuf[0] = 1; // 夹爪
                    }
                    else if (sm_cnt >= 1200 && sm_cnt < 1800)
                    {
                        airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                    }
                    else if (sm_cnt >= 1800)
                    {
                        airOperator.airOperatorTxBuf[0] = 1; // 夹爪
                    }
                    airOperator.airOperatorTxBuf[1] = 0; // 伸缩臂气泵
                    airOperator.airOperatorTxBuf[2] = 0; // 储存气泵
                    airOperator.airOperatorTxBuf[3] = 0; // 吸气
                    airOperator.airOperatorTxBuf[4] = 0; // 吸气

                    Gimbal_J60_CMD.position_ = READY_TO_GET_STRAIGHT_KFS_J60;
                    Motor_A1.TargetPos = READY_TO_GET_KFS_A1;
                    stretch_2006.outerTarget = READY_TO_GET_KFS_S2006;
                    stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM;

                    // move_2006.outerTarget = WAIT_FOR_COMBINE_M2006 + (GET_WEAPON_HEAD_M2006 - WAIT_FOR_COMBINE_M2006) * rampSignalFP(sm_cnt - 1800, 1200);
                    DJI_3508.outerTarget = WAIT_FOR_COMBINE_3508 + (GET_WEAPON_HEAD_3508 - WAIT_FOR_COMBINE_3508) * rampSignalFP(sm_cnt - 1200, 1800);
                }
                else
                {
                    actionCpltFlag = 1;
                    stateFlag = INIT;
                    sm_cnt = 0;
                }
                break;
            default:
                actionCpltFlag = 1;
                break;
            }
            break;
        case GET_KFS:
            switch (stateFlag)
            {
            case INIT:
                if (sm_cnt <= 1000)
                {
                    sm_cnt++;
                    //												chassisMoveFlag=1;
                    airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                    airOperator.airOperatorTxBuf[1] = 1; // 伸缩臂气泵
                    airOperator.airOperatorTxBuf[3] = 0; // 吸气
                    //                        move_2006.outerTarget = STANDBY_M2006;
                    //                        DJI_3508.outerTarget = STANDBY_3508;
                    if (KFS_height == 0 && KFS_orientation == 0) // lower straight
                    {
                        Gimbal_J60_CMD.position_ = GET_KFS_STRAIGHT_J60;
                        Motor_A1.TargetPos = READY_TO_GET_KFS_A1 + (GET_KFS_STRAIGHT_LOW_A1 + 0.2f - READY_TO_GET_KFS_A1) * rampSignalFP(sm_cnt, 600);
                        stretch_2006.outerTarget = READY_TO_GET_KFS_S2006 + (GET_KFS_STRAIGHT_LOW_S2006 - READY_TO_GET_KFS_S2006) * rampSignalFP(sm_cnt, 600);
                        stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM + (GET_KFS_STRAIGHT_LOW_DM - READY_TO_GET_KFS_DM) * rampSignalFP(sm_cnt, 600);
                        if (sm_cnt > 600)
                        {
                            sm_cnt = 1001;
                        }
                    }
                    else if (KFS_height == 0 && KFS_orientation == 1) // lower left
                    {
                        Gimbal_J60_CMD.position_ = READY_TO_GET_STRAIGHT_KFS_J60 + (GET_KFS_LEFT_J60 - READY_TO_GET_STRAIGHT_KFS_J60) * rampSignalFP(sm_cnt, 600);
                        Motor_A1.TargetPos = READY_TO_GET_KFS_A1 + (GET_KFS_LEFT_LOW_A1 + 0.2f - READY_TO_GET_KFS_A1) * rampSignalFP(sm_cnt - 200, 800);
                        stretch_2006.outerTarget = READY_TO_GET_KFS_S2006 + (GET_KFS_LEFT_LOW_S2006 - READY_TO_GET_KFS_S2006) * rampSignalFP(sm_cnt - 400, 600);
                        stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM + (GET_KFS_LEFT_LOW_DM - READY_TO_GET_KFS_DM) * rampSignalFP(sm_cnt - 400, 600);
                    }
                    else if (KFS_height == 0 && KFS_orientation == 2) // lower right
                    {
                        Gimbal_J60_CMD.position_ = READY_TO_GET_STRAIGHT_KFS_J60 + (GET_KFS_RIGHT_J60 - READY_TO_GET_STRAIGHT_KFS_J60) * rampSignalFP(sm_cnt, 600);
                        Motor_A1.TargetPos = READY_TO_GET_KFS_A1 + (GET_KFS_LEFT_LOW_A1 + 0.2f - READY_TO_GET_KFS_A1) * rampSignalFP(sm_cnt - 200, 800);
                        stretch_2006.outerTarget = READY_TO_GET_KFS_S2006 + (GET_KFS_LEFT_LOW_S2006 - READY_TO_GET_KFS_S2006) * rampSignalFP(sm_cnt - 400, 600);
                        stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM + (GET_KFS_LEFT_LOW_DM - READY_TO_GET_KFS_DM) * rampSignalFP(sm_cnt - 400, 600);
                    }

                    else if (KFS_height == 1 && KFS_orientation == 0) // higher straight
                    {
                        Gimbal_J60_CMD.position_ = GET_KFS_STRAIGHT_J60;
                        Motor_A1.TargetPos = READY_TO_GET_KFS_A1 + (GET_KFS_STRAIGHT_HIGH_A1 - READY_TO_GET_KFS_A1) * rampSignalFP(sm_cnt, 800);
                        stretch_2006.outerTarget = READY_TO_GET_KFS_S2006 + (GET_KFS_STRAIGHT_HIGH_S2006 - 200 - READY_TO_GET_KFS_S2006) * rampSignalFP(sm_cnt, 800);
                        stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM + (GET_KFS_STRAIGHT_HIGH_DM - READY_TO_GET_KFS_DM) * rampSignalFP(sm_cnt, 800);
                        if (sm_cnt > 800)
                        {
                            sm_cnt = 1001;
                        }
                    }
                    else if (KFS_height == 1 && KFS_orientation == 1) // higher LEFT
                    {
                        Gimbal_J60_CMD.position_ = READY_TO_GET_STRAIGHT_KFS_J60 + (GET_KFS_LEFT_J60 - READY_TO_GET_STRAIGHT_KFS_J60) * rampSignalFP(sm_cnt, 600);
                        Motor_A1.TargetPos = READY_TO_GET_KFS_A1 + (GET_KFS_LEFT_HIGH_A1 - READY_TO_GET_KFS_A1) * rampSignalFP(sm_cnt - 200, 800);
                        stretch_2006.outerTarget = READY_TO_GET_KFS_S2006 + (GET_KFS_LEFT_HIGH_S2006 - 200 - READY_TO_GET_KFS_S2006) * rampSignalFP(sm_cnt - 400, 600);
                        stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM + (GET_KFS_LEFT_HIGH_DM - READY_TO_GET_KFS_DM) * rampSignalFP(sm_cnt - 400, 600);
                    }
                    else if (KFS_height == 1 && KFS_orientation == 2) // higher RIGHT
                    {
                        Gimbal_J60_CMD.position_ = READY_TO_GET_STRAIGHT_KFS_J60 + (GET_KFS_RIGHT_J60 - READY_TO_GET_STRAIGHT_KFS_J60) * rampSignalFP(sm_cnt, 600);
                        Motor_A1.TargetPos = READY_TO_GET_KFS_A1 + (GET_KFS_LEFT_HIGH_A1 - READY_TO_GET_KFS_A1) * rampSignalFP(sm_cnt - 200, 800);
                        stretch_2006.outerTarget = READY_TO_GET_KFS_S2006 + (GET_KFS_LEFT_HIGH_S2006 - 200 - READY_TO_GET_KFS_S2006) * rampSignalFP(sm_cnt - 400, 600);
                        stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM + (GET_KFS_LEFT_HIGH_DM - READY_TO_GET_KFS_DM) * rampSignalFP(sm_cnt - 400, 600);
                    }

                    else if (KFS_height == 2 && KFS_orientation == 0) // TOP straight
                    {
                        Gimbal_J60_CMD.position_ = GET_KFS_STRAIGHT_J60;
                        Motor_A1.TargetPos = READY_TO_GET_KFS_A1 + (GET_KFS_STRAIGHT_TOP_A1 - READY_TO_GET_KFS_A1) * rampSignalFP(sm_cnt, 800);
                        stretch_2006.outerTarget = READY_TO_GET_KFS_S2006 + (GET_KFS_STRAIGHT_TOP_S2006 - 200 - READY_TO_GET_KFS_S2006) * rampSignalFP(sm_cnt, 800);
                        stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM + (GET_KFS_STRAIGHT_TOP_DM - READY_TO_GET_KFS_DM) * rampSignalFP(sm_cnt, 800);
                        if (sm_cnt > 800)
                        {
                            sm_cnt = 1001;
                        }
                    }
                    else if (KFS_height == 3 && KFS_orientation == 0) // Ⅲ get KFS from ground
                    {
                        Gimbal_J60_CMD.position_ = GET_KFS_STRAIGHT_J60;
                        Motor_A1.TargetPos = READY_TO_GET_KFS_A1 + (GET_KFS_III_A1 + 0.3f - READY_TO_GET_KFS_A1) * rampSignalFP(sm_cnt, 800);
                        stretch_2006.outerTarget = READY_TO_GET_KFS_S2006 + (GET_KFS_III_S2006 - READY_TO_GET_KFS_S2006) * rampSignalFP(sm_cnt, 800);
                        stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM + (GET_KFS_III_DM - READY_TO_GET_KFS_DM) * rampSignalFP(sm_cnt, 800);
                        if (sm_cnt > 800)
                        {
                            sm_cnt = 1001;
                        }
                    }
                }
                else if (sm_cnt > 1000 && sm_cnt <= 1500)
                {
                    if (completeActionFlag == 1)
                    {
                        sm_cnt++;
                        chassisMoveFlag = 0;
                        if (KFS_height == 0 && KFS_orientation == 0) // lower straight
                        {
                            Gimbal_J60_CMD.position_ = GET_KFS_STRAIGHT_J60;
                            Motor_A1.TargetPos = GET_KFS_STRAIGHT_LOW_A1 + 0.2f - 0.2f * rampSignalFP(sm_cnt - 1000, 200);
                            stretch_2006.outerTarget = GET_KFS_STRAIGHT_LOW_S2006;
                            stretch_DM.ctrl.pos_set = GET_KFS_STRAIGHT_LOW_DM;
                        }
                        else if (KFS_height == 0 && KFS_orientation == 1) // lower left
                        {
                            Gimbal_J60_CMD.position_ = GET_KFS_LEFT_J60;
                            Motor_A1.TargetPos = GET_KFS_LEFT_LOW_A1 + 0.2f - 0.2f * rampSignalFP(sm_cnt - 1000, 200);
                            stretch_2006.outerTarget = GET_KFS_LEFT_LOW_S2006;
                            stretch_DM.ctrl.pos_set = GET_KFS_LEFT_LOW_DM;
                        }
                        else if (KFS_height == 0 && KFS_orientation == 2) // lower right
                        {
                            Gimbal_J60_CMD.position_ = GET_KFS_RIGHT_J60;
                            Motor_A1.TargetPos = GET_KFS_LEFT_LOW_A1 + 0.2f - 0.2f * rampSignalFP(sm_cnt - 1000, 200);
                            stretch_2006.outerTarget = GET_KFS_LEFT_LOW_S2006;
                            stretch_DM.ctrl.pos_set = GET_KFS_LEFT_LOW_DM;
                        }
                        else if (KFS_height == 1 && KFS_orientation == 0) // higher straight
                        {
                            Gimbal_J60_CMD.position_ = GET_KFS_STRAIGHT_J60;
                            Motor_A1.TargetPos = GET_KFS_STRAIGHT_HIGH_A1;
                            stretch_2006.outerTarget = GET_KFS_STRAIGHT_HIGH_S2006 - 200 + 200 * rampSignalFP(sm_cnt - 1000, 200);
                            stretch_DM.ctrl.pos_set = GET_KFS_STRAIGHT_HIGH_DM;
                        }
                        else if (KFS_height == 1 && KFS_orientation == 1) // higher left
                        {
                            Gimbal_J60_CMD.position_ = GET_KFS_LEFT_J60;
                            Motor_A1.TargetPos = GET_KFS_LEFT_HIGH_A1;
                            stretch_2006.outerTarget = GET_KFS_LEFT_HIGH_S2006 - 200 + 200 * rampSignalFP(sm_cnt - 1000, 200);
                            stretch_DM.ctrl.pos_set = GET_KFS_LEFT_HIGH_DM;
                        }
                        else if (KFS_height == 1 && KFS_orientation == 2) // higher right
                        {
                            Gimbal_J60_CMD.position_ = GET_KFS_RIGHT_J60;
                            Motor_A1.TargetPos = GET_KFS_LEFT_HIGH_A1;
                            stretch_2006.outerTarget = GET_KFS_LEFT_HIGH_S2006 - 200 + 200 * rampSignalFP(sm_cnt - 1000, 200);
                            stretch_DM.ctrl.pos_set = GET_KFS_LEFT_HIGH_DM;
                        }
                        else if (KFS_height == 2 && KFS_orientation == 0) // TOP straight
                        {
                            Gimbal_J60_CMD.position_ = GET_KFS_STRAIGHT_J60;
                            Motor_A1.TargetPos = GET_KFS_STRAIGHT_TOP_A1;
                            stretch_2006.outerTarget = GET_KFS_STRAIGHT_TOP_S2006 - 200 + 200 * rampSignalFP(sm_cnt - 1000, 200);
                            stretch_DM.ctrl.pos_set = GET_KFS_STRAIGHT_TOP_DM;
                        }
                        else if (KFS_height == 3 && KFS_orientation == 0) // TOP straight
                        {
                            Gimbal_J60_CMD.position_ = GET_KFS_STRAIGHT_J60;
                            Motor_A1.TargetPos = GET_KFS_III_A1 + 0.3f - 0.3f * rampSignalFP(sm_cnt - 1000, 200);
                            stretch_2006.outerTarget = GET_KFS_III_S2006;
                            stretch_DM.ctrl.pos_set = GET_KFS_III_DM;
                        }
                    }
                }
                else
                {
                    sm_cnt++;
                    gravityCompensation_state = 1;
                    fp32 k = rampSignalFP(sm_cnt - 1500, 900);
                    if (sm_cnt < 1800)
                    {
                        chassisMoveFlag = 1;
                    }
                    if (KFS_height == 0 && KFS_orientation == 0) // lower straight
                    {
                        Gimbal_J60_CMD.position_ = HOLD_KFS_J60;
                        Motor_A1.TargetPos = GET_KFS_STRAIGHT_LOW_A1 + (HOLD_KFS_A1 - GET_KFS_STRAIGHT_LOW_A1) * k;
                        stretch_2006.outerTarget = GET_KFS_STRAIGHT_LOW_S2006 + (HOLD_KFS_S2006 - GET_KFS_STRAIGHT_LOW_S2006) * k;
                        stretch_DM.ctrl.pos_set = GET_KFS_STRAIGHT_LOW_DM + (HOLD_KFS_DM - GET_KFS_STRAIGHT_LOW_DM) * k;
                    }
                    else if (KFS_height == 0 && KFS_orientation == 1) // lower left
                    {
                        Gimbal_J60_CMD.position_ = GET_KFS_LEFT_J60 + (HOLD_KFS_J60 - GET_KFS_LEFT_J60) * k;
                        Motor_A1.TargetPos = GET_KFS_LEFT_LOW_A1 + (HOLD_KFS_A1 - GET_KFS_LEFT_LOW_A1) * k;
                        stretch_2006.outerTarget = GET_KFS_LEFT_LOW_S2006 + (HOLD_KFS_S2006 - GET_KFS_LEFT_LOW_S2006) * k;
                        stretch_DM.ctrl.pos_set = GET_KFS_LEFT_LOW_DM + (HOLD_KFS_DM - GET_KFS_LEFT_LOW_DM) * k;
                    }
                    else if (KFS_height == 0 && KFS_orientation == 2) // lower right
                    {
                        Gimbal_J60_CMD.position_ = GET_KFS_RIGHT_J60 + (HOLD_KFS_J60 - GET_KFS_RIGHT_J60) * k;
                        Motor_A1.TargetPos = GET_KFS_LEFT_LOW_A1 + (HOLD_KFS_A1 - GET_KFS_LEFT_LOW_A1) * k;
                        stretch_2006.outerTarget = GET_KFS_LEFT_LOW_S2006 + (HOLD_KFS_S2006 - GET_KFS_LEFT_LOW_S2006) * k;
                        stretch_DM.ctrl.pos_set = GET_KFS_LEFT_LOW_DM + (HOLD_KFS_DM - GET_KFS_LEFT_LOW_DM) * k;
                    }
                    else if (KFS_height == 1 && KFS_orientation == 0) // higher straight
                    {
                        Gimbal_J60_CMD.position_ = HOLD_KFS_J60;
                        Motor_A1.TargetPos = GET_KFS_STRAIGHT_HIGH_A1 + (HOLD_KFS_A1 - GET_KFS_STRAIGHT_HIGH_A1) * k;
                        stretch_2006.outerTarget = GET_KFS_STRAIGHT_HIGH_S2006 + (HOLD_KFS_S2006 - GET_KFS_STRAIGHT_HIGH_S2006) * k;
                        stretch_DM.ctrl.pos_set = GET_KFS_STRAIGHT_HIGH_DM + (HOLD_KFS_DM - GET_KFS_STRAIGHT_HIGH_DM) * k;
                    }
                    else if (KFS_height == 1 && KFS_orientation == 1) // higher left
                    {
                        Gimbal_J60_CMD.position_ = GET_KFS_LEFT_J60 + (HOLD_KFS_J60 - GET_KFS_LEFT_J60) * k;
                        Motor_A1.TargetPos = GET_KFS_LEFT_HIGH_A1 + (HOLD_KFS_A1 - GET_KFS_LEFT_HIGH_A1) * k;
                        stretch_2006.outerTarget = GET_KFS_LEFT_HIGH_S2006 + (HOLD_KFS_S2006 - GET_KFS_LEFT_HIGH_S2006) * k;
                        stretch_DM.ctrl.pos_set = GET_KFS_LEFT_HIGH_DM + (HOLD_KFS_DM - GET_KFS_LEFT_HIGH_DM) * k;
                    }
                    else if (KFS_height == 1 && KFS_orientation == 2) // higher right
                    {
                        Gimbal_J60_CMD.position_ = GET_KFS_RIGHT_J60 + (HOLD_KFS_J60 - GET_KFS_RIGHT_J60) * k;
                        Motor_A1.TargetPos = GET_KFS_LEFT_HIGH_A1 + (HOLD_KFS_A1 - GET_KFS_LEFT_HIGH_A1) * k;
                        stretch_2006.outerTarget = GET_KFS_LEFT_HIGH_S2006 + (HOLD_KFS_S2006 - GET_KFS_LEFT_HIGH_S2006) * k;
                        stretch_DM.ctrl.pos_set = GET_KFS_LEFT_HIGH_DM + (HOLD_KFS_DM - GET_KFS_LEFT_HIGH_DM) * k;
                    }
                    else if (KFS_height == 2 && KFS_orientation == 0) // TOP straight
                    {
                        Gimbal_J60_CMD.position_ = HOLD_KFS_J60;
                        Motor_A1.TargetPos = GET_KFS_STRAIGHT_TOP_A1 + (HOLD_KFS_A1 - GET_KFS_STRAIGHT_TOP_A1) * k;
                        stretch_2006.outerTarget = GET_KFS_STRAIGHT_TOP_S2006 + (HOLD_KFS_S2006 - GET_KFS_STRAIGHT_TOP_S2006) * k;
                        stretch_DM.ctrl.pos_set = GET_KFS_STRAIGHT_TOP_DM + (HOLD_KFS_DM - GET_KFS_STRAIGHT_TOP_DM) * k;
                    }
                    else if (KFS_height == 3 && KFS_orientation == 0) // TOP straight
                    {
                        Gimbal_J60_CMD.position_ = HOLD_KFS_J60;
                        Motor_A1.TargetPos = GET_KFS_III_A1 + (HOLD_KFS_A1 - GET_KFS_III_A1) * k;
                        stretch_2006.outerTarget = GET_KFS_III_S2006 + (HOLD_KFS_S2006 - GET_KFS_III_S2006) * k;
                        stretch_DM.ctrl.pos_set = GET_KFS_III_DM + (HOLD_KFS_DM - GET_KFS_III_DM) * k;
                    }
                    else
                    {
                        sm_cnt--;
                    }

                    if (sm_cnt > 2400)
                    {
                        actionCpltFlag = 1;
                        stateFlag = HOLDING_KFS;
                        sm_cnt = 0;
                        chassisMoveFlag = 0;
                        completeActionFlag = 0;
                    }
                }
                break;
            default:
                actionCpltFlag = 1;
                break;
            }
            break;
        case STORAGE_KFS:
            switch (stateFlag)
            {
            case HOLDING_KFS:
                sm_cnt++;
                chassisMoveFlag = 1;
                if (sm_cnt < 1200)
                {
                    airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                    airOperator.airOperatorTxBuf[2] = 1; // 储存气泵
                    airOperator.airOperatorTxBuf[4] = 0; // 吸气
                    if (sm_cnt < 700)
                    {
                        airOperator.airOperatorTxBuf[1] = 1; // 伸缩臂气泵
                        airOperator.airOperatorTxBuf[3] = 0; // 吸气
                    }
                    else
                    {
                        gravityCompensation_state = 0;
                        airOperator.airOperatorTxBuf[1] = 0; // 伸缩臂气泵
                        airOperator.airOperatorTxBuf[3] = 1; // 吐气
                    }
                    Gimbal_J60_CMD.position_ = HOLD_KFS_J60 + (STORAGE_KFS_J60 - HOLD_KFS_J60) * rampSignalFP(sm_cnt, 500);
                    Motor_A1.TargetPos = HOLD_KFS_A1 + (STORAGE_KFS_A1 - HOLD_KFS_A1) * rampSignalFP(sm_cnt, 600);
                    stretch_2006.outerTarget = HOLD_KFS_S2006 + (STORAGE_KFS_S2006 - HOLD_KFS_S2006) * rampSignalFP(sm_cnt, 580);
                    stretch_DM.ctrl.pos_set = HOLD_KFS_DM + (STORAGE_KFS_DM - HOLD_KFS_DM) * rampSignalFP(sm_cnt, 600);
                }
                else if (sm_cnt >= 1200 && sm_cnt < 2000)
                {
                    airOperator.airOperatorTxBuf[1] = 1; // 伸缩臂气泵
                    airOperator.airOperatorTxBuf[3] = 0; // 吸气
                    if (sm_cnt < 1700)
                    {
                        Motor_A1.TargetPos = STORAGE_KFS_A1 + (READY_TO_GET_KFS_A1 - 0.4f - STORAGE_KFS_A1) * rampSignalFP(sm_cnt - 1200, 500);
                    }
                    else
                    {
                        Motor_A1.TargetPos = READY_TO_GET_KFS_A1 - 0.4f + 0.4f * rampSignalFP(sm_cnt - 1700, 300);
                    }
                    stretch_2006.outerTarget = STORAGE_KFS_S2006 + (READY_TO_GET_KFS_S2006 - STORAGE_KFS_S2006) * rampSignalFP(sm_cnt - 1400, 600);
                    stretch_DM.ctrl.pos_set = STORAGE_KFS_DM + (READY_TO_GET_KFS_DM - STORAGE_KFS_DM) * rampSignalFP(sm_cnt - 1400, 500);
                }
                else
                {
                    actionCpltFlag = 1;
                    chassisMoveFlag = 0;
                    stateFlag = INIT;
                    sm_cnt = 0;
                }
                break;
            default:
                actionCpltFlag = 1;
                break;
            }
            break;
        case GET_KFS_BEHIND:
            switch (stateFlag)
            {
            case INIT:
                sm_cnt++;
                chassisMoveFlag = 1;
                if (sm_cnt < 1200)
                {
                    airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                    airOperator.airOperatorTxBuf[1] = 1; // 伸缩臂气泵
                    airOperator.airOperatorTxBuf[3] = 0; // 吸气
                    if (sm_cnt < 800)
                    {
                        airOperator.airOperatorTxBuf[2] = 1; // 储存气泵
                        airOperator.airOperatorTxBuf[4] = 0; // 吸气
                    }
                    else
                    {
                        gravityCompensation_state = 1;
                        airOperator.airOperatorTxBuf[2] = 0; // 储存气泵
                        airOperator.airOperatorTxBuf[4] = 1; // 吐气
                    }
                    Gimbal_J60_CMD.position_ = READY_TO_GET_STRAIGHT_KFS_J60;
                    if (sm_cnt < 300)
                    {
                        Motor_A1.TargetPos = READY_TO_GET_KFS_A1 + (GET_KFS_BEHIND_A1 - 0.5f - READY_TO_GET_KFS_A1) * rampSignalFP(sm_cnt, 300);
                        stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM + (STORAGE_KFS_DM - READY_TO_GET_KFS_DM) * rampSignalFP(sm_cnt, 300);
                    }
                    else
                    {
                        Motor_A1.TargetPos = GET_KFS_BEHIND_A1 - 0.5f + 0.5f * rampSignalFP(sm_cnt - 300, 500);
                        stretch_DM.ctrl.pos_set = 4.066f - Motor_A1.TargetPos;
                    }
                    stretch_2006.outerTarget = READY_TO_GET_KFS_S2006 + (STORAGE_KFS_S2006 - READY_TO_GET_KFS_S2006) * rampSignalFP(sm_cnt, 400);
                    //                        stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM+(STORAGE_KFS_DM-READY_TO_GET_KFS_DM)*rampSignalFP(sm_cnt,500);
                    //                        move_2006.outerTarget = GET_WEAPON_HEAD_M2006;
                    //                        DJI_3508.outerTarget = GET_WEAPON_HEAD_3508;
                }
                else if (sm_cnt >= 1200 && sm_cnt < 2000)
                {
                    airOperator.airOperatorTxBuf[2] = 0; // 储存气泵
                    airOperator.airOperatorTxBuf[4] = 0; // 吸气
                    Gimbal_J60_CMD.position_ = READY_TO_GET_STRAIGHT_KFS_J60;
                    Motor_A1.TargetPos = STORAGE_KFS_A1 + (HOLD_KFS_A1 - STORAGE_KFS_A1) * rampSignalFP(sm_cnt - 1200, 700);
                    stretch_2006.outerTarget = STORAGE_KFS_S2006 + (HOLD_KFS_S2006 - STORAGE_KFS_S2006) * rampSignalFP(sm_cnt - 1200, 700);
                    stretch_DM.ctrl.pos_set = STORAGE_KFS_DM + (HOLD_KFS_DM - STORAGE_KFS_DM) * rampSignalFP(sm_cnt - 1200, 600);
                }
                else
                {
                    chassisMoveFlag = 0;
                    actionCpltFlag = 1;
                    stateFlag = HOLDING_KFS;
                    sm_cnt = 0;
                }
                break;
            default:
                actionCpltFlag = 1;
                break;
            }
            break;
        case LEAVE_KFS_BACK:
            switch (stateFlag)
            {
            case HOLDING_KFS:
                sm_cnt++;
                if (sm_cnt < 1200)
                {
                    airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                    airOperator.airOperatorTxBuf[2] = 0; // 伸缩臂气泵
                    airOperator.airOperatorTxBuf[4] = 0; // 吸气
                    if (sm_cnt < 800)
                    {
                        airOperator.airOperatorTxBuf[1] = 1; // 储存气泵
                        airOperator.airOperatorTxBuf[3] = 0; // 吸气
                    }
                    else
                    {
                        gravityCompensation_state = 0;
                        chassisMoveFlag = 1;
                        airOperator.airOperatorTxBuf[1] = 0; // 储存气泵
                        airOperator.airOperatorTxBuf[3] = 1; // 吐气
                    }
                    Gimbal_J60_CMD.position_ = STORAGE_KFS_J60;
                    Motor_A1.TargetPos = HOLD_KFS_A1 + (LEAVE_KFS_BACK_A1 - HOLD_KFS_A1) * rampSignalFP(sm_cnt, 800);
                    stretch_2006.outerTarget = HOLD_KFS_S2006 + (LEAVE_KFS_BACK_S2006 - HOLD_KFS_S2006) * rampSignalFP(sm_cnt, 800);
                    stretch_DM.ctrl.pos_set = HOLD_KFS_DM + (LEAVE_KFS_BACK_DM - HOLD_KFS_DM) * rampSignalFP(sm_cnt, 800);

                    //                        move_2006.outerTarget = STANDBY_M2006;
                    //                        DJI_3508.outerTarget = STANDBY_3508;
                }
                else if (sm_cnt >= 1200 && sm_cnt < 1800)
                {
                    airOperator.airOperatorTxBuf[3] = 0; // 吸气
                    Motor_A1.TargetPos = LEAVE_KFS_BACK_A1 + (READY_TO_GET_KFS_A1 - LEAVE_KFS_BACK_A1) * rampSignalFP(sm_cnt - 1200, 600);
                    stretch_2006.outerTarget = LEAVE_KFS_BACK_S2006 + (READY_TO_GET_KFS_S2006 - LEAVE_KFS_BACK_S2006) * rampSignalFP(sm_cnt - 1200, 600);
                    stretch_DM.ctrl.pos_set = LEAVE_KFS_BACK_DM + (READY_TO_GET_KFS_DM - LEAVE_KFS_BACK_DM) * rampSignalFP(sm_cnt - 1200, 600);
                }
                else
                {
                    chassisMoveFlag = 0;
                    actionCpltFlag = 1;
                    stateFlag = INIT;
                    sm_cnt = 0;
                }
                break;
            default:
                actionCpltFlag = 1;
                break;
            }
            break;
        case LEAVE_KFS_FRONT:
            switch (stateFlag)
            {
            case HOLDING_KFS:
                sm_cnt++;
                if (sm_cnt < 900)
                {
                    airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                    airOperator.airOperatorTxBuf[2] = 0; // 伸缩臂气泵
                    airOperator.airOperatorTxBuf[4] = 0; // 吸气
                    if (sm_cnt < 600)
                    {
                        airOperator.airOperatorTxBuf[1] = 1; // 储存气泵
                        airOperator.airOperatorTxBuf[3] = 0; // 吸气
                    }
                    else
                    {
                        chassisMoveFlag = 1;
                        gravityCompensation_state = 0;
                        airOperator.airOperatorTxBuf[1] = 0; // 储存气泵
                        airOperator.airOperatorTxBuf[3] = 1; // 吐气
                    }
                    Gimbal_J60_CMD.position_ = HOLD_KFS_J60;
                    Motor_A1.TargetPos = HOLD_KFS_A1 + (LEAVE_KFS_FRONT_A1 - HOLD_KFS_A1) * rampSignalFP(sm_cnt, 600);
                    stretch_2006.outerTarget = HOLD_KFS_S2006 + (LEAVE_KFS_FRONT_S2006 - HOLD_KFS_S2006) * rampSignalFP(sm_cnt, 600);
                    stretch_DM.ctrl.pos_set = HOLD_KFS_DM + (LEAVE_KFS_FRONT_DM - HOLD_KFS_DM) * rampSignalFP(sm_cnt, 600);
                }
                else if (sm_cnt >= 900 && sm_cnt < 1500)
                {
                    Motor_A1.TargetPos = LEAVE_KFS_FRONT_A1 + (READY_TO_GET_KFS_A1 - LEAVE_KFS_FRONT_A1) * rampSignalFP(sm_cnt - 900, 500);
                    stretch_2006.outerTarget = LEAVE_KFS_FRONT_S2006 + (READY_TO_GET_KFS_S2006 - LEAVE_KFS_FRONT_S2006) * rampSignalFP(sm_cnt - 900, 600);
                    stretch_DM.ctrl.pos_set = LEAVE_KFS_FRONT_DM + (READY_TO_GET_KFS_DM - LEAVE_KFS_FRONT_DM) * rampSignalFP(sm_cnt - 900, 600);
                }
                else
                {
                    airOperator.airOperatorTxBuf[1] = 0; // 储存气泵
                    airOperator.airOperatorTxBuf[3] = 0; // 吸气
                    actionCpltFlag = 1;
                    chassisMoveFlag = 0;
                    stateFlag = INIT;
                    sm_cnt = 0;
                }
                break;
            default:
                actionCpltFlag = 1;
                break;
            }
            break;
        case USE_KFS:
            switch (stateFlag)
            {
            case HOLDING_KFS:
                if (sm_cnt <= 2000)
                {
                    sm_cnt++;
                    airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                    airOperator.airOperatorTxBuf[1] = 1; // 储存气泵
                    airOperator.airOperatorTxBuf[3] = 0; // 吸气
                    chassisMoveFlag = 0;
                    if (KFS_level == 0) // middle level
                    {
                        Gimbal_J60_CMD.position_ = USE_KFS_MIDDLE_J60;
                        Motor_A1.TargetPos = HOLD_KFS_A1 + (USE_KFS_MIDDLE_A1 + 0.3f - HOLD_KFS_A1) * rampSignalFP(sm_cnt - 1400, 600);
                        stretch_2006.outerTarget = HOLD_KFS_S2006 + (USE_KFS_MIDDLE_S2006 - HOLD_KFS_S2006) * rampSignalFP(sm_cnt, 600);
                        stretch_DM.ctrl.pos_set = HOLD_KFS_DM + (USE_KFS_MIDDLE_DM - HOLD_KFS_DM) * rampSignalFP(sm_cnt - 600, 800);
                        //                            move_2006.outerTarget = STANDBY_M2006;
                        //                            DJI_3508.outerTarget = STANDBY_3508;
                    }
                    else if (KFS_level == 1 && use_KFS_orientation == 0) // high left
                    {
                        Gimbal_J60_CMD.position_ = USE_KFS_TOP_LEFT_J60;
                        Motor_A1.TargetPos = HOLD_KFS_A1 + (USE_KFS_TOP_LEFT_A1 + 0.3f - HOLD_KFS_A1) * rampSignalFP(sm_cnt - 1400, 600);
                        stretch_2006.outerTarget = HOLD_KFS_S2006 + (USE_KFS_TOP_LEFT_S2006 - HOLD_KFS_S2006) * rampSignalFP(sm_cnt, 600);
                        stretch_DM.ctrl.pos_set = HOLD_KFS_DM + (USE_KFS_TOP_LEFT_DM - HOLD_KFS_DM) * rampSignalFP(sm_cnt - 600, 800);
                        //                            move_2006.outerTarget = STANDBY_M2006;
                        //                            DJI_3508.outerTarget = STANDBY_3508;
                    }
                    else if (KFS_level == 1 && use_KFS_orientation == 1) // high right
                    {
                        Gimbal_J60_CMD.position_ = USE_KFS_TOP_RIGHT_J60;
                        Motor_A1.TargetPos = HOLD_KFS_A1 + (USE_KFS_TOP_RIGHT_A1 + 0.3f - HOLD_KFS_A1) * rampSignalFP(sm_cnt - 1400, 600);
                        stretch_2006.outerTarget = HOLD_KFS_S2006 + (USE_KFS_TOP_RIGHT_S2006 - HOLD_KFS_S2006) * rampSignalFP(sm_cnt, 600);
                        stretch_DM.ctrl.pos_set = HOLD_KFS_DM + (USE_KFS_TOP_RIGHT_DM - HOLD_KFS_DM) * rampSignalFP(sm_cnt - 600, 800);
                        //                            move_2006.outerTarget = STANDBY_M2006;
                        //                            DJI_3508.outerTarget = STANDBY_3508;
                    }
                }
                else if (sm_cnt <= 3200 && sm_cnt > 2000)
                {
                    if (completeActionFlag == 1)
                    {
                        sm_cnt++;
                        if (sm_cnt > 2200 && sm_cnt < 2800)
                        {
                            gravityCompensation_state = 0;
                            airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                            airOperator.airOperatorTxBuf[1] = 0; // 储存气泵
                            airOperator.airOperatorTxBuf[3] = 1; // 吐气
                        }
                        else if (sm_cnt >= 2800)
                        {
                            airOperator.airOperatorTxBuf[3] = 0; // 吸气
                            chassisMoveFlag = 1;
                        }
                        if (KFS_level == 0) // middle level
                        {
                            Gimbal_J60_CMD.position_ = USE_KFS_MIDDLE_J60;
                            Motor_A1.TargetPos = USE_KFS_MIDDLE_A1 + 0.3f - 0.4f * rampSignalFP(sm_cnt - 2000, 600);
                            stretch_2006.outerTarget = USE_KFS_MIDDLE_S2006;
                            stretch_DM.ctrl.pos_set = USE_KFS_MIDDLE_DM;
                        }
                        else if (KFS_level == 1 && use_KFS_orientation == 0) // high left
                        {
                            Gimbal_J60_CMD.position_ = USE_KFS_TOP_LEFT_J60;
                            Motor_A1.TargetPos = USE_KFS_TOP_LEFT_A1 + 0.3f - 0.4f * rampSignalFP(sm_cnt - 2000, 600);
                            stretch_2006.outerTarget = USE_KFS_TOP_LEFT_S2006;
                            stretch_DM.ctrl.pos_set = USE_KFS_TOP_LEFT_DM;
                        }
                        else if (KFS_level == 1 && use_KFS_orientation == 1) // high right
                        {
                            Gimbal_J60_CMD.position_ = USE_KFS_TOP_RIGHT_J60;
                            Motor_A1.TargetPos = USE_KFS_TOP_RIGHT_A1 + 0.3f - 0.4f * rampSignalFP(sm_cnt - 2000, 600);
                            stretch_2006.outerTarget = USE_KFS_TOP_RIGHT_S2006;
                            stretch_DM.ctrl.pos_set = USE_KFS_TOP_RIGHT_DM;
                        }
                    }
                    else
                    {
                        chassisMoveFlag = 0;
                    }
                }
                else
                {
                    sm_cnt++;
                    chassisMoveFlag = 1;
                    airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                    airOperator.airOperatorTxBuf[1] = 0; // 储存气泵
                    airOperator.airOperatorTxBuf[3] = 0; // 吸气
                    if (KFS_level == 0)                  // middle level
                    {
                        Gimbal_J60_CMD.position_ = READY_TO_GET_STRAIGHT_KFS_J60;
                        Motor_A1.TargetPos = USE_KFS_MIDDLE_A1 - 0.1f + (READY_TO_GET_KFS_A1 - USE_KFS_MIDDLE_A1 - 0.1f) * rampSignalFP(sm_cnt - 3200, 600);
                        stretch_2006.outerTarget = USE_KFS_MIDDLE_S2006 + (READY_TO_GET_KFS_S2006 - USE_KFS_MIDDLE_S2006) * rampSignalFP(sm_cnt - 3200, 600);
                        stretch_DM.ctrl.pos_set = USE_KFS_MIDDLE_DM + (READY_TO_GET_KFS_DM - USE_KFS_MIDDLE_DM) * rampSignalFP(sm_cnt - 3200, 600);
                    }
                    else if (KFS_level == 1 && use_KFS_orientation == 0) // high left
                    {
                        Gimbal_J60_CMD.position_ = READY_TO_GET_STRAIGHT_KFS_J60;
                        Motor_A1.TargetPos = USE_KFS_TOP_LEFT_A1 - 0.1f + (READY_TO_GET_KFS_A1 - USE_KFS_TOP_LEFT_A1 - 0.1f) * rampSignalFP(sm_cnt - 3200, 600);
                        stretch_2006.outerTarget = USE_KFS_TOP_LEFT_S2006 + (READY_TO_GET_KFS_S2006 - USE_KFS_TOP_LEFT_S2006) * rampSignalFP(sm_cnt - 3200, 600);
                        stretch_DM.ctrl.pos_set = USE_KFS_TOP_LEFT_DM + (READY_TO_GET_KFS_DM - USE_KFS_TOP_LEFT_DM) * rampSignalFP(sm_cnt - 3200, 600);
                    }
                    else if (KFS_level == 1 && use_KFS_orientation == 1) // high right
                    {
                        Gimbal_J60_CMD.position_ = READY_TO_GET_STRAIGHT_KFS_J60;
                        Motor_A1.TargetPos = USE_KFS_TOP_RIGHT_A1 - 0.1f + (READY_TO_GET_KFS_A1 - USE_KFS_TOP_RIGHT_A1 - 0.1f) * rampSignalFP(sm_cnt - 3200, 600);
                        stretch_2006.outerTarget = USE_KFS_TOP_RIGHT_S2006 + (READY_TO_GET_KFS_S2006 - USE_KFS_TOP_RIGHT_S2006) * rampSignalFP(sm_cnt - 3200, 600);
                        stretch_DM.ctrl.pos_set = USE_KFS_TOP_RIGHT_DM + (READY_TO_GET_KFS_DM - USE_KFS_TOP_RIGHT_DM) * rampSignalFP(sm_cnt - 3200, 600);
                    }
                    if (sm_cnt > 3800)
                    {
                        actionCpltFlag = 1;
                        stateFlag = INIT;
                        sm_cnt = 0;
                        chassisMoveFlag = 0;
                        completeActionFlag = 0;
                    }
                }
                break;
            default:
                actionCpltFlag = 1;
                break;
            }
            break;
        default:
            break;
        }
    }
    else
    {
        switch (stateFlag)
        {
        case STAND_BY:
            switch (actionFlag)
            {
            case INIT_ACTION:
                actionCpltFlag = 0;
                break;
            default:
                airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                airOperator.airOperatorTxBuf[1] = 0; // 伸缩臂气泵
                airOperator.airOperatorTxBuf[2] = 0; // 储存气泵

                Gimbal_J60_CMD.position_ = STANDBY_J60;
                Motor_A1.TargetPos = STANDBY_A1;
                stretch_2006.outerTarget = STANDBY_S2006;
                stretch_DM.ctrl.pos_set = STANDBY_DM;

                // move_2006.outerTarget = STANDBY_M2006;
                DJI_3508.outerTarget = STANDBY_3508;
                break;
            }
            break;
        case INIT:
            switch (actionFlag)
            {
            case GET_WEAPON_HEAD_FOR_COMBINE:
            case DEINIT:
            case GET_KFS:
            case GET_KFS_BEHIND:
                actionCpltFlag = 0;
                break;
            default:
                airOperator.airOperatorTxBuf[0] = 1; // 夹爪
                airOperator.airOperatorTxBuf[1] = 0; // 伸缩臂气泵
                //                airOperator.airOperatorTxBuf[2]=0;//储存气泵

                Gimbal_J60_CMD.position_ = READY_TO_GET_STRAIGHT_KFS_J60;
                Motor_A1.TargetPos = READY_TO_GET_KFS_A1;
                stretch_2006.outerTarget = READY_TO_GET_KFS_S2006;
                stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM;

                //                move_2006.outerTarget=GET_WEAPON_HEAD_M2006;
                //                DJI_3508.outerTarget=GET_WEAPON_HEAD_3508;
                break;
            }
            break;
        case WAIT_FOR_COMBINE:
            switch (actionFlag)
            {
            case ENDING_COMBINE: // 受控才可结束
                actionCpltFlag = 0;
                break;
            default:
                airOperator.airOperatorTxBuf[0] = 0; // 夹爪
                airOperator.airOperatorTxBuf[1] = 0; // 伸缩臂气泵
                airOperator.airOperatorTxBuf[2] = 0; // 储存气泵

                Gimbal_J60_CMD.position_ = READY_TO_GET_STRAIGHT_KFS_J60;
                Motor_A1.TargetPos = READY_TO_GET_KFS_A1;
                stretch_2006.outerTarget = READY_TO_GET_KFS_S2006;
                stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM;

                // move_2006.outerTarget = WAIT_FOR_COMBINE_M2006;
                DJI_3508.outerTarget = WAIT_FOR_COMBINE_3508;
                break;
            }
            break;
        case READY_TO_GET_KFS:
            airOperator.airOperatorTxBuf[0] = 0; // 夹爪
            airOperator.airOperatorTxBuf[1] = 0; // 伸缩臂气泵
            airOperator.airOperatorTxBuf[2] = 0; // 储存气泵

            Gimbal_J60_CMD.position_ = READY_TO_GET_STRAIGHT_KFS_J60;
            Motor_A1.TargetPos = READY_TO_GET_KFS_A1;
            stretch_2006.outerTarget = READY_TO_GET_KFS_S2006;
            stretch_DM.ctrl.pos_set = READY_TO_GET_KFS_DM;

            // move_2006.outerTarget = STANDBY_M2006;
            DJI_3508.outerTarget = STANDBY_3508;
            break;
        case HOLDING_KFS:
            switch (actionFlag)
            {
            case LEAVE_KFS_BACK:
            case LEAVE_KFS_FRONT:
            case STORAGE_KFS:
            case USE_KFS:
                actionCpltFlag = 0;
                break;
            default:
                airOperator.airOperatorTxBuf[0] = 0;
                airOperator.airOperatorTxBuf[1] = 1;

                Gimbal_J60_CMD.position_ = HOLD_KFS_J60;
                Motor_A1.TargetPos = HOLD_KFS_A1;
                stretch_2006.outerTarget = HOLD_KFS_S2006;
                stretch_DM.ctrl.pos_set = HOLD_KFS_DM;

                //                move_2006.outerTarget=STANDBY_M2006;
                //                DJI_3508.outerTarget=STANDBY_3508;
                break;
            }
            break;
        default:
            break;
        }
    }
}
