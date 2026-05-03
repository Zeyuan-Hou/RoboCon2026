#include "withinBoardCommunication.h"
#include "ROBOT.h"
#include "string.h"

uint32_t communication_tim = 0;

void PackDataToZGT(void)
{
    for (uint8_t i = 0; i < 12; i++)
    {
        TxBufToZGT[i] = 0;
    }
    TxBufToZGT[0] = 0xBB;
    TxBufToZGT[5] = 0xAA;
    
    
    // 1 0初始化 1取头成功 2取头失败
    // 2 0初始化 1正在执行 2执行完成
    // 3 0初始化 1正在放块 2放块完成
    // 4 0初始化 1正在取块 2放块完成
    
    // TxBufToZGT[2] = actionFlag;
    // TxBufToZGT[3] = stateFlag;

    // // 避免收到新动作时，actionCpltFlag仍为1，使前几帧传输错误的动作完成状态
    // if (actionCpltFlag == 1)
    // {
    //     communication_tim++;
    //     if (communication_tim > 3)
    //     {
    //         TxBufToZGT[4] = actionFlag * 10;
    //     }
    // }
    // else
    // {
    //     TxBufToZGT[4] = actionFlag;
    //     communication_tim = 0;
    // }

    // //		TxBufToZGT[4]=actionCpltFlag;
    // TxBufToZGT[5] = chassisMoveFlag;
    // TxBufToZGT[6] = KFS_height;
    // TxBufToZGT[7] = KFS_orientation;
    // TxBufToZGT[8] = KFS_level;
    // TxBufToZGT[9] = use_KFS_orientation;
    // //		TxBufToZGT[10]=noWeapon;
}
void UnpackDataFromZGT(void)
{
    if (RxBufFromZGT[0] == 0xAA && RxBufFromZGT[5] == 0xBB)
    {

        // 1 任务 255执行完进入保持状态只有0才能退出 254用于接收新状态的空状态无法接收0
        //        0初始化 1取头进入255 2准备对接 3对接完成松夹爪再闭合
        //        4取块进入255
        //        5中层放块进入255 6取背后存的方块 7高层放块进入255 8取r1方块
        // 2 取块高度 1低200 2高200 3高400
        // 3 取块朝向 1左 2中 3右
        // 4 取块之后的动作 1向后扔块 2存块 3持有
        // actionFlag = (Action)RxBufFromZGT[2];
        // completeActionFlag = RxBufFromZGT[3];
        // KFS_height = RxBufFromZGT[4];
        // KFS_orientation = RxBufFromZGT[5];
        // KFS_level = RxBufFromZGT[6];
        // use_KFS_orientation = RxBufFromZGT[7];
        // init_mode = RxBufFromZGT[8];
        // system_monitor.cntMonitor.withinBoardCommunicate++;
    }
}
