#include "remote_control.h"

uint8_t UART2_Rx_Buf[9]={0};
uint8_t UART2_Tx_Buf[21]={0};


/*******************************************************************************************
函数名称：parseDataPacket
函数功能：解析来自遥控器的数据包，并将解析后的数据填充到遥控器结构体ST_JS_VALUE中。
          数据包包含摇杆的位置和按键（36合一的矩阵键盘按键和8个独立按键）信息。
输入：   1. Tx_Buf （SPI通讯的存储数组）指向包含32个元素的uint8_t数组，每个元素代表一个字节的数据。
输出：   1. jsValue 指向ST_JS_VALUE结构体的指针，用于存储解析后的摇杆值和按键控制信息。
备注：   1. 数据包的前8个元素用于存储ADC采样值，其中0, 1: 左摇杆X方向；2, 3: 左摇杆Y方向；4, 5: 右摇杆X方向；6, 7: 右摇杆Y方向。
         2. 数据包的第8个元素表示矩阵键盘的值（范围0到36），第9-16个元素表示独立按键的值。
         3. ADC采样值是12位的，需要从两个连续的字节中提取高8位和低8位来重组完整的采样值。
         4. 解析过程中会考虑数据包中的所有相关信息，并将其适当地映射到目标结构体中。
*******************************************************************************************/
void parseDataPacket(const uint8_t *Rx_Buf, ST_JS_VALUE *jsValue)
{
    jsValue->usJsLeft_X = (Rx_Buf[0] << 8) | Rx_Buf[1];  // 左摇杆X值
    jsValue->usJsLeft_Y = (Rx_Buf[2] << 8) | Rx_Buf[3];  // 左摇杆Y值
    jsValue->usJsRight_X = (Rx_Buf[4] << 8) | Rx_Buf[5]; // 右摇杆X值
    jsValue->usJsRight_Y = (Rx_Buf[6] << 8) | Rx_Buf[7]; // 右摇杆Y值

    // 按键赋值
    // jsValue->usJsKey = Rx_Buf[8];

    // 只接收一次按键值，防止状态被不断更新
    // 缺点是不能处理连续按两次同一按键的情况
    if (Rx_Buf[8] == RC_Key_pre_value)
    {
        jsValue->usJsKey = 0;
    }
    else
    {
        jsValue->usJsKey = Rx_Buf[8];
        RC_Key_pre_value = jsValue->usJsKey;
    }
}

/*******************************************************************************************
函数名称：CalculateVelocities
函数功能：基于手柄摇杆值计算机器人的线速度（X和Y方向）及角速度，并应用低通滤波器进行平滑处理。
输入：   1. jsValue 指向包含手柄摇杆读数的ST_JS_VALUE结构体指针。
          2. velocities 指向用于存储计算得到的速度值的ST_VELT结构体指针。
          3. ucGateX X方向上的阈值，只有超过此阈值才产生运动。
          4. ssXSpedLimit X方向上的最大速度限制（单位mm/s）。
          5. ucGateY Y方向上的阈值，只有超过此阈值才产生运动。
          6. ssYSpedLimit Y方向上的最大速度限制（单位mm/s）。
          7. ucGateW 角速度方向上的阈值，只有超过此阈值才产生旋转。
          8. ssWSpedLimit 角速度方向上的最大速度限制（单位0.1度/s）。
输出：   1. velocities 结构体中的成员变量被更新为计算得到的X方向速度（fpVx）、Y方向速度（fpVy）和角速度（fpW）。
备注：     1. 计算过程首先检查是否超过了设定的阈值，如果超过，则按照比例缩放至最大速度限制。
          2. 使用一阶低通滤波器对速度值进行平滑处理，以减少由于手柄输入抖动或用户操作不精确带来的快速加减速。
          3. 注意坐标系转换：左摇杆X轴对应的是fpVx，左摇杆Y轴对应的是fpVy,但要加一个负号，因为摇杆值Y方向从上到下是从0到4096，X方向从右到左是0到4096。
          4. 右摇杆X控制角速度，其值经过特定的比例缩放后赋给fpW。
*******************************************************************************************/
void CalculateVelocities(const ST_JS_VALUE *jsValue, ST_Nav *p_nav,
                         uint16_t ucGateX, int16_t ssXSpedLimit,
                         uint16_t ucGateY, int16_t ssYSpedLimit,
                         uint16_t ucGateW, int16_t ssWSpedLimit)
{
    const float smooth = 20;                       // 平滑等级，一阶低通滤波器截止频率
    static ST_LPF FJx = {0, 0, 0, smooth, 0.002f}; // 需要pre_out，故需要设为静态变量
    static ST_LPF FJy = {0, 0, 0, smooth, 0.002f};
    static ST_LPF FJw = {0, 0, 0, smooth, 0.002f};

    // 计算左摇杆X方向的速度 (fpVy)
    float Vx = jsValue->usJsLeft_X - LEFT_JS_X_MID;
    if (fabs(Vx) < ucGateX)
    {
        FJx.in = 0;
    }
    else
    {
        if (Vx > 0)
        {
            FJx.in = (Vx - ucGateX) * ssXSpedLimit / (LEFT_JS_X_MAX - LEFT_JS_X_MID);
        }
        else if (Vx < 0)
        {
            FJx.in = (Vx + ucGateX) * ssXSpedLimit / (LEFT_JS_X_MID - LEFT_JS_X_MIN);
        }
    }

    // 计算左摇杆Y方向的速度 (fpVx)

    float Vy = jsValue->usJsLeft_Y - LEFT_JS_Y_MID;
    if (fabs(Vy) < ucGateY)
    {
        FJy.in = 0;
    }
    else
    {
        if (Vy > 0)
        {
            // Y轴正向偏移
            FJy.in = (Vy - ucGateY) * ssYSpedLimit / (LEFT_JS_Y_MAX - LEFT_JS_Y_MID);
        }
        else if (Vy < 0)
        {
            // Y轴负向偏移
            FJy.in = (Vy + ucGateY) * ssYSpedLimit / (LEFT_JS_Y_MID - LEFT_JS_Y_MIN);
        }
    }

    // 计算右摇杆X方向的角速度 (fpW)
    float Vw = jsValue->usJsRight_X - RIGHT_JS_MID;
    if (fabs(Vw) < ucGateW)
    {
        FJw.in = 0; // 在死区内，设置为0
    }
    else
    {
        if (Vw > 0)
        {
            // X轴正向偏移（假设向右为正）
            FJw.in = (Vw - ucGateW) * ssWSpedLimit / (RIGHT_JS_MAX - RIGHT_JS_MID);
        }
        else if (Vw < 0)
        {
            // X轴负向偏移（假设向左为负）
            FJw.in = (Vw + ucGateW) * ssWSpedLimit / (RIGHT_JS_MID - RIGHT_JS_MIN);
        }
    }
    // 应用低通滤波
    LpFilter(&FJx);
    LpFilter(&FJy);
    LpFilter(&FJw);

    p_nav->auto_path.basic_velt.fpVx = FJx.out;  // 摇杆值X方向从右到左是从0到4096。
    p_nav->auto_path.basic_velt.fpVy = -FJy.out; // 摇杆值Y方向从上到下是从0到4096。
    p_nav->auto_path.basic_velt.fpW = -FJw.out;  //  右摇杆X控制角速度
}

void Deal_Key_State(const ST_JS_VALUE *jsValue)
{
    switch (jsValue->usJsKey)
    {
    case 1: // 状态机清零
        part_over_flag = 40;
        break;

    case 2: // 小脚和轮子初始化
        part_over_flag = 0;
        // nav.nav_state = CHASSIS_INIT;
        break;
    case 3:
        nav.nav_state = RC_LOCAL;
        break;
    case 4:
        nav.nav_state = RC_GLOBAL;
        break;

    case 5:
        foot.foot_state = FOOT_CLEAR;
        break;
    case 6:
        foot.foot_state = FOOT_INIT;
        break;
    case 7:
        foot.foot_state = FOOT_UP_PREPARE; // 一键上台阶
        break;
    case 8:
        foot.foot_state = FOOT_DOWN_PREPARE; // 一键下台阶
        break;

    case 9:
        foot.foot_state = FOOT_TEST_STAND_PREPARE; // 单独站的前置动作
        break;

    case 10:
        foot.foot_state = FOOT_TEST_STAND; // 单独站
        break;

    case 11:
        foot.foot_state = FOOT_TEST_SIT; // 单独坐
        break;

    case 12:
        up_d_state = D_READY_GET_HEAD;   // 准备取武器头  2
        if (flag_D_ready_get_head == 10) // 准备完成
        {
            up_d_state = D_GET_LEFT; // 取武器头3
        }
        break;
    case 13: // 直接从二区开始初始化大小臂
        part_over_flag = 20;
        break;
    case 14: // 直接从二区10开始并进行三区任务
        part_over_flag = 30;
        break;
    case 15: // 直接从二区12开始并进行三区任务
        part_over_flag = 31;
        break;
    case 16: // 上完R1后放方块
        //					part_over_flag=6;
        up_s_state = S_PUT_BLOCK_TOP;
        break;

    case 17: // 取上层方块
        up_s_state = S_GET_BLOCK_UP;
        break;

    case 18: // 取下层方块
        up_s_state = S_GET_BLOCK_DOWN;
        break;

    case 19: // 给双臂方块
        up_s_state = S_GIVE_D;
        break;
    case 20: // 放中层方块
        up_s_state = S_PUT_BLOCK_MIDDLE;
        break;
    case 21: // 一区状态机完成后停止
        part_over_flag = 1;
        break;

    case 22: // 从武器架到二区状态机
        part_over_flag = 2;
        break;
        //				up_s_state=S_PUT_BLOCK_MIDDLE;
        //				case 23://二区状态机
        //						part_over_flag = 3;
        //						break;

    case 23: // 三区状态机
        part_over_flag = 5;
        break;

    default:
        break;
    }
}
void Send_To_RemoteControl()
{
    uint8_t ZGT_monitor;

    if (monitor.system_error.can1_rd && monitor.system_error.can1_rd_turn && monitor.system_error.can1_ld && monitor.system_error.can1_ld_turn && monitor.system_error.can2_j60 && monitor.system_error.can2_lu && monitor.system_error.can2_lu_turn &&
        monitor.system_error.can2_ru && monitor.system_error.can2_ru_turn && monitor.system_error.go1_left && monitor.system_error.go1_right && monitor.system_error.can1_dt35 && monitor.system_error.can1_travelSwitch && monitor.system_error.rc && monitor.system_error.vofa && monitor.system_error.task1 && monitor.system_error.task2 && monitor.system_error.task3 && monitor.system_error.task4 && monitor.system_error.task5 && monitor.system_error.task6 && monitor.system_error.VISION_REC && monitor.system_error.VISION_TX && monitor.system_error.UPPER_TO_LOWER && monitor.system_error.LOWER_TO_UPPER)
    {
        ZGT_monitor = 1;
    }
    else
    {
        ZGT_monitor = 0; // 无错误
    }

    nRF24L01_ack_pay.Ack_Buf[0] = ZGT_monitor;

    nRF24L01_ack_pay.Ack_Buf[1] = (int16_t)stRobot.stPos.fpPosX >> 8;
    nRF24L01_ack_pay.Ack_Buf[2] = (int16_t)stRobot.stPos.fpPosX & 0xFF;
    nRF24L01_ack_pay.Ack_Buf[3] = (int16_t)stRobot.stPos.fpPosY >> 8;
    nRF24L01_ack_pay.Ack_Buf[4] = (int16_t)stRobot.stPos.fpPosY & 0xFF;
    nRF24L01_ack_pay.Ack_Buf[5] = (int16_t)(stRobot.stPos.fpPosQ / 10.f) >> 8;
    nRF24L01_ack_pay.Ack_Buf[6] = (int16_t)(stRobot.stPos.fpPosQ / 10.f) & 0xFF;

    nRF24L01_ack_pay.Ack_Buf[7] = nav.nav_state;                    // nav:
    nRF24L01_ack_pay.Ack_Buf[8] = nav.auto_path.number_permutation; // path:
    nRF24L01_ack_pay.Ack_Buf[9] = nav.auto_path.number_point;       // point:
    nRF24L01_ack_pay.Ack_Buf[10] = nav.auto_path.get_block;         // block:
    nRF24L01_ack_pay.Ack_Buf[11] = part_over_flag;                  // flag:

    nRF24L01_ack_pay.Ack_Buf[12] = foot.foot_state; // foot:

    nRF24L01_ack_pay.Ack_Buf[13] = up_d_state; // d_arm:
    nRF24L01_ack_pay.Ack_Buf[14] = up_s_state; // s_arm:

    nRF24L01_ack_pay.Ack_Buf[15] = vision_data_recieve.id;        // id:
    nRF24L01_ack_pay.Ack_Buf[16] = vision_data_recieve.next_id;   // next_id:
    nRF24L01_ack_pay.Ack_Buf[17] = vision_data_recieve.extra_id1; // ex_id1:
    nRF24L01_ack_pay.Ack_Buf[18] = vision_data_recieve.extra_id2; // ex_id2:
    nRF24L01_ack_pay.Ack_Buf[19] = vision_data_recieve.flag_123;  // flag123:
    nRF24L01_ack_pay.Ack_Buf[20] = vision_data_recieve.count_num; // num:
		
		HAL_UART_Transmit_DMA(&huart2, nRF24L01_ack_pay.Ack_Buf, 23); // ����

}
