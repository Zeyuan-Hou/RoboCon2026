#include "locate.h"

    /*******************************************************************************************
    函数名称：Robot_Location()
    函数功能：由双随动轮和陀螺仪数据得到机器人的坐标和姿态角（单位：mm，0.1°）
    输入：	  1.pstRobot 指向机器人总结构体的指针
              2.pstFW	 指向随动轮总结构体的指针
              3.pstGyro  指向陀螺总结构体的指针
    输出：	  1.机器人中心位姿，包括坐标和姿态角
              2.随动轮中心位姿	包括坐标
    备注：     1.此函数中的运算都是在弧度(rad)为单位的情况下进行的
              2.ALPHA_A为A随动轮逆时针旋转时线速度方向与机器人局部坐标系y正方向的夹角
              3.ALPHA_B为B随动轮逆时针旋转时线速度方向与机器人局部坐标系y正方向的夹角
              4.以上两个角度就是从y轴逆旋为正
    *******************************************************************************************/
void Robot_Location(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW)
{
    float ALPHA_B;             // B随动轮逆时针旋转时线速度方向与机器人局部坐标系y正方向的夹角 标定并赋值给宏定义ALPHA_A_Inc etc
    float ALPHA_A;             // A随动轮逆时针旋转时线速度方向与机器人局部坐标系y正方向的夹角
    float fpDeltaA, fpDeltaB;  // 随动轮走过的距离，单位：mm
    float Convert_Array[2][2]; // 距离转换矩阵，实际是3*3的，但是我们只用2*2
    float fpQ;                 // 机器人姿态角临时变量(单位：弧度)

    /*******************获取角度*********************/

    // pstRobot->stPot.fpPosQ// 航向角Q（单位：0.1度）

    // 随动轮中心姿态角是指机器人中心至随动轮中心的向量与全局坐标系Y轴的夹角
    Gyro_Data_Test.fpQ_Cur = gyro_data.yaw;
    if (fabs(Gyro_Data_Test.fpQ_Cur) > 1800)
    {
        Gyro_Data_Test.fpQ_Cur = Gyro_Data_Test.fpQ_Pre; // 如果突变超过180°时，说明有不正常的跳变，先给滤掉
    }
    if (fabs(fabs(Gyro_Data_Test.fpQ_Cur) - 1800) > 15) // 在接近180°的时候停止滤波，否则会把-180和180的跳变滤掉
    {
        if (fabs(Gyro_Data_Test.fpQ_Cur - Gyro_Data_Test.fpQ_Pre) > 100) // 说明陀螺仪存在突变
        {
            Gyro_Data_Test.fpQ_Cur = Gyro_Data_Test.fpQ_Pre;
        }
    }
    if (Gyro_Data_Test.fpQ_Cur - Gyro_Data_Test.fpQ_Pre > 1800) // 说明顺时针转过一圈了
        num_circle--;
    else if (Gyro_Data_Test.fpQ_Cur - Gyro_Data_Test.fpQ_Pre < -1800) // 说明逆时针转过一圈了
        num_circle++;
    fpSumPosQ = Gyro_Data_Test.fpQ_Cur + num_circle * 3600; // 转过的总角度

    pstRobot->stPos.fpPosQ = Gyro_Data_Test.fpQ_Cur;

    fpQ = ConvertAngle(pstRobot->stPos.fpPosQ * RADIAN_10); // 将姿态角从0.1度转换为弧度 ConvertAngle() 函数将角度限制在 [-π, π) 的范围内 避免出现360度以上或者负角度的情况。

    /**************获取随动轮方向位移****************/
    /*1. 数据采集：获取当前随动轮的编码器值*/
    pstFW->siCoderACur = degreeA;
    pstFW->siCoderBCur = degreeB;

    /*2. 异常检测：判断数据是否可信*/
    //  my_intabs() 是一个计算整数绝对值的函数，目的是比较当前编码器值和上一帧编码器值的差值。
    // 如果这个差值超过 1000，就认为数据可能异常（比如传感器抖动或者出现误差）
    // 正常情况下，2ms 内轮子的位移不可能超过 10 cm。
    if (fabs(pstFW->siCoderACur - pstFW->siCoderAPre) > 1000 ||
        fabs(pstFW->siCoderBCur - pstFW->siCoderBPre) > 1000) // 2ms 10cm不可能
    {
        pstFW->siCoderAPre = pstFW->siCoderACur;
        pstFW->siCoderBPre = pstFW->siCoderBCur;
        return;
        // 更新 siCoderAPre 和 siCoderBPre 为当前值。
        // 直接返回，不进行后续计算，确保异常数据不会影响系统。
    }

    /*3. 距离计算：随动轮走过的距离*/
    // 位移的计算逻辑分为两种情况：正转和反转
    // 因为随动轮的转动方向会影响编码器值的变化。
    if (pstFW->siCoderACur >= pstFW->siCoderAPre) // A轮正转，编码器值变大
    {
        fpDeltaA = (pstFW->siCoderACur - pstFW->siCoderAPre) * FW_Len_A_Inc;
        // 编码器值的变化量（pstFW->siCoderACur - pstFW->siCoderAPre）乘以 FW_Len_A_Inc（编码器单位到物理位移的转换系数）得到轮子走过的实际距离。
        ALPHA_A = ALPHA_A_Inc;
    }
    else // A轮反转
    {
        fpDeltaA = (pstFW->siCoderACur - pstFW->siCoderAPre) * FW_Len_A_Dec;
        ALPHA_A = ALPHA_A_Dec;
    }
    if (pstFW->siCoderBCur >= pstFW->siCoderBPre) // B轮正转,编码器值变大
    {
        fpDeltaB = (pstFW->siCoderBCur - pstFW->siCoderBPre) * FW_Len_B_Inc;
        ALPHA_B = ALPHA_B_Inc;
    }
    else // B轮反转
    {
        fpDeltaB = (pstFW->siCoderBCur - pstFW->siCoderBPre) * FW_Len_B_Dec;
        ALPHA_B = ALPHA_B_Dec;
    }

    /**************解算随动轮中心坐标****************/
    /*1. 矩阵分母系数计算：Sin_B_A*/
    // 两个随动轮的运动在某种程度上是耦合的，它们的相对夹角会影响整体坐标的解算。Sin_B_A 作为分母，用于描述随动轮之间的几何关系。

    //    /*2. 转换矩阵系数计算*/
    Convert_Array[0][0] = -sinf(ALPHA_A + fpQ - PI);
    Convert_Array[0][1] = -sinf(ALPHA_B + fpQ);
    Convert_Array[1][0] = cosf(ALPHA_A + fpQ - PI);
    Convert_Array[1][1] = cosf(ALPHA_B + fpQ);
    /*3. 随动轮中心坐标更新*/
    // pstFW->stPot.fpPosX 和 pstFW->stPot.fpPosY：随动轮中心在全局坐标系中的位置。
    pstFW->stPos.fpPosX += Convert_Array[0][0] * fpDeltaA + Convert_Array[0][1] * fpDeltaB;
    pstFW->stPos.fpPosY += Convert_Array[1][0] * fpDeltaA + Convert_Array[1][1] * fpDeltaB;

    /**************解算机器人中心坐标****************/
    // FW_rob_Alpha为机器人中心指向随动轮中心的矢量(同样要求从y轴开始逆时针旋转为正)，该角度范围为[0，2*pi)。【已知的常量，由随动轮的安装位置决定。】
    // FW_Rob_Len：表示机器人中心到随动轮中心的距离。

    pstRobot->stPos.fpPosX = fpStartX + pstFW->stPos.fpPosX - (-sinf(FW_rob_Alpha + fpQ) + sinf(FW_rob_Alpha)) * FW_Rob_Len;
    pstRobot->stPos.fpPosY = fpStartY + pstFW->stPos.fpPosY + (-cosf(FW_rob_Alpha + fpQ) + cosf(FW_rob_Alpha)) * FW_Rob_Len;

    pstRobot->stPos.fpPosX = pstRobot->stPos.fpPosX + fpPosXOffset; // 纠偏设置的为0，即初始化坐标为（0，0），全局坐标，局部坐标重合
    pstRobot->stPos.fpPosY = pstRobot->stPos.fpPosY + fpPosYOffset;

    /*随动轮编码器数据和陀螺仪数据保存*/
    pstFW->siCoderAPre = pstFW->siCoderACur;
    pstFW->siCoderBPre = pstFW->siCoderBCur;

    Gyro_Data_Test.fpQ_Pre = Gyro_Data_Test.fpQ_Cur;
}

/*******************************************************************************************
函数名称：DT35_relocation_new()
函数功能：由DT35数据得到机器人的坐标和姿态角（单位：mm，0.1°）
输入：	  1.pstRobot    指向机器人总结构体的指针
          2.pstFW	    指向随动轮总结构体的指针
          3.p_dt35_save 指向上一刻DT35总结构体的指针
          4.p_dt35_now  指向当前DT35总结构体的指针
输出：	  1.每个dt35到墙的距离
          2.由dt35解算出的机器人中心位姿，包括坐标和姿态角
备注：    1.此函数中的运算都是在弧度为单位的情况下进行的
          2.K_DT35_Y1，B_DT35_Y1，K_DT35_Y2，B_DT35_Y2，K_DT35_Y3，B_DT35_Y3，K_DT35_Y4，B_DT35_Y4
            是DT35得到电压模拟量线性映射解算距离的系数 需测定数据进行matlab拟合计算
*******************************************************************************************/
void DT35_relocation_new(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW, ST_DT35 *p_dt35_save, ST_DT35 *p_dt35_now)
{
    float fpQ;
    p_dt35_now->robot_q = pstRobot->stPos.fpPosQ;        // 单位0.1°
    fpQ = ConvertAngle(p_dt35_now->robot_q * RADIAN_10); // 弧度

    // 利用电压模拟量线性映射计算dt35到墙的距离
    p_dt35_now->dt35_x1 = fabs(K_DT35_X1 * p_dt35_now->dt35_voltage_x1 + B_DT35_X1);
    p_dt35_now->dt35_x2 = fabs(K_DT35_X2 * p_dt35_now->dt35_voltage_x2 + B_DT35_X2);
    p_dt35_now->dt35_y1 = fabs(K_DT35_Y1 * p_dt35_now->dt35_voltage_y1 + B_DT35_Y1);
    p_dt35_now->dt35_y2 = fabs(K_DT35_Y2 * p_dt35_now->dt35_voltage_y2 + B_DT35_Y2);

    /*以下需根据场地实测来决策使用哪两个dt35进行定位*/
    // dt35所在边的中心到墙面的X或Y方向的距离
    p_dt35_now->robot_x = (p_dt35_now->dt35_x1) * cos(fpQ);
    p_dt35_now->robot_y = (p_dt35_now->dt35_y2) * cos(fpQ);

    /******更新随动轮目标位置************/
    //		if(pstRobot->stPos)
    //		{
    stFollowerWheel.stPos.fpPosX = p_dt35_now->robot_x - (-cosf(FW_rob_Alpha + fpQ) + cosf(FW_rob_Alpha)) * FW_Rob_Len;
    stFollowerWheel.stPos.fpPosY = p_dt35_now->robot_y - (-sinf(FW_rob_Alpha + fpQ) + sinf(FW_rob_Alpha)) * FW_Rob_Len;
    //		}
}

void dt35_relocation(void)
{
    DT35_NEW.x_sucpect = 0;
    DT35_NEW.y_sucpect = 0;

    // 车身朝向不在正直角上，舍去这一次的值
    if (fabs(stRobot.stPos.fpPosQ) < 20)
        DT35_NEW.dt35_angle = DEG_0;
    else if (fabs(stRobot.stPos.fpPosQ - 900) < 20)
        DT35_NEW.dt35_angle = DEG_90;
    else if (fabs(stRobot.stPos.fpPosQ - 1800) < 20 || fabs(stRobot.stPos.fpPosQ + 1800) < 2)
        DT35_NEW.dt35_angle = DEG_180;
    else if (fabs(stRobot.stPos.fpPosQ + 900) < 20)
        DT35_NEW.dt35_angle = DEG_270;
    else
    {
        DT35_NEW.x_sucpect = 1;
        DT35_NEW.y_sucpect = 1;
    }

    // 在某些方向上超过了极限距离，舍去该方向DT35值
    if (DT35_NEW.dt35_angle == DEG_0 || DT35_NEW.dt35_angle == DEG_180)
    {
        if (DT35_NEW.dt35_x1 > 8000 || DT35_NEW.dt35_x1 < 50 || DT35_NEW.dt35_x2 > 8000 || DT35_NEW.dt35_x2 < 50)
            DT35_NEW.x_sucpect = 1;
        if (DT35_NEW.dt35_y1 > 9500 || DT35_NEW.dt35_y1 < 50 || DT35_NEW.dt35_y2 > 9500 || DT35_NEW.dt35_y2 < 50)
            DT35_NEW.y_sucpect = 1;

        if ((DT35_NEW.dt35_y1 + DT35_NEW.dt35_y2) < 14800 || (DT35_NEW.dt35_y1 + DT35_NEW.dt35_y2) > 15200)
            DT35_NEW.y_sucpect = 1;
    }
    if (DT35_NEW.dt35_angle == DEG_90 || DT35_NEW.dt35_angle == DEG_270)
    {
        if (DT35_NEW.dt35_x1 > 9500 || DT35_NEW.dt35_x1 < 50 || DT35_NEW.dt35_x2 > 9500 || DT35_NEW.dt35_x2 < 50)
            DT35_NEW.x_sucpect = 1;
        if (DT35_NEW.dt35_y1 > 8000 || DT35_NEW.dt35_y1 < 50 || DT35_NEW.dt35_y2 > 8000 || DT35_NEW.dt35_y2 < 50)
            DT35_NEW.y_sucpect = 1;

        if ((DT35_NEW.dt35_y1 + DT35_NEW.dt35_y2) < 7800 || (DT35_NEW.dt35_y1 + DT35_NEW.dt35_y2) > 8200)
            DT35_NEW.y_sucpect = 1;
    }

    // 同方向的两DT35差距太大，舍去该方向的值
    // if (fabs(DT35_NEW.dt35_x1 - DT35_NEW.dt35_x2) > 50)
    //     DT35_NEW.x_sucpect = 1;

    // 每个DT35的连续值发生了剧烈跳变，则舍去该方向DT35值
    DT35_NEW.dt35_x1_save[0] = DT35_NEW.dt35_x1_save[1];
    DT35_NEW.dt35_x1_save[1] = DT35_NEW.dt35_x1_save[2];
    DT35_NEW.dt35_x1_save[2] = DT35_NEW.dt35_x1_save[3];
    DT35_NEW.dt35_x1_save[3] = DT35_NEW.dt35_x1_save[4];
    DT35_NEW.dt35_x1_save[4] = DT35_NEW.dt35_x1;

    for (int i = 0; i < 5; i++)
    {
        DT35_NEW.dt35_x1_save_sum += DT35_NEW.dt35_x1_save[i];
    }
    for (int i = 0; i < 5; i++)
    {
        if (fabs(DT35_NEW.dt35_x1_save[i] - DT35_NEW.dt35_x1_save_sum / 5.f) > 30)
            DT35_NEW.x_sucpect = 1;
    }
    DT35_NEW.dt35_x1_save_sum = 0;

    DT35_NEW.dt35_x2_save[0] = DT35_NEW.dt35_x2_save[1];
    DT35_NEW.dt35_x2_save[1] = DT35_NEW.dt35_x2_save[2];
    DT35_NEW.dt35_x2_save[2] = DT35_NEW.dt35_x2_save[3];
    DT35_NEW.dt35_x2_save[3] = DT35_NEW.dt35_x2_save[4];
    DT35_NEW.dt35_x2_save[4] = DT35_NEW.dt35_x2;

    for (int i = 0; i < 5; i++)
    {
        DT35_NEW.dt35_x2_save_sum += DT35_NEW.dt35_x2_save[i];
    }
    for (int i = 0; i < 5; i++)
    {
        if (fabs(DT35_NEW.dt35_x2_save[i] - DT35_NEW.dt35_x2_save_sum / 5.f) > 30)
            DT35_NEW.x_sucpect = 1;
    }

    DT35_NEW.dt35_x2_save_sum = 0;

    DT35_NEW.dt35_y1_save[0] = DT35_NEW.dt35_y1_save[1];
    DT35_NEW.dt35_y1_save[1] = DT35_NEW.dt35_y1_save[2];
    DT35_NEW.dt35_y1_save[2] = DT35_NEW.dt35_y1_save[3];
    DT35_NEW.dt35_y1_save[3] = DT35_NEW.dt35_y1_save[4];
    DT35_NEW.dt35_y1_save[4] = DT35_NEW.dt35_y1;

    for (int i = 0; i < 5; i++)
    {
        DT35_NEW.dt35_y1_save_sum += DT35_NEW.dt35_y1_save[i];
    }
    for (int i = 0; i < 5; i++)
    {
        if (fabs(DT35_NEW.dt35_y1_save[i] - DT35_NEW.dt35_y1_save_sum / 5.f) > 30)
            DT35_NEW.y_sucpect = 1;
    }
    DT35_NEW.dt35_y1_save_sum = 0;

    DT35_NEW.dt35_y2_save[0] = DT35_NEW.dt35_y2_save[1];
    DT35_NEW.dt35_y2_save[1] = DT35_NEW.dt35_y2_save[2];
    DT35_NEW.dt35_y2_save[2] = DT35_NEW.dt35_y2_save[3];
    DT35_NEW.dt35_y2_save[3] = DT35_NEW.dt35_y2_save[4];
    DT35_NEW.dt35_y2_save[4] = DT35_NEW.dt35_y2;

    for (int i = 0; i < 5; i++)
    {
        DT35_NEW.dt35_y2_save_sum += DT35_NEW.dt35_y2_save[i];
    }
    for (int i = 0; i < 5; i++)
    {
        if (fabs(DT35_NEW.dt35_y2_save[i] - DT35_NEW.dt35_y2_save_sum / 5.f) > 30)
            DT35_NEW.y_sucpect = 1;
    }

    DT35_NEW.dt35_y2_save_sum = 0;

    float NEW_x_total = 0;
    float NEW_y1_total = 0;
    float NEW_y2_total = 0;
    if (DT35_NEW.x_sucpect == 0) // 在上述判断下仍然通过了检测，就开始进行赋值
    {
        NEW_x_total = (DT35_NEW.dt35_x1 + DT35_NEW.dt35_x2) * 0.5f;
        switch (DT35_NEW.dt35_angle)
        {
        case DEG_0:
            DT35_NEW.dt35_robot_x = FIELD_WIDTH - NEW_x_total;
            stRobot.stPos.fpPosX = DT35_NEW.dt35_robot_x;
            break;
        case DEG_90:
            DT35_NEW.dt35_robot_y = FIELD_HEIGHT - NEW_x_total;
            stRobot.stPos.fpPosY = DT35_NEW.dt35_robot_y;
            break;
        case DEG_180:
            DT35_NEW.dt35_robot_x = NEW_x_total;
            stRobot.stPos.fpPosX = DT35_NEW.dt35_robot_x;
            break;
        case DEG_270:
            DT35_NEW.dt35_robot_y = NEW_x_total;
            stRobot.stPos.fpPosY = DT35_NEW.dt35_robot_y;
            break;
        }
    }
    if (DT35_NEW.y_sucpect == 0)
    {
        switch (DT35_NEW.dt35_angle)
        {
        case DEG_0:
            break;
        case DEG_90:
            NEW_y1_total = DT35_NEW.dt35_y1;
            NEW_y2_total = FIELD_WIDTH - DT35_NEW.dt35_y2;
            if (fabs(NEW_y1_total - NEW_y2_total) < 200)
            {
                DT35_NEW.dt35_robot_x = (NEW_y1_total + NEW_y2_total) * 0.5f;
            }
            stRobot.stPos.fpPosX = DT35_NEW.dt35_robot_x;
            break;
        case DEG_180:
            break;
        case DEG_270:
            NEW_y1_total = FIELD_WIDTH - DT35_NEW.dt35_y1;
            NEW_y2_total = DT35_NEW.dt35_y2;
            if (fabs(NEW_y1_total - NEW_y2_total) < 200)
            {
                DT35_NEW.dt35_robot_x = (NEW_y1_total + NEW_y2_total) * 0.5f;
            }
            stRobot.stPos.fpPosX = DT35_NEW.dt35_robot_x;
            break;
        }
    }
}

/*-------------------------------------------------------------------------------------------------
函数功能：轮子速度逆解算成车体的速度，滤波后存在Wheelvelt_To_Bodyvelt结构体里
全向轮解算
-------------------------------------------------------------------------------------------------*/
void WheelveltToBodyvelt(void)
{
    Wheelvelt_To_Bodyvelt.W.in = (leftup_motor.anglev + rightup_motor.anglev + rightdown_motor.anglev + leftdown_motor.anglev) * R_WHEEL / (4 * RUN_GEAR_RATIO * R_ROBOT * RADIAN);
    Wheelvelt_To_Bodyvelt.Vx.in = (-leftup_motor.anglev - rightup_motor.anglev + rightdown_motor.anglev + leftdown_motor.anglev) * R_WHEEL / (4 * sin(PI / 4) * RUN_GEAR_RATIO);
    Wheelvelt_To_Bodyvelt.Vy.in = (-leftup_motor.anglev + rightup_motor.anglev + rightdown_motor.anglev - leftdown_motor.anglev) * R_WHEEL / (4 * sin(PI / 4) * RUN_GEAR_RATIO);
    LpFilter(&Wheelvelt_To_Bodyvelt.W);
    LpFilter(&Wheelvelt_To_Bodyvelt.Vx);
    LpFilter(&Wheelvelt_To_Bodyvelt.Vy);
}

void PositionToVelt(void)
{
    // 通过TD计算车身的速度
    posX_veltX.aim = stRobot.stPos.fpPosX;
    posY_veltY.aim = stRobot.stPos.fpPosY;
    posW_veltW.aim = fpSumPosQ / 10;
    CalTD(&posX_veltX);
    CalTD(&posY_veltY);
    CalTD(&posW_veltW);
    // 引入低通滤波来滤除惯导系统微分的噪声
    global_velt_filter.global_vx.in = posX_veltX.x2;
    global_velt_filter.global_vy.in = posY_veltY.x2;
    global_velt_filter.global_w.in = posW_veltW.x2;
    LpFilter(&global_velt_filter.global_vx);
    LpFilter(&global_velt_filter.global_vy);
    LpFilter(&global_velt_filter.global_w);
    stRobot.stVelt_global.fpVx = global_velt_filter.global_vx.out;
    stRobot.stVelt_global.fpVy = global_velt_filter.global_vy.out;
    stRobot.stVelt_global.fpW = global_velt_filter.global_w.out;
    // 全局坐标系的速度转换为车身局部坐标系的速度
    float fpQ = ConvertAngle(stRobot.stPos.fpPosQ * RADIAN_10);
    float cosQ = cosf(fpQ);
    float sinQ = sinf(fpQ);
    stRobot.stVelt_local.fpVx = stRobot.stVelt_global.fpVx * cosQ + stRobot.stVelt_global.fpVy * sinQ;
    stRobot.stVelt_local.fpVy = -stRobot.stVelt_global.fpVx * sinQ + stRobot.stVelt_global.fpVy * cosQ;
    stRobot.stVelt_local.fpW = stRobot.stVelt_global.fpW;
    // 轮子速度也转换成车身局部速度
    WheelveltToBodyvelt();
    // 不同方式求出来的车身速度进行卡尔曼滤波
    KalmanUpdate(&KF_Vx, stRobot.stVelt_local.fpVx, Wheelvelt_To_Bodyvelt.Vx.out);
    KalmanUpdate(&KF_Vy, stRobot.stVelt_local.fpVy, Wheelvelt_To_Bodyvelt.Vy.out);
    KalmanUpdate(&KF_W, stRobot.stVelt_local.fpW, Wheelvelt_To_Bodyvelt.W.out);
}

/*******************************************************************************************
函数名称：UpdatePositionFeedback()
函数功能：将定位得到的机器人当前x y坐标和姿态角更新至导航反馈中
输入：	  1.p_nav    		指向导航总结构体的指针
          2.pstRobot	    指向机器人总结构体的指针
输出：
备注：    正负号可能需要纠正更改

*******************************************************************************************/
void UpdatePositionFeedback(ST_Nav *p_nav, ST_ROBOT *pstRobot)
{

    p_nav->auto_path.pos_pid.x.fpFB = pstRobot->stPos.fpPosX;      // 更新 X 坐标反馈
    p_nav->auto_path.pos_pid.y.fpFB = pstRobot->stPos.fpPosY;      // 更新 Y 坐标反馈
    p_nav->auto_path.pos_pid.w.fpFB = pstRobot->stPos.fpPosQ / 10; // 更新姿态角反馈 反馈单位为°
}
