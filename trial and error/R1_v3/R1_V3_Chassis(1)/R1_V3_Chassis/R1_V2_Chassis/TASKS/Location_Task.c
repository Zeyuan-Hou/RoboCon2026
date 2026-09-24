#include "Location_Task.h"


/*******************************************************************************************
函数名称：Follower_Wheel_Location()
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
 void Follower_Wheel_Location(ST_ROBOT *pstRobot, ST_FOLLOWER_WHEEL *pstFW)
{
    fp32 ALPHA_B;             // B随动轮逆时针旋转时线速度方向与机器人局部坐标系y正方向的夹角 标定并赋值给宏定义ALPHA_A_Inc etc
    fp32 ALPHA_A;             // A随动轮逆时针旋转时线速度方向与机器人局部坐标系y正方向的夹角
    fp32 fpDeltaA, fpDeltaB;  // 随动轮走过的距离，单位：mm
    fp32 Convert_Array[2][2]; // 距离转换矩阵，实际是3*3的，但是我们只用2*2
    fp32 fpQ;                 // 机器人姿态角临时变量(单位：弧度)


    /*******************获取角度*********************/

    // pstRobot->stPot.fpPosQ// 航向角Q（单位：0.1度）

    // 随动轮中心姿态角是指机器人中心至随动轮中心的向量与全局坐标系Y轴的夹角
	
		Gyro_Data_Test.fpQ_Cur = gyro_data.yaw;
		if(fabs(Gyro_Data_Test.fpQ_Cur)>1800)
		{
			Gyro_Data_Test.fpQ_Cur = Gyro_Data_Test.fpQ_Pre;//如果突变超过180°时，说明有不正常的跳变，先给滤掉
		}
		if(fabs( fabs(Gyro_Data_Test.fpQ_Cur) - 1800)>15)//在接近180°的时候停止滤波，否则会把-180和180的跳变滤掉
		{
			if(fabs(Gyro_Data_Test.fpQ_Cur - Gyro_Data_Test.fpQ_Pre)>100)//说明陀螺仪存在突变
			{	
				Gyro_Data_Test.fpQ_Cur = Gyro_Data_Test.fpQ_Pre;
			}
		}
		if(Gyro_Data_Test.fpQ_Cur - Gyro_Data_Test.fpQ_Pre>1800)//说明顺时针转过一圈了
			num_circle--;
		else if(Gyro_Data_Test.fpQ_Cur - Gyro_Data_Test.fpQ_Pre<-1800)//说明逆时针转过一圈了
			num_circle++;
//		fpSumPosQ = (Gyro_Data_Test.fpQ_Cur + num_circle*3600)*0.996758501666824328f;//转过的总角度
		
			fpSumPosQ = (Gyro_Data_Test.fpQ_Cur + num_circle*3600);//转过的总角度
		
		pstRobot->stPos.fpPosQ = fpSumPosQ-num_circle*3600;
		

		
    fpQ = ConvertAngle(pstRobot->stPos.fpPosQ * RADIAN_10);       // 将姿态角从0.1度转换为弧度 ConvertAngle() 函数将角度限制在 [-π, π) 的范围内 避免出现360度以上或者负角度的情况。

    /**************获取随动轮方向位移****************/
    /*1. 数据采集：获取当前随动轮的编码器值*/
    pstFW->siCoderACur = degreeA ;  // 使用相对值
    pstFW->siCoderBCur = degreeB ;

    /*2. 异常检测：判断数据是否可信*/
    //  my_intabs() 是一个计算整数绝对值的函数，目的是比较当前编码器值和上一帧编码器值的差值。
    // 如果这个差值超过 1000，就认为数据可能异常（比如传感器抖动或者出现误差）
    // 正常情况下，2ms 内轮子的位移不可能超过 10 cm。
    if (my_intabs(pstFW->siCoderACur - pstFW->siCoderAPre) > 1000 ||
        my_intabs(pstFW->siCoderBCur - pstFW->siCoderBPre) > 1000) // 2ms 10cm不可能
    {
        pstFW->siCoderAPre = pstFW->siCoderACur;
        pstFW->siCoderBPre = pstFW->siCoderBCur;
        return;
        // 更新 siCoderAPre 和 siCoderBPre 为当前值。
        //直接返回，不进行后续计算，确保异常数据不会影响系统。
    }

    /*3. 距离计算：随动轮走过的距离*/
    // 位移的计算逻辑分为两种情况：正转和反转
    //因为随动轮的转动方向会影响编码器值的变化。
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
		

	    //注意，这个时候是用纯随动轮定位覆盖了之前的坐标，所以这里的机器人中心坐标就是纯随动轮定位
			
			
		pstRobot->stPos.fpPosX = fpStartX + pstFW->stPos.fpPosX - (-sinf(FW_rob_Alpha + fpQ) + sinf(FW_rob_Alpha)) * FW_Rob_Len;
		pstRobot->stPos.fpPosY = fpStartY + pstFW->stPos.fpPosY + (-cosf(FW_rob_Alpha + fpQ) + cosf(FW_rob_Alpha)) * FW_Rob_Len;
		



        pstRobot->stPos.fpPosX = pstRobot->stPos.fpPosX + fpPosXOffset;
        pstRobot->stPos.fpPosY = pstRobot->stPos.fpPosY + fpPosYOffset;
		
    /*随动轮编码器数据和陀螺仪数据保存*/
    pstFW->siCoderAPre = pstFW->siCoderACur;
    pstFW->siCoderBPre = pstFW->siCoderBCur;
		
		Gyro_Data_Test.fpQ_Pre = Gyro_Data_Test.fpQ_Cur;
}



void UpdatePositionFeedback(ST_Nav *p_nav, ST_ROBOT *pstRobot)
{
   
    p_nav->auto_path.pos_pid.x.fpFB = pstRobot->stPos.fpPosX; // 更新 X 坐标反馈
    p_nav->auto_path.pos_pid.y.fpFB = pstRobot->stPos.fpPosY; // 更新 Y 坐标反馈
	
//	  //视觉直接读yaw
//	  p_nav->auto_path.pos_pid.w.fpFB = pstRobot->stPos.fpPosQ/10; // 更新角度反馈，单位为°
	
	  //陀螺仪读yaw要用这个
    p_nav->auto_path.pos_pid.w.fpFB = fpSumPosQ /10 ; // 更新姿态角反馈 反馈单位为°

}
/*-------------------------------------------------------------------------------------------------
函数功能：轮子速度逆解算成车体的速度，滤波后存在Wheelvelt_To_Bodyvelt结构体里
-------------------------------------------------------------------------------------------------*/
void WheelveltToBodyvelt(void)
{
	Wheelvelt_To_Bodyvelt.W.in=(leftup_motor.anglev+rightup_motor.anglev+rightdown_motor.anglev+leftdown_motor.anglev)*R_WHEEL/(4*RUN_GEAR_RATIO*R_ROBOT*RADIAN);
	Wheelvelt_To_Bodyvelt.Vx.in=(-leftup_motor.anglev-rightup_motor.anglev+rightdown_motor.anglev+leftdown_motor.anglev)*R_WHEEL/(4*sin(PI/4)*RUN_GEAR_RATIO);
	Wheelvelt_To_Bodyvelt.Vy.in=(-leftup_motor.anglev+rightup_motor.anglev+rightdown_motor.anglev-leftdown_motor.anglev)*R_WHEEL/(4*sin(PI/4)*RUN_GEAR_RATIO);
	
	LpFilter(&Wheelvelt_To_Bodyvelt.W);
	LpFilter(&Wheelvelt_To_Bodyvelt.Vx);
	LpFilter(&Wheelvelt_To_Bodyvelt.Vy);
	
}
void PositionToVelt(void)
{
		//通过TD计算车身的速度
		posX_veltX.m_aim = stRobot.stPos.fpPosX;
		posY_veltY.m_aim = stRobot.stPos.fpPosY;
		posW_veltW.m_aim = fpSumPosQ /10;
		CalTD(&posX_veltX);
		CalTD(&posY_veltY);
		CalTD(&posW_veltW);
		//引入低通滤波来滤除惯导系统微分的噪声
		global_velt_filter.global_vx.in = posX_veltX.m_x2;
		global_velt_filter.global_vy.in = posY_veltY.m_x2;
		global_velt_filter.global_w.in  = posW_veltW.m_x2;
	
		LpFilter(&global_velt_filter.global_vx);
		LpFilter(&global_velt_filter.global_vy);
		LpFilter(&global_velt_filter.global_w);
	
		stRobot.stVelt_global.fpVx = global_velt_filter.global_vx.out;
		stRobot.stVelt_global.fpVy = global_velt_filter.global_vy.out;
		stRobot.stVelt_global.fpW  = global_velt_filter.global_w.out;
	
		//全局坐标系的速度转换为车身局部坐标系的速度
		fp32 fpQ = ConvertAngle(stRobot.stPos.fpPosQ * RADIAN_10);
		fp32 cosQ = cosf(fpQ);
		fp32 sinQ = sinf(fpQ);
		stRobot.stVelt_local.fpVx =  stRobot.stVelt_global.fpVx * cosQ + stRobot.stVelt_global.fpVy * sinQ;
		stRobot.stVelt_local.fpVy = -stRobot.stVelt_global.fpVx * sinQ + stRobot.stVelt_global.fpVy * cosQ;
		stRobot.stVelt_local.fpW  = stRobot.stVelt_global.fpW;
		
		//轮子速度也转换成车身局部速度
		WheelveltToBodyvelt();
		//不同方式求出来的车身速度进行卡尔曼滤波
		KalmanUpdate(&KF_Vx,stRobot.stVelt_local.fpVx,Wheelvelt_To_Bodyvelt.Vx.out);
		KalmanUpdate(&KF_Vy,stRobot.stVelt_local.fpVy,Wheelvelt_To_Bodyvelt.Vy.out);
		KalmanUpdate(&KF_W,stRobot.stVelt_local.fpW,Wheelvelt_To_Bodyvelt.W.out);
}

/*****************************************new_part***********************************************************************************/
//根据随动轮反馈中心位置坐标算车速函数
void UpdateSpeedFeedback(ST_ROBOT *pstRobot) 
{
	td_Vx.m_aim = stRobot.stPos.fpPosX;
	td_Vy.m_aim = stRobot.stPos.fpPosY;
	
	CalTD(&td_Vx);
	CalTD(&td_Vy);
	
	lpf_Vx.in = td_Vx.m_x2;
	lpf_Vy.in = td_Vy.m_x2;
	
	LpFilter(&lpf_Vx);
	LpFilter(&lpf_Vy);
	
	pstRobot->stVelt_global.fpVx = lpf_Vx.out;
	pstRobot->stVelt_global.fpVy = lpf_Vy.out;
}


//判断DT35的值是否可信
void DT35_judge(void)
{
//	if((DT35_x<=10)||(DT35_x>=3500)||(DT35_X1_Count<25))
//	{DT35_xFlag = 0;}
//	else{DT35_xFlag = 1;}
//	
//	if((DT35_x_<=10)||(DT35_x_>=3500)||(DT35_Y1_Count<25))
//	{DT35_x_Flag = 0;}
//	else {DT35_x_Flag = 1;}
//	
//	if((DT35_y<=10)||(DT35_y>=3500)||(DT35_X2_Count<25))
//	{DT35_yFlag = 0;}
//	else {DT35_yFlag = 1;}
//	
//	if((DT35_y_<=10)||(DT35_y_>=3500)||(DT35_Y2_Count<25))
//	{DT35_y_Flag = 0;}
//	else{DT35_y_Flag = 1;}
	
		if((DT35_x<=10)||(DT35_x>=3500))
	{DT35_xFlag = 0;}
	else{DT35_xFlag = 1;}
	
	if((DT35_x_<=10)||(DT35_x_>=6200))
	{DT35_x_Flag = 0;}
	else {DT35_x_Flag = 1;}
	
	if((DT35_y<=10)||(DT35_y>=3500))
	{DT35_yFlag = 0;}
	else {DT35_yFlag = 1;}
	
	if((DT35_y_<=10)||(DT35_y_>=3500))
	{DT35_y_Flag = 0;}
	else{DT35_y_Flag = 1;}

}




void DT_flag(ST_ROBOT *pstRobot)
{
	float robot_angle = ConvertAngle(gyro_data.yaw * RADIAN_10);  // 将姿态角从0.1度转换为弧度 ConvertAngle() 函数将角度限制在 [-π, π) 的范围内 避免出现360度以上或者负角度的情况。
	float robot_angle_cos = fabs(cosf(robot_angle));	//arccos(0.98)约等于11.478度  arccos(0.15)约等于81.37度
	
	//判断机器人朝向
if(fabs(robot_angle_cos - 1)<=0.0006)	
{
	//机器人基本朝向前方（0度/180度）
		if(cosf(robot_angle)>=0)	//机器人朝向为初始方向
		{
			DT35_GYRO_STATE = FACE_0;		//机器人朝向前方			
		}
		else if(cosf(robot_angle)<0)
		{
			DT35_GYRO_STATE = FACE_180;	//机器人朝向后方
		}
}else if(fabs(robot_angle_cos - 0)<=0.0349)
	{
		if(robot_angle>=0)	    //机器人朝向为初始方向逆时针旋转90度
		{
			DT35_GYRO_STATE = FACE_90;
		}
		else if(robot_angle<0)  //机器人朝向为逆时针旋转270度
		{
			DT35_GYRO_STATE = FACE_270;
		}
	}else{
					DT35_GYRO_STATE = OTHER;
}





		//判断机器人位于哪个区域
if((((fabs(gyro_data.pitch)>=100)||(fabs(gyro_data.roll)>=100))&&(test_stRobot.stPos.fpPosZ>200))||(test_stRobot.stPos.fpPosZ>300))
{	
	FLAG_REGION3 = 1;//当这个标志位被置1，定位的逻辑就会被强制拉到三区
	REGION3_temp  = 1;
}








if(FLAG_REGION3 == 0&&FLAG_REGION3_manual==0)
{
if(pstRobot->stPos.fpPosX + length_x <= REGION1_X - REGION_TOLERANCE)
{
	REGION_STATE = REGION1;
	flag_DT35_locate = 1;
}else if((pstRobot->stPos.fpPosX >= REGION1_X + REGION_TOLERANCE)&&(pstRobot->stPos.fpPosX + length_x <= FIELD_X - REGION2_3_X -REGION_TOLERANCE)&&(pstRobot->stPos.fpPosY + length_y <= REGION2_Y+REGION_TOLERANCE ))
{
	REGION_STATE = REGION2_1;
	flag_DT35_locate = 1;
}else if((pstRobot->stPos.fpPosX >= REGION1_X-100 /*REGION_TOLERANCE*/)&&(pstRobot->stPos.fpPosX + length_x <= FIELD_X - REGION2_3_X +REGION_TOLERANCE)&&(pstRobot->stPos.fpPosY >= FIELD_Y - REGION2_Y -200))
{
	REGION_STATE = REGION2_2;
	flag_DT35_locate = 1;
}else if((pstRobot->stPos.fpPosX >= FIELD_X - REGION2_3_X - REGION_TOLERANCE)&&(pstRobot->stPos.fpPosX <= FIELD_X - REGION_TOLERANCE))
{
	REGION_STATE = REGION2_3;
	flag_DT35_locate = 1;
}else if(pstRobot->stPos.fpPosX >= FIELD_X - REGION_TOLERANCE)
{
	REGION_STATE = REGION3;
	flag_DT35_locate = 1;
}else
{
	REGION_STATE = OTHERS;   //由于判断区域时留有一定容忍度，所以会有部分死区
	flag_DT35_locate = 0;
}
}







else if(FLAG_REGION3==1||FLAG_REGION3_manual==1)
{
	REGION_STATE = REGION3;
	flag_DT35_locate = 1;
}



 if (nav.nav_state==NAV_AREA_3_RESET)
{
	REGION_STATE = REGION3_RESET;
	flag_DT35_locate = 1;
}

}



//四个DT35全局定位
void DT35_gyro_four(ST_ROBOT *pstRobot)
{
	DT_flag(pstRobot);

switch(DT35_GYRO_STATE)
{
	case FACE_0:
		DT35_x = DT35_distance.Num_1;
		DT35_x_ = DT35_distance.Num_3;
		DT35_y = DT35_distance.Num_2;
		DT35_y_ = DT35_distance.Num_4;
		DT35_judge();
		LENGTH_X = length_x;
		LENGTH_Y = length_y;
	  length_differx_1 = length_differX_1;
		length_differx_2 = length_differX_2;
		length_differy_1 = length_differY_1;	
		length_differy_2 = length_differY_2;


	break;
	
	case FACE_90:
		DT35_x = DT35_distance.Num_4;
		DT35_x_ = DT35_distance.Num_2;
		DT35_y = DT35_distance.Num_1;
		DT35_y_ = DT35_distance.Num_3;
		DT35_judge();
		LENGTH_X = length_y;
		LENGTH_Y = length_x;
		length_differx_1 = length_differY_1;
		length_differx_2 = length_differY_2;	
		length_differy_1 = length_differX_1;
		length_differy_2 = length_differX_2;	

	break;
	
	case FACE_180:
		DT35_x =  DT35_distance.Num_3;
		DT35_x_ = DT35_distance.Num_1;
		DT35_y =  DT35_distance.Num_4;
	  DT35_y_ = DT35_distance.Num_2;
		DT35_judge();
		LENGTH_X = length_x;
		LENGTH_Y = length_y;
		length_differx_1 = length_differX_2;	
		length_differx_2 = length_differX_1;
		length_differy_1 = length_differY_2;
		length_differy_2 = length_differY_1;

	break;
	
	case FACE_270:
		DT35_x = DT35_distance.Num_2;
		DT35_x_ = DT35_distance.Num_4;
		DT35_y = DT35_distance.Num_3;
	  DT35_y_ = DT35_distance.Num_1;
		DT35_judge();
		LENGTH_X = length_y;
		LENGTH_Y = length_x;	
		length_differx_1 = length_differY_2;
		length_differx_2 = length_differY_1;	
		length_differy_1 = length_differX_2;
		length_differy_2 = length_differX_1;	

	break;
	
	case OTHER:
	flag_DT35_locate = 0;
	break;
	
	
	default:
		break;
}	

}




void DT35_REGION(ST_ROBOT *pstRobot)
{
			switch(REGION_STATE)
		{
			case REGION1:

				if((fabs(DT35_y_ - pstRobot->stPos.fpPosY) <= 150)&&(DT35_y_Flag==1)) //判断Y负方向是否打到场地边缘
						{pstRobot->stPos.fpPosY = DT35_y_ + length_differy_2-length_differY_2;} 
				
						
						
				if (fabs(DT35_x - DT35_x_) >= 1300)//判断X正方向是否打到三区
				{
					//进入这一步时，要看X负方向的定位
					if((fabs(DT35_x_ - pstRobot->stPos.fpPosX) <=80 )&&(DT35_x_Flag==1))//判断X负方向是否打到场地边缘
						{
							pstRobot->stPos.fpPosX = DT35_x_+ length_differx_2-length_differX_2;
						}
				}

				else//X正负方向？？？都可信时
				{
							if(fabs(DT35_x+DT35_x_+LENGTH_X-REGION1_X)<=20&&DT35_xFlag==1&&DT35_x_Flag==1&&(4800>=stRobot.stPos.fpPosY&&stRobot.stPos.fpPosY>=1200))//若X方向两端距离都比较近，取均值//这一步卡的很严格
							{
								pstRobot->stPos.fpPosX = (DT35_x_+ length_differx_2+REGION1_X-DT35_x-length_differx_1)/2.0f-length_differX_2;
							}

							else
							{ 
								
								//也就是尽量用X正方向定位
								if(fabs(REGION1_X-DT35_x-LENGTH_X-pstRobot->stPos.fpPosX)<=100&&DT35_xFlag==1&&(4800>=stRobot.stPos.fpPosY&&stRobot.stPos.fpPosY>=1200))
								{
									pstRobot->stPos.fpPosX=REGION1_X-DT35_x-length_differx_1-length_differX_2;
								}	
								//不行了再用X负
								else 	if(fabs(DT35_x_ - pstRobot->stPos.fpPosX) <=80&&DT35_x_Flag==1)
								{
									pstRobot->stPos.fpPosX=DT35_x_+length_differx_2-length_differX_2;
								}		
							}
						}


					
		if (DT35_xFlag==1&&DT35_y_Flag==1&&DT35_Correct_Flag==1)
	{
    pstRobot->stPos.fpPosX=REGION1_X-DT35_x-length_differx_1-length_differX_2;
  	pstRobot->stPos.fpPosY = DT35_y_ + length_differy_2-length_differY_2; 
    
	stRobot.stPos.fpPosX=pstRobot->stPos.fpPosX;
    stRobot.stPos.fpPosY=pstRobot->stPos.fpPosY;

	}	
			break;
			
			case REGION2_1:
//				if(DT35_x>=DT35_x_)
//					{
//						
//						//取X方向比较近的DT35距离
//					//也就是说基本都在用X负定位
//				{	
//					if(fabs(FIELD_X - DT35_x - LENGTH_X- pstRobot->stPos.fpPosX)<=50&&DT35_xFlag==1)
//					{pstRobot->stPos.fpPosX = FIELD_X - DT35_x - length_differx_1-length_differX_2;}
//				}
//				else
//					{
                    

					if(fabs(DT35_x_-pstRobot->stPos.fpPosX)<=100&&DT35_x_Flag==1)//这个区域感觉X正就没有任何参考价值
					{pstRobot->stPos.fpPosX=DT35_x_+length_differx_2-length_differX_2;}	
					
					

//				}
//					
//			}
				
				
				
				if((fabs(REGION2_Y-DT35_y-LENGTH_Y-DT35_y_)<=100)&&DT35_yFlag==1&&DT35_y_Flag==1)//若Y方向两端距离计算后接近REGION2_Y，取均值
				{pstRobot->stPos.fpPosY=(DT35_y_+ length_differy_2+REGION2_Y-DT35_y-length_differy_1)/2.0f-length_differY_2 ;}
				
				else 
				{     							
					    if(fabs(REGION2_Y-DT35_y-LENGTH_Y-pstRobot->stPos.fpPosY)<=100&&DT35_yFlag==1)//优先使用Y正方向定位，因为靠墙
							{pstRobot->stPos.fpPosY = REGION2_Y-DT35_y-length_differy_1-length_differY_2;}	
							
							
							else if(fabs(DT35_y_ - pstRobot->stPos.fpPosY) <=50&&DT35_y_Flag==1)
							{pstRobot->stPos.fpPosY=DT35_y_+length_differy_2-length_differY_2;}
	
				}
				
    //在末端单独赋值进行覆盖
    if (DT35_x_Flag==1&&DT35_yFlag==1&&DT35_Correct_Flag==1)
	{
	pstRobot->stPos.fpPosX=DT35_x_+length_differx_2-length_differX_2;
	pstRobot->stPos.fpPosY = REGION2_Y-DT35_y-length_differy_1-length_differY_2;

		
	stRobot.stPos.fpPosX=pstRobot->stPos.fpPosX;
  stRobot.stPos.fpPosY=pstRobot->stPos.fpPosY;	

	}


			break;
			
			case REGION2_2:
				
			
//				if(DT35_x>=DT35_x_)//取X方向比较近的DT35距离
					//当X负更小的时候
			
			     
			    if(fabs(FIELD_X - DT35_x - LENGTH_X- pstRobot->stPos.fpPosX)<=80&&DT35_xFlag==1)//优先使用X正方向定位，这边定位条件更好
					{pstRobot->stPos.fpPosX = FIELD_X - DT35_x - length_differx_1-length_differX_2;}
					
					else if(fabs(DT35_x_-pstRobot->stPos.fpPosX)<=50&&DT35_x_Flag==1)
					{pstRobot->stPos.fpPosX=DT35_x_+length_differx_2-length_differX_2;}		

					

				

				
				
				
				
				if((fabs(REGION2_Y-DT35_y-LENGTH_Y-DT35_y_)<=100)&&DT35_yFlag==1&&DT35_y_Flag==1)//若Y方向两端距离计算后接近REGION2_Y，取均值
				{pstRobot->stPos.fpPosY=(DT35_y_+length_differy_2+REGION2_Y-DT35_y-length_differy_1)/2.0f+FIELD_Y-REGION2_Y-length_differY_2;}
				else 
					{
							if(fabs(FIELD_Y-REGION2_Y+DT35_y_ - pstRobot->stPos.fpPosY) <=100&&DT35_y_Flag==1)//优先利用Y负方向定位
							{pstRobot->stPos.fpPosY=FIELD_Y-REGION2_Y+DT35_y_+length_differy_2-length_differY_2;}
							
							else if(fabs(FIELD_Y-DT35_y-LENGTH_Y-pstRobot->stPos.fpPosY)<=100&&DT35_yFlag==1)
							{pstRobot->stPos.fpPosY = FIELD_Y-DT35_y-length_differy_1-length_differY_2;}		
				}
				
				

					
				
				
				if (DT35_xFlag==1&&DT35_y_Flag==1&&DT35_Correct_Flag==1)
	{
		pstRobot->stPos.fpPosX = FIELD_X - DT35_x - length_differx_1-length_differX_2;
		pstRobot->stPos.fpPosY=FIELD_Y-REGION2_Y+DT35_y_+length_differy_2-length_differY_2;
		
		stRobot.stPos.fpPosX=pstRobot->stPos.fpPosX;
    stRobot.stPos.fpPosY=pstRobot->stPos.fpPosY;


	}
	
			break;
				
			
			case REGION2_3:
				if(DT35_y>=DT35_y_)//Y正大于Y负时
   				{//利用Y负定位
						if(fabs(DT35_y_-pstRobot->stPos.fpPosY)<=100&&DT35_y_Flag==1)
						{pstRobot->stPos.fpPosY=DT35_y_+length_differy_2-length_differY_2;}
				}
				
				
				
				else{//利用Y正定位
					
					
					if(fabs(FIELD_Y-DT35_y-LENGTH_Y-pstRobot->stPos.fpPosY)<=100&&DT35_yFlag==1)
					{pstRobot->stPos.fpPosY=FIELD_Y-DT35_y-length_differy_1-length_differY_2;}

				}
				
				
				
				
				if(fabs(DT35_x+DT35_x_+LENGTH_X-REGION2_3_X)<=20&&DT35_xFlag==1&&DT35_x_Flag==1)//优先使用混合定位
				{
					pstRobot->stPos.fpPosX = (FIELD_X - DT35_x -length_differx_1 + FIELD_X - REGION2_3_X + DT35_x_+length_differx_2)/2.0f-length_differX_2;
				}
				
				else
					{
					if(fabs(FIELD_X - DT35_x -LENGTH_X-pstRobot->stPos.fpPosX)<=70&&DT35_xFlag==1)//优先利用X正方向定位，因为靠墙
					{pstRobot->stPos.fpPosX=FIELD_X - DT35_x -length_differx_1-length_differX_2;}
					
					else if(fabs(FIELD_X-REGION2_3_X+DT35_x_-pstRobot->stPos.fpPosX)<=100&&DT35_x_Flag==1)
					{
					pstRobot->stPos.fpPosX=FIELD_X-REGION2_3_X+DT35_x_+length_differx_2-length_differX_2;
					}
					
					
				}
				
				
	            if(DT35_y>=DT35_y_)//Y正大于Y负时
{			
				
	if (DT35_xFlag==1&&DT35_y_Flag==1&&DT35_Correct_Flag==1)
	{
		pstRobot->stPos.fpPosX=FIELD_X - DT35_x -length_differx_1-length_differX_2;
		pstRobot->stPos.fpPosY=DT35_y_+length_differy_2-length_differY_2;
		
		stRobot.stPos.fpPosX=pstRobot->stPos.fpPosX;
        stRobot.stPos.fpPosY=pstRobot->stPos.fpPosY;


	}
}
		else 
{
		if (DT35_xFlag==1&&DT35_yFlag==1&&DT35_Correct_Flag==1)
	{
		pstRobot->stPos.fpPosX=FIELD_X - DT35_x -length_differx_1-length_differX_2;
		
		pstRobot->stPos.fpPosY=FIELD_Y-DT35_y-length_differy_1-length_differY_2;
		
		stRobot.stPos.fpPosX=pstRobot->stPos.fpPosX;
        stRobot.stPos.fpPosY=pstRobot->stPos.fpPosY;


	}



}
			
		
		
		
			break;
			
			case REGION3://用于三区竞技赛和单项赛（非重试）
				
			FLAG_REGION3 = 1;


			
			if(DT35_x>=REGION3_X-LENGTH_X||DT35_x<=100)
			{	DT35_xFlag = 0;}
			else
			{	DT35_xFlag = 1;}
			if(DT35_x_>=REGION3_X-LENGTH_X||DT35_x_<=100)
			{	DT35_x_Flag = 0;}
			else 
			{DT35_x_Flag = 1;}



			//需要加遥控，令FLAG_REGION3_manual=1，REGION3_temp=1
			
			
			if((fabs(gyro_data.pitch)>=200||(fabs(gyro_data.roll)>=200))&&REGION3_temp==0)//判断开始上坡
				{	REGION3_temp = 1;}			
				else if(REGION3_temp==1&&(fabs(gyro_data.pitch)<=100)&&fabs(gyro_data.roll)<=100)//判断刚刚上完坡，到达平地
				{
					region3_state = 1;
				}

				
			switch(region3_state)
				{
		static uint8_t y_flag=0;
		static uint8_t x_flag=0;
					case 1:	
						DT35_xFlag = 1;

						if(DT35_y_<=1000)		//上坡后，以y_和x方向为准，以上坡后的角落为新起点，坐标系方向不变，即三区X方向均为负数
						{pstRobot->stPos.fpPosY = DT35_y_;
							y_flag = 1;}
						if(DT35_xFlag==1&&DT35_x<=800)
						{
							pstRobot->stPos.fpPosX = -DT35_x;
								
							x_flag = 1;
						}

						if(x_flag==1&&y_flag==1)	//直到Y，X两个方向都用DT35刷新过了，才会切到下一个状态
						{
							REGION3_temp = 2;
							region3_state = 2;//完成定位的标志
						}
						break;
						
					case 2:
					 if(fabs(DT35_x+DT35_x_+LENGTH_X-REGION3_X)<=80&&DT35_xFlag==1&&DT35_x_Flag==1)		//三区受到遮挡的情况比较多，因此刷新条件比较严格，主要靠随动轮
					{	pstRobot->stPos.fpPosX = (- (DT35_x + REGION3_X - LENGTH_X - DT35_x_))/2.0f;}
					//Y方向暂时不进行定位，使用随动轮
						break;


					default:
						break;
				}


					if (DT35_xFlag==1&&DT35_y_Flag==1&&DT35_Correct_Flag==1)
	{
		pstRobot->stPos.fpPosX = -(DT35_x + length_differx_1+length_differY_2);
		pstRobot->stPos.fpPosY=DT35_y_+length_differy_2-length_differX_2;
		
		stRobot.stPos.fpPosX=pstRobot->stPos.fpPosX;
        stRobot.stPos.fpPosY=pstRobot->stPos.fpPosY;
	}
			break;
			






			case OTHERS:
			break;

			case REGION3_RESET:

        
/**************************竞技赛三区重试************************************/
					 if(fabs(DT35_x+DT35_x_+LENGTH_X-REGION3_X)<=80&&DT35_xFlag==1&&DT35_x_Flag==1)		//三区受到遮挡的情况比较多，因此刷新条件比较严格，主要靠随动轮
					{	pstRobot->stPos.fpPosX = (- (DT35_x + REGION3_X - LENGTH_X - DT35_x_))/2.0f;}
            //Y方向DT35不可信，暂时不进行定位，使用纯随动轮
                      	
			
			if (DT35_xFlag==1&&DT35_y_Flag==1&&DT35_Correct_Flag==1)
			{
		pstRobot->stPos.fpPosX = -(DT35_x + length_differx_1-length_differX_1);
		pstRobot->stPos.fpPosY=DT35_y_+length_differy_2-length_differY_2;
 
		stRobot.stPos.fpPosX=pstRobot->stPos.fpPosX;
        stRobot.stPos.fpPosY=pstRobot->stPos.fpPosY;
	

	        }
/**************************竞技赛三区重试************************************/					
			break;

		default:
						break;
				
				

			





		
		
	}
}
  
void all_locate(void)
{   	static uint32_t dt35_time = 0;
	
	
	
	
	    static float raw_x = 0, raw_y = 0;//定位的原始值
	
    //利用随动轮定位	
		Follower_Wheel_Location(&stRobot, &stFollowerWheel);//随动轮累加定位，得到纯随动轮更新后的坐标
	  
	
//	  //利用雷达定位
//	  Vision_Location();

	  raw_x = stRobot.stPos.fpPosX - fpPosXOffset;//计算出纯随动轮判定出的X坐标
      raw_y = stRobot.stPos.fpPosY - fpPosYOffset;//计算出纯随动轮判定出的Y坐标

     ST_ROBOT temp_robot;//中间量结构体
     temp_robot.stPos.fpPosX = stRobot.stPos.fpPosX;  // 复制当前值
     temp_robot.stPos.fpPosY = stRobot.stPos.fpPosY;
     temp_robot.stPos.fpPosQ = stRobot.stPos.fpPosQ;

	
	
	
	
	// 判断区域（不修改stRobot）
     DT_flag(&temp_robot);//机器人朝向和区域判断，更新DT35_GYRO_STATE和REGION_STATE
	   DT35_gyro_four(&temp_robot);//更新4个DT35的坐标值和可信度

        if(DT35_GYRO_STATE!=OTHER)//仅在角度正的时候计算差值和计数
		{
		DT35_Delta[0]=fabs(DT35_x-DT35_distance_pre.Num_1);
		DT35_Delta[1]=fabs(DT35_y-DT35_distance_pre.Num_2);
		DT35_Delta[2]=fabs(DT35_x_-DT35_distance_pre.Num_3);
		DT35_Delta[3]=fabs(DT35_y_-DT35_distance_pre.Num_4);
		 
		DT35_distance_pre.Num_1=DT35_x;
		DT35_distance_pre.Num_2=DT35_y;
		DT35_distance_pre.Num_3=DT35_x_;
		DT35_distance_pre.Num_4=DT35_y_;
		 
		if(DT35_Delta[0]<=2.5f){DT35_X1_Count++;}
		else {DT35_X1_Count=0;}
		if(DT35_Delta[1]<=2.5f){DT35_Y1_Count++;}
		else{DT35_Y1_Count=0;}
		if(DT35_Delta[2]<=2.5f){DT35_X2_Count++;}
		else{DT35_X2_Count=0;}
		if(DT35_Delta[3]<=2.5f){DT35_Y2_Count++;}
		else{DT35_Y2_Count=0;}
		}








        

	
	
	
		if(dt35_time >= 10) 
		{ // 每100ms校正一次
        // 保存当前随动轮位置
		   //使用临时变量计算DT35位置，不修改stRobot	
 			
if(flag_DT35_locate)
	{ DT35_REGION(&temp_robot);//重定位
				
    fpPosXOffset = temp_robot.stPos.fpPosX-raw_x;//计算出新总修正量
    fpPosYOffset = temp_robot.stPos.fpPosY-raw_y;//计算出新总修正量

	//提前1ms覆盖上去先
    stRobot.stPos.fpPosX = raw_x + fpPosXOffset;//真正利用DT35刷新定位
    stRobot.stPos.fpPosY = raw_y + fpPosYOffset;

	dt35_time = 0;
	flag_DT35_locate = 0;//这一次定位完成
				
				DT35_X1_Count=0;
				DT35_Y1_Count=0;
				DT35_X2_Count=0;
				DT35_Y2_Count=0;
				
		if(DT35_Correct_Flag==1)
			{
			DT35_Correct_Flag=0;
		}
			
		}
	



		
  }
		
		dt35_time++;
}

