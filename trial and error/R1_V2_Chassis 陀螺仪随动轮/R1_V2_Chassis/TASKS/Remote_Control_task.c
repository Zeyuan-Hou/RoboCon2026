#include "Navigation_Task.h"


//这个就是遥控器应用层的函数（区分于底层函数），是去年比赛用的一版，做你们的任务时候请将多余的代码删掉。

/*******************************************************************************************
函数名称：parseDataPacket
函数功能：解析来自遥控器的数据包，并将解析后的数据填充到遥控器结构体ST_JS_VALUE中。
          数据包包含摇杆的位置和按键（36合一的矩阵键盘按键和8个独立按键）信息。
输入：   1. Rx_Buf （SPI通讯的存储数组）指向包含32个元素的uint8_t数组，每个元素代表一个字节的数据。
输出：   1. jsValue 指向ST_JS_VALUE结构体的指针，用于存储解析后的摇杆值和按键控制信息。
备注：   1. 数据包的前8个元素用于存储ADC采样值，其中0, 1: 左摇杆X方向；2, 3: 左摇杆Y方向；4, 5: 右摇杆X方向；6, 7: 右摇杆Y方向。
         2. 数据包的第8个元素表示矩阵键盘的值（范围0到36），第9-16个元素表示独立按键的值。
         3. ADC采样值是12位的，需要从两个连续的字节中提取高8位和低8位来重组完整的采样值。
         4. 解析过程中会考虑数据包中的所有相关信息，并将其适当地映射到目标结构体中。
*******************************************************************************************/


void parseDataPacket(const uint8_t Rx_Buf[32], ST_JS_VALUE *jsValue) {
    
    // ADC采样值处理（组装）
    
    jsValue->usJsLeft_X= (Rx_Buf[0] << 8) | Rx_Buf[1]; // 组装左摇杆X方向值    左X
    jsValue->usJsLeft_Y= (Rx_Buf[2] << 8) | Rx_Buf[3]; // 组装左摇杆Y方向值    左Y
    jsValue->usJsRight_X = (Rx_Buf[4] << 8) | Rx_Buf[5]; // 组装右摇杆X方向值    右X
    jsValue->usJsRight_Y = (Rx_Buf[6] << 8) | Rx_Buf[7]; // 组装右摇杆Y方向值    右Y

    
    // 矩阵键盘值赋值


	jsValue->usJsKey = Rx_Buf[8];
	
	//先给上板发Y轴值
	uart1_tx_buffer[1]=Rx_Buf[2];//用于上下板通讯
	uart1_tx_buffer[2]=Rx_Buf[3];//用于上下板通讯
			g_Usart2_Tx_buf[0]=jsValue->usJsKey;
	//用于给上板发送气泵相关
//	uart1_tx_buffer[3]=0;
	//再给上板发按键值
	uart1_tx_buffer[4]=jsValue->usJsKey;//用于上下板通讯

	switch (jsValue->usJsKey)
	{  //按键1到14分别对应spot0到spot13，15到17分别对应三条路径，26和27分别对应起重机的两种状态，28和30分别对应state1的两种状态，29对应全局手动模式
		
		//贴近一区的三个spot不设条件是为了方便在一区的时候可以直接先修改spot更改进入二区的位置
		case 23:
			spot=0;
					if(nav.nav_state==NAV_AREA_2||nav.nav_state==NAV_GLOBAL_MANUAL)
				{
		
				 flag_record=1;
				}
		break;
		
		case 22:
			spot=1;
							if(nav.nav_state==NAV_AREA_2||nav.nav_state==NAV_GLOBAL_MANUAL)
				{
		
				 flag_record=1;
				}
				if(nav.nav_state==NAV_AREA_1||nav.nav_state==NAV_GLOBAL_MANUAL)
					{
           vision_tx[1]=7;
					}
					
		break;
		
		case 21:
			
							if(nav.nav_state==NAV_AREA_2||nav.nav_state==NAV_GLOBAL_MANUAL)
				{
		     spot=2;
				 flag_record=1;
				}
							if(nav.nav_state==NAV_AREA_1||nav.nav_state==NAV_GLOBAL_MANUAL)
					{
           vision_tx[1]=1;
					}
		break;
		
		case 19:
			if(nav.nav_state==NAV_AREA_2||nav.nav_state==NAV_GLOBAL_MANUAL)
				{
			   spot=3;
				 flag_record=1;
				}
		break;
		case 17:
			if(nav.nav_state==NAV_AREA_2||nav.nav_state==NAV_GLOBAL_MANUAL)
				{
			   spot=4;
				 flag_record=1;
				}
							if(nav.nav_state==NAV_AREA_1||nav.nav_state==NAV_GLOBAL_MANUAL)
					{
           vision_tx[1]=2;
					}
		break;
		case 15:
			if(nav.nav_state==NAV_AREA_2||nav.nav_state==NAV_GLOBAL_MANUAL)
				{
			   spot=5;
					 flag_record=1;
				}
		break;
		case 13:
			if(nav.nav_state==NAV_AREA_2||nav.nav_state==NAV_GLOBAL_MANUAL)
				{
			   spot=6;
					 flag_record=1;
				}
							if(nav.nav_state==NAV_AREA_1||nav.nav_state==NAV_GLOBAL_MANUAL)
					{
           vision_tx[1]=3;
					}
		break;
		case 11:
			if(nav.nav_state==NAV_AREA_2||nav.nav_state==NAV_GLOBAL_MANUAL)
				{
			   spot=7;
					 flag_record=1;
				}
		break;
		case 9:
			if(nav.nav_state==NAV_AREA_2||nav.nav_state==NAV_GLOBAL_MANUAL)
				{
			   spot=8;
					 flag_record=1;
				}
			if(nav.nav_state==NAV_AREA_3_RESET||nav.nav_state==NAV_AREA_3||nav.nav_state==NAV_GLOBAL_MANUAL||nav.nav_state==NAV_AREA_3_SINGLE)
					{
						Region3_Spot=3;
				    flag_record=1;
					}
			if(nav.nav_state==NAV_AREA_1||nav.nav_state==NAV_GLOBAL_MANUAL)
					{
           vision_tx[1]=4;
					}
		break;
		case 7:
			if(nav.nav_state==NAV_AREA_2||nav.nav_state==NAV_GLOBAL_MANUAL)
				{
			   spot=9;
					 flag_record=1;
				}
		break;
		case 5:
			if(nav.nav_state==NAV_AREA_2||nav.nav_state==NAV_GLOBAL_MANUAL)
				{
			   spot=10;
					 flag_record=1;
				}
			if(nav.nav_state==NAV_AREA_3_RESET||nav.nav_state==NAV_AREA_3||nav.nav_state==NAV_GLOBAL_MANUAL||nav.nav_state==NAV_AREA_3_SINGLE)
					{
						Region3_Spot=2;
				    flag_record=1;
					}
			if(nav.nav_state==NAV_AREA_1||nav.nav_state==NAV_GLOBAL_MANUAL)
					{
           vision_tx[1]=5;
					}
		break;
		case 3:
				if(nav.nav_state==NAV_AREA_2||nav.nav_state==NAV_GLOBAL_MANUAL)
				{
			   spot=11;
					 flag_record=1;
				}
		break;
				
				
				
		case 2:
			if(nav.nav_state==NAV_AREA_2||nav.nav_state==NAV_GLOBAL_MANUAL)
				{
			   spot=12;
					 flag_record=1;
				}
		break;
		case 1:
			if(nav.nav_state==NAV_AREA_2||nav.nav_state==NAV_GLOBAL_MANUAL)
				{
			   spot=13;
					 flag_record=1;
				}
		 if(nav.nav_state==NAV_AREA_3_RESET||nav.nav_state==NAV_AREA_3||nav.nav_state==NAV_GLOBAL_MANUAL||nav.nav_state==NAV_AREA_3_SINGLE)
					{
						Region3_Spot=1;
				    flag_record=1;
					}
			if(nav.nav_state==NAV_AREA_1||nav.nav_state==NAV_GLOBAL_MANUAL)
					{
           vision_tx[1]=6;
					}
		break;
				
				
		case 18:
			nav.nav_state=NAV_AREA_1;//一区
		  flag_record=1;
		break;
		
		case 14:
			nav.nav_state=NAV_AREA_2;//点对点二区
		  flag_record=1;
		break;
		case 10:
			uphill=0;
			nav.nav_state=NAV_AREA_3;//三区导航
		    flag_record=1;
		break;
		
		case 6://在三区重试就摁这个


		
/***************************竞技赛三区重试*******************************/
//		//标志位更新
//		  uphill=1;//并无大用
//		  uphilling = 0;//并无大用
//		  region3_state=2;//并无大用
//		  REGION3_temp=2;//在这里并无大用
//		  flag_record=1;
//		//   FLAG_REGION3_manual=1;// 防止再次刷新REGION_STATE		
//		  nav.nav_state=NAV_AREA_3_RESET;//三区导航重试
//		  REGION_STATE =REGION3_RESET;//直接切到三区重试的状态，避免因为位置误差导致的区域判断错误
//		  Q=0;//不转了
/***************************竞技赛三区重试*******************************/

		
		
		
		
		
		
		
		
		
		
		
		
		
		
/***************************单项赛*******************************/
nav.nav_state=NAV_AREA_3_SINGLE;//三区导航重试
flag_record=1;

Gyro_Data_Test.fpQ_Cur = gyro_data.yaw;
Gyro_Data_Test.fpQ_Pre = gyro_data.yaw;
/***************************单项赛*******************************/
		break;
		
		
		case 26:
			Crane_State=Crane_Up;
        break;
		case 27:
			if(flag_record==0){
			flag_record=1;}

            vision_tx[1]=8;
					
        break;
		
		case 28:
			Crane_State=Crane_Down;
		
		break;
		case 20:
			  flag_global_manual=1;
		      enter=0;//关于重进二区导航的准备
			  nav.nav_state=NAV_GLOBAL_MANUAL;
		break;
		
    case 30://前进按钮
      if(nav.nav_state==NAV_AREA_1&&(nav.auto_path.run_time>=1000))//防止一下子把状态切换完了
     {
	     state1+=1;
         flag_record=1;
		 nav.auto_path.run_time=0;
       			 
      }
		 //遥控控制竞技赛三区切换状态
		 if(nav.nav_state==NAV_AREA_3&&(nav.auto_path.run_time>=1000)&&(state3)>=3)
		 {
		 	     state3+=1;
         flag_record=1;
		 nav.auto_path.run_time=0;		 
		 }
		 
		 
		 	//遥控控制单项赛三区切换状态
		 		 if(nav.nav_state==NAV_AREA_3_SINGLE&&(nav.auto_path.run_time>=1000)&&(state3_SINGLE)>=3)
		 {
		 state3_SINGLE+=1;
         flag_record=1;
		 nav.auto_path.run_time=0;		 
		 }
		 

		                 	
    break;

    case 32://手动矫正DT35误判//32
	

   DT35_Correct_Flag=1;


    break;
		
		case 29:
		
//		gyro_reset_flag=1;
//		

//				if(gyro_reset_flag){
//		HAL_UART_Transmit_DMA(&huart2, g_Usart2_Tx_buf, 1);
//					g_Usart2_Tx_buf[0]=0;
//				gyro_reset_flag=0;}	
		
		
		break;
		
	
	
    
	default:
	break;

	}
    // 独立按键值赋值
    for (int i = 0; i < 8; ++i)
	 {
        jsValue->indepen_usJsKey[i] = Rx_Buf[9 + i];
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
    const float smooth = 20; // 平滑等级，一阶低通滤波器截止频率
    static ST_LPF FJx = {0, 0, 0, smooth, 0.002f}; // 需要pre_out，故需要设为静态变量
    static ST_LPF FJy = {0, 0, 0, smooth, 0.002f};
    static ST_LPF FJw = {0, 0, 0, smooth, 0.002f};

    // 计算左摇杆X方向的速度 (fpVy)
    float Vx = jsValue->usJsLeft_X - LEFT_JS_X_MID;
    if (fabs(Vx) < ucGateX) {
        FJx.in = 0;
    } 
		else {
			if(Vx > 0)
			{
				FJx.in = (Vx- ucGateX) * ssXSpedLimit /(LEFT_JS_X_MAX-LEFT_JS_X_MID );
			}
			else if (Vx < 0)
			{
				FJx.in = (Vx + ucGateX) * ssXSpedLimit /(LEFT_JS_X_MID-LEFT_JS_X_MIN );
			}
    }

    // 计算左摇杆Y方向的速度 (fpVx)

    float Vy = jsValue->usJsLeft_Y - LEFT_JS_Y_MID;

			if (fabs(Vy) < ucGateY) 
			{
					FJy.in = 0;
			} 
			else {
					if (Vy > 0) {
							// Y轴正向偏移
							FJy.in = (Vy - ucGateY) * ssYSpedLimit / (LEFT_JS_Y_MAX - LEFT_JS_Y_MID);
					} 
					else if (Vy < 0) {
							// Y轴负向偏移
							FJy.in = (Vy + ucGateY) * ssYSpedLimit / (LEFT_JS_Y_MID - LEFT_JS_Y_MIN);
					}
			}


// 计算右摇杆X方向的角速度 (fpW)
		float Vw = jsValue->usJsRight_X - RIGHT_JS_MID;
		if (fabs(Vw) < ucGateW) {
				FJw.in = 0; // 在死区内，设置为0
		} 
		else {
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


    p_nav->auto_path.basic_velt.fpVx = FJx.out; // 摇杆值X方向从右到左是从0到4096。
    p_nav->auto_path.basic_velt.fpVy  = -FJy.out; // 摇杆值Y方向从上到下是从0到4096。
    p_nav->auto_path.basic_velt.fpW = -FJw.out ; //  右摇杆X控制角速度
}
extern ST_VECTOR expect_robot_local_Velt;
void pack_data(uint8_t buffer[])
{

    buffer[0] =REMOTE_BUFFER[0];
    buffer[1] =REMOTE_BUFFER[1];
    buffer[2] =REMOTE_BUFFER[2];
	buffer[3] =0;
    buffer[4] =0;
	
}


//void monitor()
//{
//  if(system_monitor.motor_LU_fps> 1100 || system_monitor.motor_LU_fps < 900)
//	{	
//		All_Monitor.lu=0;
//	}
//	else 
//		All_Monitor.lu=16;
//	if(system_monitor.motor_RU_fps > 1100 || system_monitor.motor_RU_fps < 900)
//	{
//	    All_Monitor.ru=0;
//	}
//	else
//		All_Monitor.ru=32;
//	if(system_monitor.motor_LD_fps  > 1100 || system_monitor.motor_LD_fps  < 900)
//	{
//	    All_Monitor.ld=0;
//	}
//	else
//		All_Monitor.ld=64;
//	if(system_monitor.motor_RD_fps > 1100 || system_monitor.motor_RD_fps < 900)
//	{
//	    All_Monitor.rd=0;
//	}
//	else
//		All_Monitor.rd=128;
//	if(system_monitor.uart2_fps > 510 || system_monitor.uart2_fps < 380)
//	{
//	    All_Monitor.gyro=0;
//	}
//	else
//		All_Monitor.gyro=1;
//	if(system_monitor.motor3508_fps > 1100 || system_monitor.motor3508_fps < 900)
//	{
//	    All_Monitor.motor_3508=0;
//	}
//	else
//		All_Monitor.motor_3508=2;
//	if(system_monitor.air_board_fps > 850 || system_monitor.air_board_fps < 750)
//	{
//	    All_Monitor.air=0;
//	}
//	else
//		All_Monitor.air=4;
//	if(system_monitor.communicate_rx_fps > 1100 || system_monitor.communicate_rx_fps < 900)
//	{
//	    All_Monitor.cr=0;
//	}
//	else
//		All_Monitor.cr=8;
//	if(system_monitor.communicate_tx_fps > 1100 || system_monitor.communicate_tx_fps < 900)
//	{
//	    All_Monitor.ct=0;
//	}
//	else
//		All_Monitor.ct=16;
//	REMOTE_BUFFER[0]=BUFFER[0];
//	REMOTE_BUFFER[1]=BUFFER[1]+All_Monitor.lu+All_Monitor.ru+All_Monitor.ld+All_Monitor.rd;
//	REMOTE_BUFFER[2]=All_Monitor.gyro+All_Monitor.motor_3508+All_Monitor.air+All_Monitor.cr+All_Monitor.ct;
//	
//}


