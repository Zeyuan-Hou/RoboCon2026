#include "Logic.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "Trajectory.h"


// 判断是否到达目标位置的函数，利用位置容差判断
// 参数：对应电机的容许误差
uint8_t BlockArm_State_Judge(float Joint1, float Joint2, float Joint3, float Gimbal)
{
    if (fabs(BlockArm_Joint1_J60Data.position_ - MotorInput.BlockArm_Joint1_J60) < Joint1 &&
        fabs(BlockArm_Joint2_J60Data.position_ - MotorInput.BlockArm_Joint2_J60) < Joint2 &&
        fabs(BlockArm_Joint3_3508.angle - MotorInput.BlockArm_Joint3_3508) < Joint3 &&
        fabs(BlockArm_Gimbal_J60Data.position_ - MotorInput.BlockArm_Gimbal_J60) < Gimbal)
    {
        return 1; // 已经到达目标位置
    }
    else
    {
        return 0; // 还没有到达目标位置
    }
}

//     /**
//  * 求解两杆机械臂的逆运动学，并施加角度约束
//  * @param L1 杆1长度
//  * @param L2 杆2长度
//  * @param x  末端x坐标
//  * @param y  末端y坐标
//  * @param theta1 输出杆1与水平方向夹角（弧度），范围[0, 3π/4]
//  * @param theta2 输出杆2与水平方向夹角（弧度），范围[-π/2, π/2]，且 theta2 <= theta1
//  * @return 1表示找到满足条件的解，0表示无解或不可达
//  */
// uint8_t BlockArm_BackSolve(ARM_BACKSOLVING *params)
// {
//     d = sqrt(params->x * params->x + params->y * params->y);

//     // 检查可达性
//     if (d > params->L1 + params->L2 + EPS || d < fabs(params->L1 - params->L2) - EPS) 
//     {
//         return 0;  // 不可达
//     }

//     float beta = atan2(params->y, params->x);  // 末端方向角，范围 [-π, π]

//     // 计算杆1与连线之间的夹角 alpha
//     float cos_alpha = ((params->L1 * params->L1) + d * d - (params->L2 * params->L2)) / (2.f * params->L1 * d);
//     if (cos_alpha > 1.f) cos_alpha = 1.f;
//     if (cos_alpha < -1.f) cos_alpha = -1.f;
//     float alpha = acos(cos_alpha);  // [0, π]

//     // 两个候选的 theta1
//     theta1_cand[0] =  beta + alpha  ;
// 	theta1_cand[1] =  beta - alpha;
	
//     for (int i = 0; i < 2; i++) 
// 	{
//         float th1 = theta1_cand[i];

//         // 检查 th1 是否在 [0, 3*π/4] 内
//         if (th1 < 0.f || th1 > 3.f*PI / 4.f) 
// 		{
//             continue;
//         }

//         // 计算杆1末端坐标
//         float x1 = params->L1 * (float)cos(th1);
//         float y1 = params->L1 * (float)sin(th1);

//         // 计算杆2方向角
//         float dx = params->x - x1;
//         float dy = params->y - y1;
//         float th2 = (float)atan2(dy, dx);  // 范围 [-π, π]

//         // 检查 th2 是否在 [-π/2, π/2] 内
//         if (th2 < -(PI / 2.f )|| th2 > (PI / 2.f)) 
//         {
//             continue;
//         }

//         // 检查 th2 <= th1
//         if (th2 > th1 + EPS) 
//         {
//             continue;
//         }

//         // 找到满足条件的解
//         params->theta1 = th1*DEG;
//         params->theta2 = th2*DEG; 
//         return 1;
//     }

//     return 0;  // 无满足约束的解
// }

//根据输入的x，y及第三级机械臂与水平方向的夹角解算电机目标角度
void BlockArm_InputSolve(ARM_BACKSOLVING *params,MOTORINPUT *Input)
{
    if(BlockArm_BackSolve(params)==1)
	{
        Input->BlockArm_Joint1_J60=(params->theta1-params->theta1_0);//J60正转，机械臂逆时针转
        Input->BlockArm_Joint2_J60=-(params->theta2-params->theta2_0);//J60正转，机械臂顺时针转
	}
}

void BlockArm_Init(void)
{

    
    if (BlockArm_Task_Change.First_Init==1)
    {   
		Arm_BackSolving.x=Arm_BackSolving.x0;
		Arm_BackSolving.y=Arm_BackSolving.y0;
        BlockArm_BackSolve(&Arm_BackSolving);
        Arm_BackSolving.theta1_0=Arm_BackSolving.theta1;
        Arm_BackSolving.theta2_0=Arm_BackSolving.theta2;

        Gravity_BlockArm.angle0[0]=Arm_BackSolving.theta1;
        Gravity_BlockArm.angle0[1]=Arm_BackSolving.theta2;
        Gravity_BlockArm.angle0[2]=0;
        BlockArm_Task_Change.First_Init=0;
    }//第一次初始化，根据给出的初始位置计算初始角度
    
    Traj_Angle_Init(&Traj_BlockArm_Joint3,MotorInput.BlockArm_Joint3_3508, 76.f, 240.f, 1.f);
	if(fabsf(MotorInput.BlockArm_Gimbal_J60)<=10)
	{
		Traj_Angle_Init(&Traj_Gimbal, MotorInput.BlockArm_Gimbal_J60, BlockArm_Gimbal_J60_Init_Position, 90.f, 0.3f);
	}
	else 
	{
		Traj_Angle_Init(&Traj_Gimbal, MotorInput.BlockArm_Gimbal_J60, BlockArm_Gimbal_J60_Init_Position, 90.f, 10.f);
	}
    Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y, 145.f, 175.f,0.f,0.f, 500.f,1000.f);
	
//	Traj_Angle_Init(&Traj_BlockArm_Joint3,MotorInput.BlockArm_Joint3_3508, -90.f, 180.f, 3.f);
//    Traj_Angle_Init(&Traj_Gimbal, MotorInput.BlockArm_Gimbal_J60, 180, 90.f, 3.f);
//    Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y, 300.f, 380.f,0.f,0.f, 500.f,2000.f);
	
}

void BlockArm_Logic(void)
{
    static uint32_t Ready_Time;
    static uint32_t Move_Time;
    static uint32_t Fetch_Time;
    static uint32_t Retract_Time;
    static uint32_t StoreBlock_Time;
    static uint32_t PickUpBlock_Time;
    static uint32_t PutBlock_Time;
	static uint8_t MovementFlag[10]={0};
	
	if(BlockArm_Task_State!=BlockArm_Task_StatePre)
	{
		for(uint16_t i=0;i<10;i++)
		{
			MovementFlag[i]=0;
		}
	}//切状态时清空运动标志位
	
	BlockArm_Task_StatePre=BlockArm_Task_State;
	
    switch (BlockArm_Task_State)
    {
    case BLOCKARM_TASK_INIT:
        if (Ready_Time<=3000&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
        {
            BlockArm_Init();
        }
        Ready_Time++;
        if (BlockArm_Task_Change.Ready==1&&Ready_Time>=100)
        {
            Ready_Time=0;
            Move_Time=0;
            Fetch_Time=0;
            Retract_Time=0;
            StoreBlock_Time=0;
            PickUpBlock_Time=0;
            PutBlock_Time=0;
            QD_Ctrl.BlockArm_Fetch_Air_Pump_State=1;//打开机械臂气泵
            QD_Ctrl.BlockArm_Store0_Air_Pump_State=1;//打开存块气泵
            QD_Ctrl.BlockArm_Store1_Air_Pump_State=1;//打开存块气泵
            BlockArm_Task_State=BLOCKARM_TASK_MOVE_TO_FETCH_POSITION;
            BlockArm_Task_Change.Ready=0;
        }//进入二区取方块状态

        if (BlockArm_Task_Change.Ready==2&&Ready_Time>=100)
        {
            Ready_Time=0;
            Move_Time=0;
            Fetch_Time=0;
            Retract_Time=0;
            StoreBlock_Time=0;
            PickUpBlock_Time=0;
            PutBlock_Time=0;
            QD_Ctrl.BlockArm_Fetch_Air_Pump_State=1;//打开机械臂气泵
            QD_Ctrl.BlockArm_Store0_Air_Pump_State=1;//打开存块气泵
            QD_Ctrl.BlockArm_Store1_Air_Pump_State=1;//打开存块气泵
            BlockArm_Task_State=BLOCKARM_TASK_PICKUP_BLOCK;
            BlockArm_Task_Change.Ready=0;
        }//进入三区存方块状态

        break;
    
    case BLOCKARM_TASK_MOVE_TO_FETCH_POSITION:
        
        switch (BlockArm_Task_Change.Block_Height)//根据方块高度执行不同的动作
        {
        case 0://取地上的方块
				
			if(MovementFlag[0]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
			{
				Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,  325,  200, 0.f, 0.f,1000.f, 5000.f);
				Traj_Angle_Init(&Traj_BlockArm_Joint3,MotorInput.BlockArm_Joint3_3508, -90.f, 720.f, 0.3f);
				MovementFlag[0]=1;
			}
			else if(MovementFlag[0]==1&&MovementFlag[1]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
			{				
				Traj_Arc_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y-75.f, 75.f, 90.f, -90.f,0.f,1000.f, 1000.f, 5000.f);

				MovementFlag[1]=1;
			}
			else if(MovementFlag[1]==1&&MovementFlag[2]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
			{				
				Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,  Arm_BackSolving.x,  Arm_BackSolving.y-50.f, 1000.f, 0.f,1000.f, 10000.f);
				Gravity_BlockArm.k_Joint3=BlockArm_Joint3_GfWithBlock;
				MovementFlag[2]=1;
			}
			else if(MovementFlag[2]==1&&MovementFlag[3]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
			{		
				vTaskDelay(pdMS_TO_TICKS(200));
				Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y, 300.f, 380.f, 0.f, 0.f,1000.f, 5000.f);		
				MovementFlag[3]=1;
			}
							
			if (MovementFlag[3]==1&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)//记得加检查角度
			{
				BlockArm_Task_State=BLOCKARM_TASK_RETRACT_ARM;//切换下一状态
				return;
			}
			
            break;
			
        case 1://取200方块
            if(MovementFlag[0]==0&&Traj_Arm.State==TRAJ_DONE)
            {
                Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,200.f, 200.f,0.f,0.f, 2000.f, 5000.f);
				Traj_Angle_Init(&Traj_BlockArm_Joint3,MotorInput.BlockArm_Joint3_3508, 0.f, 720.f, 0.2f);
				MovementFlag[0]=1;
            }
            else if(MovementFlag[0]==1&&MovementFlag[1]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
            {
                Traj_Arc_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y-100.f, 100.f, 90.f, -90.f,0.f,2000.f, 2000.f, 15000.f);
				MovementFlag[1]=1;
            }
			else if(MovementFlag[1]==1&&MovementFlag[2]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
            {
                Traj_Arc_Init(&Traj_Arm, Arm_BackSolving.x+100, Arm_BackSolving.y, 100.f, 180.f, 90.f,2000.f,1000.f, 2000.f, 15000.f);
				MovementFlag[2]=1;
            }
            else if(MovementFlag[2]==1&&MovementFlag[3]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
            {
                Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,  Arm_BackSolving.x+50.f,  Arm_BackSolving.y, 1000.f, 0.f,2000.f, 10000.f);
				MovementFlag[3]=1;
            }
			
			
			if (MovementFlag[3]==1&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)//记得加检查角度
            {
				BlockArm_Task_State=BLOCKARM_TASK_FETCH_BLOCK;//切换下一状态
				return;
            }
            break;

        case 2://取400方块
			
			
			if(MovementFlag[0]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
            {
				Traj_Angle_Init(&Traj_BlockArm_Joint3,MotorInput.BlockArm_Joint3_3508, 0.f, 720.f, 0.2f);
                Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,200.f, 200.f, 0.f, 0.f, 2000.f, 15000.f);
				MovementFlag[0]=1;
            }
            else if(MovementFlag[0]==1&&MovementFlag[1]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
            {
                Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,Arm_BackSolving.x+250.f, Arm_BackSolving.y, 0.f, 0.f, 2000.f, 15000.f);
				MovementFlag[1]=1;
            }

            if (MovementFlag[1]==1&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)//记得加检查角度
            {
				BlockArm_Task_State=BLOCKARM_TASK_FETCH_BLOCK;//切换下一状态
				return;
            }
            break;

        case 3://取600方块
            if(MovementFlag[0]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
            {
                Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,150.f, 400.f, 0.f, 1000.f, 2000.f, 10000.f);
				MovementFlag[0]=1;
            }
            else if(MovementFlag[0]==1&&MovementFlag[1]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
            {
				Traj_Angle_Init(&Traj_BlockArm_Joint3,MotorInput.BlockArm_Joint3_3508, 0.f, 720.f, 0.2f);
                Traj_Arc_Init(&Traj_Arm, Arm_BackSolving.x+100.f, Arm_BackSolving.y, 100.f, 180.f, -90.f, 1000.f,2000.f, 2000.f, 10000.f);
				MovementFlag[1]=1;
            }
            else if(MovementFlag[1]==1&&MovementFlag[2]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
            {
                Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,  Arm_BackSolving.x+250.f,  Arm_BackSolving.y, 2000.f, 0.f, 2000.f, 10000.f);
				MovementFlag[2]=1;
            }
			
			if (MovementFlag[2]==1&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)//记得加检查角度
            {
				BlockArm_Task_State=BLOCKARM_TASK_FETCH_BLOCK;//切换下一状态
				return;				
            }
            break;
        
        default:
            break;
        }
        Move_Time++;
        break;
    
    case BLOCKARM_TASK_FETCH_BLOCK://取台阶上的方块时执行的吸取与收回动作

            
			if(Fetch_Time==0)
			{
				Arm_BackSolving.x+=100;
			}
			
			if(MovementFlag[0]==1&&Traj_Arm.State==TRAJ_DONE)
            {
				Gravity_BlockArm.k_Joint3=BlockArm_Joint3_GfWithBlock;
			}
			
			switch (BlockArm_Task_Change.Block_Height)
            {
            case 1:
                if(MovementFlag[0]==0&&Traj_Arm.State==TRAJ_DONE&&Fetch_Time>=200)
                {
                    Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,200.f, 400.f, 0.f, 0.f, 2000.f, 10000.f);
				    MovementFlag[0]=1;
                }
                break;
            
            case 2:
                if(MovementFlag[0]==0&&Traj_Arm.State==TRAJ_DONE&&Fetch_Time>=200)
                {
                    Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,200.f, 400.f, 0.f, 0.f, 2000.f, 10000.f);
				    MovementFlag[0]=1;
                }
                break;

            
            case 3:
                if(MovementFlag[0]==0&&Traj_Arm.State==TRAJ_DONE&&Fetch_Time>=200)
                {
                    Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,200.f, 400.f, 0.f, 0.f, 2000.f, 10000.f);
				    MovementFlag[0]=1;
                }
                break;

            
            default:
                break;
            }
			

            if(MovementFlag[0]==1&&Traj_Arm.State==TRAJ_DONE)
            {
				BlockArm_Task_State=BLOCKARM_TASK_RETRACT_ARM;//进入下一个状态
				return;				
			}

            Fetch_Time++;
        break;
    
    case BLOCKARM_TASK_RETRACT_ARM://收回方块到存储位置
            
		if(BlockArm_Task_Change.BlockStore[2]==1)
		{
			if (BlockArm_Task_Change.Ready==2)
			{
				Ready_Time=0;
				Move_Time=0;
				Fetch_Time=0;
				Retract_Time=0;
				StoreBlock_Time=0;
				PickUpBlock_Time=0;
				PutBlock_Time=0;
				QD_Ctrl.BlockArm_Fetch_Air_Pump_State=1;//打开机械臂气泵
				QD_Ctrl.BlockArm_Store1_Air_Pump_State=1;//打开存块气泵
				BlockArm_Task_State=BLOCKARM_TASK_PUT_BLOCK;
				BlockArm_Task_Change.Ready=0;
				BlockArm_Task_Change.BlockStore[2]=0;
			}
			return;
		}
	
		if (BlockArm_Task_Change.BlockStore[0]==0&&BlockArm_Task_Change.BlockStore_State==1)
        {	
            if(MovementFlag[0]==0&&MovementFlag[2]==0&&Traj_Arm.State==TRAJ_DONE)
            {
				if(BlockArm_Task_Change.Block_Height!=0)
                {
					Traj_Angle_Init(&Traj_Gimbal,MotorInput.BlockArm_Gimbal_J60, 90.f, 180.f, 1.f);
					MovementFlag[0]=1;
				}
				else
				{
					Traj_Angle_Init(&Traj_Gimbal,MotorInput.BlockArm_Gimbal_J60, 180.f, 180.f, 1.5f);
					MovementFlag[2]=1;
				}
            }
            else if(MovementFlag[0]==1&&MovementFlag[1]==0&&Traj_Gimbal.State==TRAJ_DONE&&BlockArm_Task_Change.Block_Height!=0)
            {
				Traj_Angle_Init(&Traj_BlockArm_Joint3, MotorInput.BlockArm_Joint3_3508, -90.f,180.f, 0.8f);
				Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,280.f, 400.f, 0.f, 0.f, 800.f, 5000.f);
				MovementFlag[1]=1;
            }
            else if(MovementFlag[1]==1&&MovementFlag[2]==0&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
            {
                Traj_Angle_Init(&Traj_Gimbal,MotorInput.BlockArm_Gimbal_J60, 180.f, 180.f, 1.f);
				MovementFlag[2]=1;
            }
			else if(MovementFlag[2]==1&&MovementFlag[3]==0&&Traj_Gimbal.State==TRAJ_DONE)
            {
                Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,Arm_BackSolving.x,Arm_BackSolving.y-80.f, 0.f, 0.f, 1000.f, 10000.f);
				MovementFlag[3]=1;
            }

		    if (MovementFlag[3]==1&&Traj_Arm.State==TRAJ_DONE)//记得加检查角度
			{
                BlockArm_Task_Change.BlockStore[0]=1;
                BlockArm_Task_State=BLOCKARM_TASK_STORAGE_BLOCK;//进入下一个状态
				return;
			}
        }
//		else if (BlockArm_Task_Change.BlockStore[1]==0)
//        {
//            if(MovementFlag[0]==0&&MovementFlag[2]==0&&Traj_Arm.State==TRAJ_DONE)
//            {
//				if(BlockArm_Task_Change.Block_Height!=0)
//                {
//					Traj_Angle_Init(&Traj_Gimbal,MotorInput.BlockArm_Gimbal_J60, 90.f, 180.f, 1.f);
//					MovementFlag[0]=1;
//				}
//				else
//				{
//					Traj_Angle_Init(&Traj_Gimbal,MotorInput.BlockArm_Gimbal_J60, 115.f, 180.f, 1.5f);
//					MovementFlag[2]=1;
//				}
//            }
//            else if(MovementFlag[0]==1&&MovementFlag[1]==0&&Traj_Gimbal.State==TRAJ_DONE)
//            {
//                Traj_Angle_Init(&Traj_BlockArm_Joint3, MotorInput.BlockArm_Joint3_3508, -95.f, 180.f, 1.5f);
//                Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,330.f, 380.f, 0.f, 0.f, 500.f, 2000.f);
//				MovementFlag[1]=1;
//            }
//            else if(MovementFlag[1]==1&&MovementFlag[2]==0&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
//            {
//                Traj_Angle_Init(&Traj_Gimbal,MotorInput.BlockArm_Gimbal_J60, 115.f, 180.f, 1.f);
//				MovementFlag[2]=1;
//            }
//			else if(MovementFlag[2]==1&&MovementFlag[3]==0&&Traj_Gimbal.State==TRAJ_DONE)
//            {
//                Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,Arm_BackSolving.x,Arm_BackSolving.y-80.f, 0.f, 0.f, 500.f, 2000.f);
//				MovementFlag[3]=1;
//            }

//		    if (MovementFlag[3]==1&&Traj_Arm.State==TRAJ_DONE)//记得加检查角度
//			{
//                BlockArm_Task_Change.BlockStore[1]=1;
//                BlockArm_Task_State=BLOCKARM_TASK_STORAGE_BLOCK;//进入下一个状态
//				return;
//			}
//        }  
			
        else if (BlockArm_Task_Change.BlockStore[2]==0&&BlockArm_Task_Change.BlockStore_State==3)
        {
			
			if(MovementFlag[0]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
			{
                if(BlockArm_Task_Change.Area_State!=3)
				{
					Traj_Angle_Init(&Traj_Gimbal,MotorInput.BlockArm_Gimbal_J60, 90.f, 180.f, 1.f);
				}
				MovementFlag[0]=1;
			}
			else if(MovementFlag[0]==1&&MovementFlag[1]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
			{
				Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,  250.f,  350.f, 0.f, 0.f,500.f, 2000.f);
				Traj_Angle_Init(&Traj_BlockArm_Joint3, MotorInput.BlockArm_Joint3_3508, 0.f, 180.f, 0.8f);
				MovementFlag[1]=1;
			}
			else if(MovementFlag[1]==1&&MovementFlag[2]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
			{				
				BlockArm_Task_Change.BlockStore[2]=1;
			}
			
        }

        Retract_Time++;
        break;

    case BLOCKARM_TASK_STORAGE_BLOCK://换气存储方块,回到初始状态
        
		//等待方块放稳

        if (MovementFlag[0]==0&&Traj_Arm.State==TRAJ_DONE&&StoreBlock_Time<=10)
        {
            QD_Ctrl.BlockArm_Fetch_Air_Pump_State=0;//关闭机械臂气泵
            QD_Ctrl.BlockArm_Fetch_Solenoid_Valve_State=1;//打开机械臂电磁阀，喷气
			Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,Arm_BackSolving.x,Arm_BackSolving.y+80.f, 0.f, 0.f, 1000.f, 5000.f);
			MovementFlag[0]=1;
        }
        //等待提起方块
        else if (StoreBlock_Time>=250&&StoreBlock_Time<300)
        {
            QD_Ctrl.BlockArm_Fetch_Solenoid_Valve_State=0;//停止喷气
			Gravity_BlockArm.k_Joint3=BlockArm_Joint3_GfNoBlock;
        }
		
        else if(MovementFlag[0]==1&&MovementFlag[1]==0&&Traj_Arm.State==TRAJ_DONE&&StoreBlock_Time==300)
        {            
			Traj_Angle_Init(&Traj_Gimbal, MotorInput.BlockArm_Gimbal_J60, 0.f, 360.f, 1.5f);
			if(BlockArm_Task_Change.Block_Height==3)
			{
				Traj_Angle_Init(&Traj_BlockArm_Joint3, MotorInput.BlockArm_Joint3_3508, 76.f, 180.f, 1.f);
			}
			MovementFlag[1]=1;
        }
        
		if(MovementFlag[1]==1&&MotorInput.BlockArm_Gimbal_J60<=90.f&&StoreBlock_Time>600)
        {
			BlockArm_Task_State=BLOCKARM_TASK_INIT;
			return;
        }
        
        StoreBlock_Time++;
        break;

    case BLOCKARM_TASK_PICKUP_BLOCK://三区拿起存储的方块
                
//		if (BlockArm_Task_Change.BlockStore[1]==1)
//        {
//            if(MovementFlag[0]==0&&Traj_Arm.State==TRAJ_DONE)
//            {
//                Traj_Angle_Init(&Traj_Gimbal,MotorInput.BlockArm_Gimbal_J60, 115.f, 180.f, 2.f);
//				Traj_Angle_Init(&Traj_BlockArm_Joint3, MotorInput.BlockArm_Joint3_3508, -95.f, 180.f, 1.f);
//                Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,330.f, 400.f, 0.f, 0.f, 1000.f, 2000.f);

//				MovementFlag[0]=1;
//            }
//            else if(MovementFlag[0]==1&&MovementFlag[1]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
//            {
//				QD_Ctrl.BlockArm_Store0_Air_Pump_State=0;
//                QD_Ctrl.BlockArm_Store0_Solenoid_Valve_State=1;//打开存储电磁阀，喷气
//				Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,Arm_BackSolving.x,Arm_BackSolving.y-80.f, 0.f, 0.f, 500.f, 2000.f);
//                vTaskDelay(pdMS_TO_TICKS(100));
//                QD_Ctrl.BlockArm_Store0_Solenoid_Valve_State=0;//关闭存储电磁阀，停止喷气
//				vTaskDelay(pdMS_TO_TICKS(400));
//				MovementFlag[1]=1;
//            }
//			else if(MovementFlag[1]==1&&MovementFlag[2]==0&&Traj_Arm.State==TRAJ_DONE)
//            {
//				Gravity_BlockArm.k_Joint3=BlockArm_Joint3_GfWithBlock;
//                Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,Arm_BackSolving.x,Arm_BackSolving.y+80.f, 0.f, 0.f, 500.f, 2000.f);
//				Traj_Angle_Init(&Traj_Gimbal,MotorInput.BlockArm_Gimbal_J60, 90.f, 180.f, 0.5f);
//                vTaskDelay(pdMS_TO_TICKS(500));
//				Traj_Angle_Init(&Traj_BlockArm_Joint3, MotorInput.BlockArm_Joint3_3508, 10.f, 60.f, 3.f);
//                vTaskDelay(pdMS_TO_TICKS(2000));
//                Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,250.f, 400.f, 0.f, 0.f, 500.f, 2000.f);
//                vTaskDelay(pdMS_TO_TICKS(500));
//				Traj_Angle_Init(&Traj_Gimbal,MotorInput.BlockArm_Gimbal_J60, 0.f, 180.f, 1.f);
//				MovementFlag[2]=1;
//            }

//		    if (MovementFlag[2]==1&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)//记得加检查角度
//			{
//                BlockArm_Task_Change.BlockStore[0]=0;
//                BlockArm_Task_State=BLOCKARM_TASK_PUT_BLOCK;//进入下一个状态
//				return;
//			}
//        }
		if (BlockArm_Task_Change.BlockStore[0]==1)
        {
			if(MovementFlag[0]==0&&Traj_Arm.State==TRAJ_DONE)
            {
                Traj_Angle_Init(&Traj_Gimbal,MotorInput.BlockArm_Gimbal_J60, 180.f, 360.f, 0.8f);
				Traj_Angle_Init(&Traj_BlockArm_Joint3, MotorInput.BlockArm_Joint3_3508, -90.f, 720.f, 0.3f);
                Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,300.f, 400.f, 0.f, 0.f, 1000.f, 15000.f);

				MovementFlag[0]=1;
            }
            else if(MovementFlag[0]==1&&MovementFlag[1]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
            {
				QD_Ctrl.BlockArm_Store0_Air_Pump_State=0;
				Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,Arm_BackSolving.x,Arm_BackSolving.y-80.f, 0.f, 0.f, 1000.f, 10000.f);
                vTaskDelay(pdMS_TO_TICKS(200));
				QD_Ctrl.BlockArm_Store0_Solenoid_Valve_State=1;//打开存储电磁阀，喷气
				Gravity_BlockArm.k_Joint3=BlockArm_Joint3_GfWithBlock;
				vTaskDelay(pdMS_TO_TICKS(50));
                Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,Arm_BackSolving.x,Arm_BackSolving.y+80.f, 0.f, 0.f, 1000.f, 10000.f);
                vTaskDelay(pdMS_TO_TICKS(150));				
                QD_Ctrl.BlockArm_Store0_Solenoid_Valve_State=0;//关闭存储电磁阀，停止喷气
				vTaskDelay(pdMS_TO_TICKS(300));
				MovementFlag[1]=1;
            }
			else if(MovementFlag[1]==1&&MovementFlag[2]==0&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)
            {
	
				Traj_Angle_Init(&Traj_Gimbal,MotorInput.BlockArm_Gimbal_J60, 90.f, 360.f, 0.5f);
                vTaskDelay(pdMS_TO_TICKS(500));
				Traj_Angle_Init(&Traj_BlockArm_Joint3, MotorInput.BlockArm_Joint3_3508, 0.f, 180.f, 0.8f);
                vTaskDelay(pdMS_TO_TICKS(500));
                Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,250.f, 350.f, 0.f, 0.f, 1000.f, 10000.f);
				Traj_Angle_Init(&Traj_Gimbal,MotorInput.BlockArm_Gimbal_J60, 0.f, 180.f, 1.5f);
				MovementFlag[2]=1;
            }

		    if (MovementFlag[2]==1&&Traj_Gimbal.State==TRAJ_DONE&&Traj_Arm.State==TRAJ_DONE&&Traj_BlockArm_Joint3.State==TRAJ_DONE)//记得加检查角度
			{
                BlockArm_Task_Change.BlockStore[0]=0;
                BlockArm_Task_State=BLOCKARM_TASK_PUT_BLOCK;//进入下一个状态
				return;
			}
        }
        PickUpBlock_Time++;
        break;

    case BLOCKARM_TASK_PUT_BLOCK:
        
		
		if(BlockArm_Task_Change.Reach_Area3==2)
		{
			return;
		}
		if(MovementFlag[0]==0&&Traj_Arm.State==TRAJ_DONE)
        {
			if(BlockArm_Task_Change.Block_Height!=0)
			{
				Traj_Line_Init(&Traj_Arm, Arm_BackSolving.x, Arm_BackSolving.y,450.f, 350.f, 0.f, 0.f, 500.f, 2000.f);
			}
			MovementFlag[0]=1;
        }
        else if(MovementFlag[0]==1&&MovementFlag[1]==0&&Traj_Arm.State==TRAJ_DONE)
        {
			QD_Ctrl.BlockArm_Fetch_Air_Pump_State=0;//关闭机械臂气泵
			QD_Ctrl.BlockArm_Fetch_Solenoid_Valve_State=1;//打开机械臂电磁阀，喷气
			vTaskDelay(pdMS_TO_TICKS(200));
			QD_Ctrl.BlockArm_Fetch_Solenoid_Valve_State=0;//关闭机械臂电磁阀，停止喷气
            Gravity_BlockArm.k_Joint3=BlockArm_Joint3_GfNoBlock;			
			MovementFlag[1]=1;
			vTaskDelay(pdMS_TO_TICKS(1000));
        }
        if (MovementFlag[1]==1)
        {
            BlockArm_Task_State=BLOCKARM_TASK_INIT;
			return;
        }
        
        PutBlock_Time++;
        break;

    default:
        break;
    }
}

void Air_Pump_Control(void)
{ // 控制气泵的函数，根据不同状态控制不同的气泵动作
	//新气动板
//    AirOperaterCtrlBuf[0] =	QD_Ctrl.PoleArm_Upper_Solenoid_Valve_State;    	  	
//    AirOperaterCtrlBuf[1] =	QD_Ctrl.PoleArm_Lower_Solenoid_Valve_State;      
//    AirOperaterCtrlBuf[2] =	QD_Ctrl.BlockArm_Fetch_Solenoid_Valve_State;  
//    AirOperaterCtrlBuf[3] =	QD_Ctrl.BlockArm_Store0_Air_Pump_State;
//    AirOperaterCtrlBuf[4] =	QD_Ctrl.BlockArm_Store1_Air_Pump_State;
//    AirOperaterCtrlBuf[5] =	QD_Ctrl.BlockArm_Fetch_Air_Pump_State;
//	AirOperaterCtrlBuf[6] =	QD_Ctrl.BlockArm_Store1_Solenoid_Valve_State;
//    AirOperaterCtrlBuf[7] =	QD_Ctrl.BlockArm_Store0_Solenoid_Valve_State;    
	//
	AirOperaterCtrlBuf[0] =	QD_Ctrl.BlockArm_Store0_Air_Pump_State;	//4	
    AirOperaterCtrlBuf[1] =	QD_Ctrl.BlockArm_Store1_Air_Pump_State; //5
    AirOperaterCtrlBuf[2] =	QD_Ctrl.BlockArm_Fetch_Air_Pump_State;  //6
    AirOperaterCtrlBuf[3] =	QD_Ctrl.BlockArm_Store1_Solenoid_Valve_State; //7
    AirOperaterCtrlBuf[4] =	QD_Ctrl.BlockArm_Store0_Solenoid_Valve_State; //8
    AirOperaterCtrlBuf[5] =	QD_Ctrl.BlockArm_Fetch_Solenoid_Valve_State;//3
	AirOperaterCtrlBuf[6] =	QD_Ctrl.PoleArm_Upper_Solenoid_Valve_State; 
    AirOperaterCtrlBuf[7] =	QD_Ctrl.PoleArm_Lower_Solenoid_Valve_State;    
}
 
//对接微调函数
void  PoleArm_Adjustment(void)
{
    if ((p_vision_data.x2==0&&p_vision_data.y2==0&&p_vision_data.yaw2==0&&p_vision_data.yaw3==0&&p_vision_data.distance==0)||fabsf(PoleArm_Adjustment_2006.angle)>=90)
    {
		MotorInput.PoleArm_Adjustment_2006=0;
        return;//如果视觉数据无效，则不进行调整
    }
    float error = p_vision_data.y2+QR_OFFSET_X;
	PID_Calc(&PoleArm_Adjustment_PID, 0, error);
	MotorInput.PoleArm_Adjustment_2006=-PoleArm_Adjustment_PID.fpU;
}

//判断是否到达目标位置的函数，利用位置容差判断
uint8_t PoleArm_State_Judge(float Joint1, float Joint2, float FrictionWheel)
{
    if(fabs(PoleArm_Joint1_3508.angle - MotorInput.PoleArm_Joint1_3508) < Joint1 &&
       fabs(PoleArm_Joint2_2006.angle - MotorInput.PoleArm_Joint2_2006) < Joint2 &&
       fabs(PoleArm_FrictionWheel_3508.anglev - MotorInput.PoleArm_FrictionWheel_3508) < FrictionWheel)
    {
        return 1; // 已经到达目标位置
    }
    else
    {
        return 0; // 还没有到达目标位置
    }
}

void PoleArm_Task_TD_Calc(void)//在三个move状态下计算跟随目标的函数并给motorinput赋值
{
  switch(PoleArm_Task_State)
  {
    case POLEARM_TASK_INIT:// 在初始化状态下，设置初始位置

    MotorInput.PoleArm_Joint1_3508=PoleArm_Joint1_3508_Init_Position;
    MotorInput.PoleArm_Joint2_2006=PoleArm_Joint2_2006_Init_Position;
    MotorInput.PoleArm_FrictionWheel_3508=PoleArm_FrictionWheel_3508_Init_Position;
    break;

    case POLEARM_TASK_MOVE_TO_FETCH_POSITION:// 在移动到取杆位置的状态下，计算跟随目标

    // MotorInput.PoleArm_Joint1_3508=PoleArm_Joint1_3508_Fetch_Position;
    // MotorInput.PoleArm_FrictionWheel_3508=PoleArm_FrictionWheel_3508_Fetch_Position;
    // if(PoleArm_Task_Timer.POLEARM_TASK_MOVE_TO_FETCH_POSITION > 350)//在任务执行时间超过0.35s时，开始移动第二个关节，以增加任务的鲁棒性
    // {
    //     MotorInput.PoleArm_Joint2_2006=PoleArm_Joint2_2006_Fetch_Position;
    // }

    PoleArm_Joint1_3508_TD.aim = PoleArm_Joint1_3508_Fetch_Position;

    if (PoleArm_Task_Timer.POLEARM_TASK_MOVE_TO_FETCH_POSITION > 1000)//在任务执行时间超过1s时，开始移动第一个关节，以增加任务的鲁棒性
    {
		PoleArm_Joint2_2006_TD.aim = PoleArm_Joint2_2006_Fetch_Position;
    }
	
	CalTD(&PoleArm_Joint1_3508_TD);
	CalTD(&PoleArm_Joint2_2006_TD);
	
	MotorInput.PoleArm_Joint1_3508=PoleArm_Joint1_3508_TD.x1;
	MotorInput.PoleArm_Joint2_2006=PoleArm_Joint2_2006_TD.x1;
    MotorInput.PoleArm_FrictionWheel_3508=PoleArm_FrictionWheel_3508_Place_Position;

    break;
  
    case POLEARM_TASK_MOVE_TO_PLACE_POSITION:// 在移动到放杆位置的状态下，计算跟随目标
	
	PoleArm_Joint2_2006_TD.aim = PoleArm_Joint2_2006_Place_Position;

    if (PoleArm_Task_Timer.POLEARM_TASK_MOVE_TO_PLACE_POSITION > 1000)//在任务执行时间超过1s时，开始移动第一个关节，以增加任务的鲁棒性
    {
		PoleArm_Joint1_3508_TD.aim = PoleArm_Joint1_3508_Place_Position;
    }
	
	CalTD(&PoleArm_Joint1_3508_TD);
	CalTD(&PoleArm_Joint2_2006_TD);
	
	MotorInput.PoleArm_Joint1_3508=PoleArm_Joint1_3508_TD.x1;
	MotorInput.PoleArm_Joint2_2006=PoleArm_Joint2_2006_TD.x1;
    MotorInput.PoleArm_FrictionWheel_3508=PoleArm_FrictionWheel_3508_Place_Position;
    break;
  
    case POLEARM_TASK_ALREADY_TO_FETCH_POSITION:// 在已经到达取杆位置的状态下，可以执行取杆操作
    MotorInput.PoleArm_Joint1_3508=PoleArm_Joint1_3508_Fetch_Position;
    MotorInput.PoleArm_Joint2_2006=PoleArm_Joint2_2006_Fetch_Position;
    MotorInput.PoleArm_FrictionWheel_3508=PoleArm_FrictionWheel_3508_Fetch_Position;
    break;

    case POLEARM_TASK_ALREADY_TO_PLACE_POSITION:// 在已经到达放杆位置的状态下，可以执行放杆操作
    MotorInput.PoleArm_Joint1_3508=PoleArm_Joint1_3508_Place_Position;
    MotorInput.PoleArm_Joint2_2006=PoleArm_Joint2_2006_Place_Position;      
    MotorInput.PoleArm_FrictionWheel_3508=PoleArm_FrictionWheel_3508_Place_Position;
    break;

    case POLEARM_TASK_ALREADY_TO_CONNECT_POSITION:// 在已经到达连接位置的状态下，可以执行连接操作
    MotorInput.PoleArm_Joint1_3508=PoleArm_Joint1_3508_Connect_Position;
    MotorInput.PoleArm_Joint2_2006=PoleArm_Joint2_2006_Connect_Position;
    MotorInput.PoleArm_FrictionWheel_3508=PoleArm_FrictionWheel_3508_Connect_Position;
    break;

    case POLEARM_TASK_ALREADY_TO_STORE_POSITION:// 在已经到达存储位置的状态下，可以执行存储操作
    MotorInput.PoleArm_Joint1_3508=PoleArm_Joint1_3508_Store_Position;
    MotorInput.PoleArm_Joint2_2006=PoleArm_Joint2_2006_Store_Position;
    MotorInput.PoleArm_FrictionWheel_3508=PoleArm_FrictionWheel_3508_Store_Position;
    break;

    case POLEARM_TASK_ERROR:// 在任务异常状态下，可以执行错误处理、重置等操作
	PoleArm_Joint1_3508_TD.aim = PoleArm_Joint1_3508_Error_Position;

    if (PoleArm_Task_Timer.POLEARM_TASK_MOVE_TO_FETCH_POSITION > 200)//在任务执行时间超过1s时，开始移动第一个关节，以增加任务的鲁棒性
    {
		PoleArm_Joint2_2006_TD.aim = PoleArm_Joint2_2006_Error_Position;
    }
	
	CalTD(&PoleArm_Joint1_3508_TD);
	CalTD(&PoleArm_Joint2_2006_TD);
	
	MotorInput.PoleArm_Joint1_3508=PoleArm_Joint1_3508_TD.x1;
	MotorInput.PoleArm_Joint2_2006=PoleArm_Joint2_2006_TD.x1;
    MotorInput.PoleArm_FrictionWheel_3508=PoleArm_FrictionWheel_3508_Place_Position;
    break;

    default:
    break;
  }
  
}

void PoleArm_Logic(void)
{   
    if (PoleArm_Task_State==POLEARM_TASK_MANUAL_CONTROL)
    {
        if (fabs(Communicate_Js_Value.usJsLeft_Y - LEFT_JS_Y_MID) >= 500)
        {
            if (Communicate_Js_Value.usJsLeft_Y > LEFT_JS_Y_MID)
            {
                MotorInput.PoleArm_FrictionWheel_3508 = PoleArm_FrictionWheel_3508_Manual_Position*(Communicate_Js_Value.usJsLeft_Y -LEFT_JS_Y_MID)/(LEFT_JS_Y_MAX-LEFT_JS_Y_MID);//手动控制摩擦轮，左右摇杆控制正转，摇杆越远转速越快
            }
            else 
            {
                MotorInput.PoleArm_FrictionWheel_3508 = PoleArm_FrictionWheel_3508_Manual_Position*(Communicate_Js_Value.usJsLeft_Y -LEFT_JS_Y_MID)/(LEFT_JS_Y_MID-LEFT_JS_Y_MIN);//手动控制摩擦轮，左右摇杆控制正转，摇杆越远转速越快
            }
        }
		else
		{
			MotorInput.PoleArm_FrictionWheel_3508=0;
		}

        return;//在手动控制状态下，不执行自动任务逻辑，直接返回
    }

    if(PoleArm_Task_State != PoleArm_Task_State_Pre)
    {
        PoleArm_Task_Timer.PoleArm_Task_Ticks = 0;//状态切换时，重置任务计时器
    }

    PoleArm_Task_State_Pre = PoleArm_Task_State;

    switch (PoleArm_Task_State)
    {
        case POLEARM_TASK_INIT:// 初始化任务，设置初始位置等

            PoleArm_Task_Timer.POLEARM_TASK_MOVE_TO_FETCH_POSITION=0;
            PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_FETCH_POSITION=0;
            PoleArm_Task_Timer.POLEARM_TASK_MOVE_TO_PLACE_POSITION=0;
            PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_PLACE_POSITION=0;
            PoleArm_Task_Timer.POLEARM_TASK_MOVE_TO_CONNECT_POSITION=0;
            PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_CONNECT_POSITION=0;
            PoleArm_Task_Timer.POLEARM_TASK_MOVE_TO_STORE_POSITION=0;
            PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_STORE_POSITION=0;
            PoleArm_Task_Timer.POLEARM_TASK_ERROR=0;
            if (PoleArm_Task_Change.Fetch_Pole==1)
            {
                PoleArm_Task_State=POLEARM_TASK_MOVE_TO_FETCH_POSITION;
                PoleArm_Task_Change.Fetch_Pole=0;
            }
            
            break;

        case POLEARM_TASK_MOVE_TO_FETCH_POSITION:
            //在移动到取杆位置的状态下，计算跟随目标
            PoleArm_Task_Timer.POLEARM_TASK_MOVE_TO_FETCH_POSITION = PoleArm_Task_Timer.PoleArm_Task_Ticks;
            // 如果已经到达取杆位置，切换状态
            if (PoleArm_Task_Timer.POLEARM_TASK_MOVE_TO_FETCH_POSITION>=2000)
            {
                PoleArm_Task_State = POLEARM_TASK_ALREADY_TO_FETCH_POSITION;
            }

            else if (PoleArm_Task_Timer.POLEARM_TASK_MOVE_TO_FETCH_POSITION > MAX_POLEARM_TASK_PERIOD) 
            {
                // 如果任务执行时间超过最大周期，切换到错误状态
                PoleArm_Task_State = POLEARM_TASK_ERROR;
            }
            break;

        case POLEARM_TASK_ALREADY_TO_FETCH_POSITION:
            // 已经到达取杆位置，可以执行取杆操作

			PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_FETCH_POSITION = PoleArm_Task_Timer.PoleArm_Task_Ticks;

            if (PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_FETCH_POSITION<=100)
            {
                QD_Ctrl.PoleArm_Upper_Solenoid_Valve_State=1;
            }
			
			if (PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_FETCH_POSITION==300)
            {
                PoleArm_Joint1_3508_Fetch_Position-=30;
				PoleArm_Joint2_2006_Fetch_Position-=0;
				PoleArm_Joint2_2006_TD.x1=PoleArm_Joint2_2006_Fetch_Position;
				PoleArm_Joint1_3508_TD.x1=PoleArm_Joint1_3508_Fetch_Position;
				PoleArm_Joint2_2006_TD.aim=PoleArm_Joint2_2006_Fetch_Position;
				PoleArm_Joint1_3508_TD.aim=PoleArm_Joint1_3508_Fetch_Position;
            }
			
			if (PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_FETCH_POSITION>=300&&PoleArm_Task_Change.Place_Pole==1)
			{
                PoleArm_Task_State=POLEARM_TASK_MOVE_TO_PLACE_POSITION;
                PoleArm_Task_Change.Place_Pole=0;
            }
            
            // 执行取杆操作后，切换状态
            
            break;

        case POLEARM_TASK_MOVE_TO_PLACE_POSITION:
            //在移动到放杆位置的状态下，计算跟随目标
            PoleArm_Task_Timer.POLEARM_TASK_MOVE_TO_PLACE_POSITION = PoleArm_Task_Timer.PoleArm_Task_Ticks;
            // 如果已经到达放杆位置，切换状态
            if (PoleArm_State_Judge(8.f,6.f,5.f)&&PoleArm_Task_Timer.POLEARM_TASK_MOVE_TO_PLACE_POSITION>=3000) 
			{
                PoleArm_Task_State = POLEARM_TASK_ALREADY_TO_PLACE_POSITION;
            }
            break;

        case POLEARM_TASK_ALREADY_TO_PLACE_POSITION:
            // 已经到达放杆位置，可以执行放杆操作
            if (PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_PLACE_POSITION<=100)
            {
                QD_Ctrl.PoleArm_Lower_Solenoid_Valve_State=1;
            }

            if (PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_PLACE_POSITION>=300&&PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_PLACE_POSITION<=300)
            {
                QD_Ctrl.PoleArm_Upper_Solenoid_Valve_State=0;
            }
			
			if (PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_PLACE_POSITION>=400&&PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_PLACE_POSITION<=900)
            {
                PoleArm_FrictionWheel_3508_Place_Position=180.f;
            }
			
			if (PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_PLACE_POSITION>=900&&PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_PLACE_POSITION<=1000)
            {
                PoleArm_FrictionWheel_3508_Place_Position=-0.f;
            }
            
            if (PoleArm_Task_Change.Connect_Pole==1)
            {
                PoleArm_Task_State=POLEARM_TASK_ALREADY_TO_CONNECT_POSITION;
                PoleArm_Task_Change.Connect_Pole=0;
            }
            
            PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_PLACE_POSITION = PoleArm_Task_Timer.PoleArm_Task_Ticks;
            // 执行放杆操作后，可以选择继续循环或者结束任务
            break;

            
        case POLEARM_TASK_ALREADY_TO_CONNECT_POSITION:
            // 已经到达连接位置，可以执行连接操作
            
            if (PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_CONNECT_POSITION>=1800)
            {
                PoleArm_FrictionWheel_3508_Connect_Position=-0.5;
            }
            if (PoleArm_Task_Change.Store_Pole==1)
            {
                PoleArm_Task_State=POLEARM_TASK_ALREADY_TO_STORE_POSITION;
                PoleArm_Task_Change.Store_Pole=0;
            }
            
            PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_CONNECT_POSITION = PoleArm_Task_Timer.PoleArm_Task_Ticks;
            // 执行连接操作后，可以选择继续循环或者结束任务
            break;

        case POLEARM_TASK_ALREADY_TO_STORE_POSITION:
            // 已经到达存储位置，可以执行存储操作
            
            if (PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_STORE_POSITION>=1000)
            {
                PoleArm_FrictionWheel_3508_Store_Position=0.5;
            }
            
            PoleArm_Task_Timer.POLEARM_TASK_ALREADY_TO_STORE_POSITION = PoleArm_Task_Timer.PoleArm_Task_Ticks;
            // 执行存储操作后，可以选择继续循环或者结束任务
            break;

        case POLEARM_TASK_ERROR:
            // 处理任务异常情况，例如错误处理、重置等
		    PoleArm_Task_Timer.POLEARM_TASK_ERROR = PoleArm_Task_Timer.PoleArm_Task_Ticks;
            // 如果已经到达取杆位置，切换状态
			
			if(PoleArm_Task_Timer.POLEARM_TASK_ERROR==0)
			{
				
				PoleArm_Joint2_2006_TD.x1=MotorInput.PoleArm_Joint2_2006;
				PoleArm_Joint1_3508_TD.x1=MotorInput.PoleArm_Joint1_3508;
				PoleArm_Joint2_2006_TD.aim=MotorInput.PoleArm_Joint2_2006;
				PoleArm_Joint1_3508_TD.aim=MotorInput.PoleArm_Joint1_3508;
				PoleArm_Joint1_3508_Fetch_Position=365.f;
				PoleArm_Joint2_2006_Fetch_Position=-15.f;
				QD_Ctrl.PoleArm_Upper_Solenoid_Valve_State=0;
				QD_Ctrl.PoleArm_Lower_Solenoid_Valve_State=0;
			}
            if (PoleArm_Task_Timer.POLEARM_TASK_ERROR>=1500)
            {
                PoleArm_Task_State = POLEARM_TASK_INIT;
            }
			
            break;

        default:
            break;
    }
}


void Communicate_Task(void)
{
    USART1_Receive_Data(Communicate_KEY);
    USART1_Transmit_Data();
}

void USART1_Receive_Data(uint8_t Communicate_KEY)
{
    //在二区时，按位置确定物块高度
    if ((Communicate_KEY==1||Communicate_KEY==3||Communicate_KEY==5||Communicate_KEY==7||Communicate_KEY==15||Communicate_KEY==22)&&BlockArm_Task_Change.Area_State==2)
    {
        BlockArm_Task_Change.Block_Height=1;
    }
    else if ((Communicate_KEY==2||Communicate_KEY==9||Communicate_KEY==11||Communicate_KEY==17||Communicate_KEY==19||Communicate_KEY==21||Communicate_KEY==23)&&BlockArm_Task_Change.Area_State==2)
    {
        BlockArm_Task_Change.Block_Height=2;
    }
    else if (Communicate_KEY==13&&BlockArm_Task_Change.Area_State==2)
    {
        BlockArm_Task_Change.Block_Height=3;
    }
    //根据按键切换所处区域
    else if (Communicate_KEY==18)
    {
        BlockArm_Task_Change.Area_State=1;
    }
    else if (Communicate_KEY==14)
    {
        BlockArm_Task_Change.Area_State=2;
    }
    else if (Communicate_KEY==10)//三区
    {
        BlockArm_Task_Change.Area_State=3;
		QD_Ctrl.BlockArm_Fetch_Air_Pump_State=1;
		QD_Ctrl.BlockArm_Store1_Air_Pump_State=1;
		QD_Ctrl.BlockArm_Store0_Air_Pump_State=1;
		BlockArm_Task_Change.BlockStore[0]=1;//预装一个方块
    }
	else if (Communicate_KEY==6)//    `三区重试
    {
        BlockArm_Task_Change.Area_State=3;
		QD_Ctrl.BlockArm_Fetch_Air_Pump_State=1;
		QD_Ctrl.BlockArm_Store1_Air_Pump_State=1;
		QD_Ctrl.BlockArm_Store0_Air_Pump_State=1;
		BlockArm_Task_Change.BlockStore[0]=1;//预装一个方块
    }

    //一区任务
    else if (Communicate_KEY==4&&BlockArm_Task_Change.Area_State==1)
    {
        PoleArm_Task_Change.Fetch_Pole=1;
		vTaskDelay(pdMS_TO_TICKS(150));
    }
    else if (Communicate_KEY==8&&BlockArm_Task_Change.Area_State==1)
    {
        PoleArm_Task_Change.Place_Pole=1;
		vTaskDelay(pdMS_TO_TICKS(150));
    }
    else if (Communicate_KEY==12&&BlockArm_Task_Change.Area_State==1)
    {
        PoleArm_Task_Change.Connect_Pole=1;
		vTaskDelay(pdMS_TO_TICKS(150));
    }
    else if (Communicate_KEY==16&&BlockArm_Task_Change.Area_State==1)
    {
        PoleArm_Task_Change.Store_Pole=1;
		vTaskDelay(pdMS_TO_TICKS(150));
    }
    else if (Communicate_KEY==20&&BlockArm_Task_Change.Area_State==1)
    {
		
    }
    //二区任务
    else if (Communicate_KEY==4&&BlockArm_Task_Change.Area_State==2&&BlockArm_Task_State==BLOCKARM_TASK_INIT)
    {
		BlockArm_Task_Change.BlockStore_State=1;
		BlockArm_Task_Change.BlockStore[0]=0;
        BlockArm_Task_Change.Ready=1;
		vTaskDelay(pdMS_TO_TICKS(300));
    }
    else if (Communicate_KEY==8&&BlockArm_Task_Change.Area_State==2)
    {		
		BlockArm_Task_Change.BlockStore_State=2;
		BlockArm_Task_Change.BlockStore[1]=0;
        BlockArm_Task_Change.Ready=1;
		vTaskDelay(pdMS_TO_TICKS(300));
    }
    else if (Communicate_KEY==12&&BlockArm_Task_Change.Area_State==2)
    {
		BlockArm_Task_Change.BlockStore_State=3;
		BlockArm_Task_Change.BlockStore[2]=0;
        BlockArm_Task_Change.Ready=1;
		vTaskDelay(pdMS_TO_TICKS(300));
    }
    else if (Communicate_KEY==16&&BlockArm_Task_Change.Area_State==2)
    {
    }
    else if (Communicate_KEY==20&&BlockArm_Task_Change.Area_State==2)
    {

    }

    //三区任务
    else if (Communicate_KEY==4&&(BlockArm_Task_Change.Reach_Area3==2||BlockArm_Task_State==BLOCKARM_TASK_INIT||BlockArm_Task_State==BLOCKARM_TASK_RETRACT_ARM))
    {
		if(BlockArm_Task_Change.Reach_Area3!=2)
		{
			BlockArm_Task_Change.Ready=2;
		}
		else
		{
			BlockArm_Task_Change.Reach_Area3=3;
		}
    }
    else if (Communicate_KEY==8&&BlockArm_Task_Change.Area_State==3&&BlockArm_Task_State==BLOCKARM_TASK_INIT)
    {
		BlockArm_Task_Change.BlockStore_State=3;
        BlockArm_Task_Change.Block_Height=0;
        BlockArm_Task_Change.Ready=1;
		BlockArm_Task_Change.BlockStore[0]=1;
		BlockArm_Task_Change.BlockStore[1]=1;
    }
    else if (Communicate_KEY==12&&BlockArm_Task_Change.Area_State==3)
    {
    }
    else if (Communicate_KEY==16&&BlockArm_Task_Change.Area_State==3)
    {
    }
    else if (Communicate_KEY==20&&BlockArm_Task_Change.Area_State==3)
    {
    }
	else if(BlockArm_Task_Change.Reach_Area3==1&&BlockArm_Task_Change.Area_State==3)
	{
		BlockArm_Task_Change.Ready=2;
		BlockArm_Task_Change.Reach_Area3=2;
	}

    //通用任务
    else if (Communicate_KEY==25)
    {
        if (PoleArm_Task_State!=POLEARM_TASK_MANUAL_CONTROL)
        {
            PoleArm_Task_State=POLEARM_TASK_MANUAL_CONTROL;
            vTaskDelay(pdMS_TO_TICKS(300));
			return;
        }

        if (PoleArm_Task_State==POLEARM_TASK_MANUAL_CONTROL)
        {
            PoleArm_Task_State=PoleArm_Task_State_Pre;
            vTaskDelay(pdMS_TO_TICKS(300));
			return;
        }
    }
	else if (Communicate_KEY==31)
    {
		if(BlockArm_Task_Change.Area_State==2||BlockArm_Task_Change.Area_State==3)
		{
			BlockArm_Task_State=BLOCKARM_TASK_INIT;
			QD_Ctrl.BlockArm_Fetch_Air_Pump_State=0;
			BlockArm_Task_Change.BlockStore[2]=0;
		}
		else if(BlockArm_Task_Change.Area_State==1)
		{
			PoleArm_Task_State=POLEARM_TASK_ERROR;
		}
    }
}

void USART1_Transmit_Data(void)
{
    HAL_UART_Transmit_DMA(&huart1, AirCtrl, 3);
}

void Motor_CAN_Send(void)
{
	if(MotorCtrl_Flag==0)
    {
		//用TD计算取杆机械臂各个电机的Input
		PoleArm_Task_TD_Calc();
		//解算出取块机械臂各个电机的Input
		BlockArm_InputSolve(&Arm_BackSolving,&MotorInput);


	}
	else if(MotorCtrl_Flag==1)
	{
		MotorInput.BlockArm_Joint1_J60=BlockArm_Joint1_J60Data.position_;
		MotorInput.BlockArm_Joint2_J60=BlockArm_Joint2_J60Data.position_;
		MotorInput.BlockArm_Joint3_3508=BlockArm_Joint3_3508_PID.outer.fpFB;
		MotorInput.BlockArm_Gimbal_J60=BlockArm_Gimbal_J60Data.position_;
		MotorInput.PoleArm_Joint2_2006=PoleArm_Joint2_2006.angle;
		MotorInput.PoleArm_Joint1_3508=PoleArm_Joint1_3508.angle;
	}
	//控制气泵和电磁阀
	Air_Pump_Control();

    //对电机的Input值进行限幅
    MotorInput.BlockArm_Gimbal_J60=ClipFloat(MotorInput.BlockArm_Gimbal_J60,-5,180);//由上往下看，J60正转，云台顺时针转
    MotorInput.BlockArm_Joint1_J60=ClipFloat(MotorInput.BlockArm_Joint1_J60,-150,0);//J60旋转方向与机械臂旋转方向相反，J60正转，大臂逆时针转
    MotorInput.BlockArm_Joint2_J60=ClipFloat(MotorInput.BlockArm_Joint2_J60,-30,90);//J60旋转方向与机械臂旋转方向相同，J60正转，小臂顺时针转
    MotorInput.BlockArm_Joint3_3508=ClipFloat(MotorInput.BlockArm_Joint3_3508,-95,90);//3508旋转方向与机械臂旋转方向相同，3508正转，第三节机械臂顺时针转
    MotorInput.PoleArm_FrictionWheel_3508=ClipFloat(MotorInput.PoleArm_FrictionWheel_3508,-2880,2880);//正转时摩擦轮顺时针转
    MotorInput.PoleArm_Joint1_3508=ClipFloat(MotorInput.PoleArm_Joint1_3508,0,540);//3508旋转方向与机械臂旋转方向相同,3508正转,大臂逆时针转
    MotorInput.PoleArm_Joint2_2006=ClipFloat(MotorInput.PoleArm_Joint2_2006,-540,180);//2006旋转方向与机械臂旋转方向相同,2006正转,小臂逆时针转

    //计算重力前馈
    Gravity_BlockArm.q[0]=MotorInput.BlockArm_Joint1_J60;//J60旋转方向与机械臂旋转方向相反
    Gravity_BlockArm.q[1]=-MotorInput.BlockArm_Joint2_J60;//J60旋转方向与机械臂旋转方向相同
    Gravity_BlockArm.q[2]=MotorInput.BlockArm_Joint3_3508;//3508旋转方向与机械臂旋转方向相同
    Gravity_PoleArm.q[0]=-MotorInput.PoleArm_Joint1_3508/PoleArm_Joint1_uiGearRatio;//3508旋转方向与机械臂旋转方向相同
    Gravity_PoleArm.q[1]=-MotorInput.PoleArm_Joint2_2006/PoleArm_Joint2_uiGearRatio;//3508旋转方向与机械臂旋转方向相同
    BlockArm_GravityFeedforward(&Gravity_BlockArm);
    PoleArm_GravityFeedforward(&Gravity_PoleArm);
    //CAN1发送
	DJI_MotorCtrl();
		
    AirCtrl[0]=Bin_Array_To_u8(AirOperaterCtrlBuf);//解算出发送给气动板控制电磁阀的一位uint8数组
	AirCtrl[1]=Bin_Array_To_u8(&AirOperaterCtrlBuf[6]);//解算出发送给气动板控制电磁阀的一位uint8数组
//	QD_CANx_SendstdData(&hcan1,0x300,AirCtrl,1);//将一位数组发送给气动板

//	Bin_Array_To_AirCtrl_New(AirOperaterCtrlBuf, AirCtrl);
	
    //CAN2发送
    YSC_MotorCtrl(&BlockArm_Joint1_J60CMD,&BlockArm_Joint1_J60Data);
    YSC_MotorCtrl(&BlockArm_Joint2_J60CMD,&BlockArm_Joint2_J60Data);
    YSC_MotorCtrl(&BlockArm_Gimbal_J60CMD,&BlockArm_Gimbal_J60Data);
    
}
