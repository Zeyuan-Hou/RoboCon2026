#include "CAN_Bsp.h"


/// @brief CAN初始化
/// @param 无
void CAN_BSP_Init(void)
{
    CAN1_FILTER_CONFIG();
	CAN2_FILTER_CONFIG();
    HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
}

/// @brief CAN1过滤器初始化
/// @param 无 
void CAN1_FILTER_CONFIG(void)
{
    CAN_FilterTypeDef CAN_FilterConfigStructure;
    CAN_FilterConfigStructure.FilterBank=0;//起始滤波器，CAN2对应14
    CAN_FilterConfigStructure.FilterFIFOAssignment=CAN_FILTER_FIFO0;//滤波器地址，对应中断
    CAN_FilterConfigStructure.FilterIdHigh=0x0000;//接收的ID
    CAN_FilterConfigStructure.FilterIdLow=0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh=0x0000;//掩码ID，放弃不要的ID
    CAN_FilterConfigStructure.FilterMaskIdLow=0x0000;
    CAN_FilterConfigStructure.FilterMode=CAN_FILTERMODE_IDMASK;//滤波器掩码模式
    CAN_FilterConfigStructure.FilterScale=CAN_FILTERSCALE_32BIT;//32位滤波
    CAN_FilterConfigStructure.SlaveStartFilterBank=14;//0至14-1个滤波器归CAN1，其余归CAN2
    CAN_FilterConfigStructure.FilterActivation=ENABLE;

    HAL_CAN_ConfigFilter(&hcan1,&CAN_FilterConfigStructure);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
}

/// @brief CAN2过滤器初始化
/// @param 无 
void CAN2_FILTER_CONFIG(void)
{
    CAN_FilterTypeDef CAN_FilterConfigStructure;
    CAN_FilterConfigStructure.FilterBank=14;//起始滤波器，CAN2对应14
    CAN_FilterConfigStructure.FilterFIFOAssignment=CAN_FILTER_FIFO0;//滤波器地址，对应中断
    CAN_FilterConfigStructure.FilterIdHigh=0x0000;//接收的ID
    CAN_FilterConfigStructure.FilterIdLow=0x0000;
    CAN_FilterConfigStructure.FilterMaskIdHigh=0x0000;//掩码ID，放弃不要的ID
    CAN_FilterConfigStructure.FilterMaskIdLow=0x0000;
    CAN_FilterConfigStructure.FilterMode=CAN_FILTERMODE_IDMASK;//滤波器掩码模式
    CAN_FilterConfigStructure.FilterScale=CAN_FILTERSCALE_32BIT;//32位滤波
    CAN_FilterConfigStructure.SlaveStartFilterBank=28;//0至14-1个滤波器归CAN1，其余归CAN2
    CAN_FilterConfigStructure.FilterActivation=ENABLE;

    HAL_CAN_ConfigFilter(&hcan2,&CAN_FilterConfigStructure);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1)
    {
        CAN_RxHeaderTypeDef RxHeader;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxMsg_CAN1);
        System_Monitor.Cnt_Can_Receive[0]++;
        switch (RxHeader.StdId)
        {
        case 0x201: // 取杆机械臂第二个关节的2006
            PoleArm_Joint2_2006.EncoderNum = DJI_GetEncoderNumber(&PoleArm_Joint2_2006, RxMsg_CAN1);
            PoleArm_Joint2_2006.encoder_speed = DJI_GetSpeed(&RxHeader, RxMsg_CAN1);
            DJI_Abs_Encoder_Process(&PoleArm_Joint2_2006.motor_encoder, PoleArm_Joint2_2006.EncoderNum);
            PoleArm_Joint2_2006.angle = PoleArm_Joint2_2006.motor_encoder.siSumValue / (float)8192 * 360.f / (float)M2006_uiGearRatio;
            PoleArm_Joint2_2006.anglev = PoleArm_Joint2_2006.encoder_speed / (float)M2006_uiGearRatio;
            System_Monitor.Cnt_PoleArm_Joint2_2006_Receive++;
            break;

        case 0x202: // 取杆机械臂第一个关节的3508
            PoleArm_Joint1_3508.EncoderNum = DJI_GetEncoderNumber(&PoleArm_Joint1_3508, RxMsg_CAN1);
            PoleArm_Joint1_3508.encoder_speed = DJI_GetSpeed(&RxHeader, RxMsg_CAN1);
            DJI_Abs_Encoder_Process(&PoleArm_Joint1_3508.motor_encoder, PoleArm_Joint1_3508.EncoderNum);
            PoleArm_Joint1_3508.angle = PoleArm_Joint1_3508.motor_encoder.siSumValue / (float)8192 * 360.f / (float)M3508_uiGearRatio;
            PoleArm_Joint1_3508.anglev = PoleArm_Joint1_3508.encoder_speed / (float)M3508_uiGearRatio;
            System_Monitor.Cnt_PoleArm_Joint1_3508_Receive++;
            break;

		
		
		
		
        case 0x203: // 存杆摩擦轮的3508
            PoleArm_FrictionWheel_3508.EncoderNum = DJI_GetEncoderNumber(&PoleArm_FrictionWheel_3508, RxMsg_CAN1);
            PoleArm_FrictionWheel_3508.encoder_speed = DJI_GetSpeed(&RxHeader, RxMsg_CAN1);
            DJI_Abs_Encoder_Process(&PoleArm_FrictionWheel_3508.motor_encoder, PoleArm_FrictionWheel_3508.EncoderNum);
            PoleArm_FrictionWheel_3508.angle = PoleArm_FrictionWheel_3508.motor_encoder.siSumValue / (float)8192 * 360.f/(float)M3508_uiGearRatio_Longer ;
//            PoleArm_FrictionWheel_3508_LFP.in=PoleArm_FrictionWheel_3508.encoder_speed;
//            LpFilter(&PoleArm_FrictionWheel_3508_LFP);
            PoleArm_FrictionWheel_3508.anglev = PoleArm_FrictionWheel_3508.encoder_speed/(float)M3508_uiGearRatio_Longer;
            System_Monitor.Cnt_PoleArm_FrictionWheel_3508_Receive++;
            break;

        case 0x204: // 取块机械臂第三个关节的3508
            BlockArm_Joint3_3508.EncoderNum = DJI_GetEncoderNumber(&BlockArm_Joint3_3508, RxMsg_CAN1);
            BlockArm_Joint3_3508.encoder_speed = DJI_GetSpeed(&RxHeader, RxMsg_CAN1);
            DJI_Abs_Encoder_Process(&BlockArm_Joint3_3508.motor_encoder, BlockArm_Joint3_3508.EncoderNum);
            BlockArm_Joint3_3508.angle = BlockArm_Joint3_3508.motor_encoder.siSumValue / (float)8192 * 360.f / (float)M3508_uiGearRatio/BlockArm_Joint3_uiGearRatio;
            BlockArm_Joint3_3508.anglev = BlockArm_Joint3_3508.encoder_speed / (float)M3508_uiGearRatio/BlockArm_Joint3_uiGearRatio;
            System_Monitor.Cnt_BlockArm_Joint3_3508_Receive++;
            break;

        case 0x205: //对接微调装置的2006
            PoleArm_Adjustment_2006.EncoderNum = DJI_GetEncoderNumber(&PoleArm_Adjustment_2006, RxMsg_CAN1);
            PoleArm_Adjustment_2006.encoder_speed = DJI_GetSpeed(&RxHeader, RxMsg_CAN1);
            DJI_Abs_Encoder_Process(&PoleArm_Adjustment_2006.motor_encoder, PoleArm_Adjustment_2006.EncoderNum);
            PoleArm_Adjustment_2006.angle = PoleArm_Adjustment_2006.motor_encoder.siSumValue / (float)8192 * 360.f / (float)M2006_uiGearRatio;
            PoleArm_Adjustment_2006.anglev = PoleArm_Adjustment_2006.encoder_speed / (float)M2006_uiGearRatio;
            System_Monitor.Cnt_PoleArm_Adjustment_2006_Receive++;
            break;
		
		

        default:
            break;
        }
    }
    if (hcan->Instance == CAN2)
    {
        CAN_RxHeaderTypeDef RxHeader;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxMsg_CAN2);
        System_Monitor.Cnt_Can_Receive[1]++;
        switch (RxHeader.StdId)
        {
        case (0x01 << 5) | 0x10 | 0x01: // 取块机械臂第一个关节J60失能指令包，ID为1
            if (RxMsg_CAN2[0] == 0)
                J60_Enable_State[0] = DISABLE_STATE;
            break;
        case (0x02 << 5) | 0x10 | 0x01: // 取块机械臂第一个关节J60使能指令包，ID为1
            if (RxMsg_CAN2[0] == 0)
                J60_Enable_State[0] = ENABLE_STATE;
            break;
        case (0x04 << 5) | 0x10 | 0x01: // 取块机械臂第一个关节J60数据包，ID为1
            YSC_UintsToFloats(RxMsg_CAN2, &BlockArm_Joint1_J60Data);
            System_Monitor.Cnt_BlockArm_Joint1_J60_Receive++;
            break;

        case (0x01 << 5) | 0x10 | 0x02: // 取块机械臂第二个关节J60失能指令包，ID为2
            if (RxMsg_CAN2[0] == 0)
                J60_Enable_State[1] = DISABLE_STATE;
            break;
        case (0x02 << 5) | 0x10 | 0x02: // 取块机械臂第二个关节J60使能指令包，ID为2
            if (RxMsg_CAN2[0] == 0)
                J60_Enable_State[1] = ENABLE_STATE;
            break;
        case (0x04 << 5) | 0x10 | 0x02: // 取块机械臂第二个关节J60数据包，ID为2
            YSC_UintsToFloats(RxMsg_CAN2, &BlockArm_Joint2_J60Data);
            System_Monitor.Cnt_BlockArm_Joint2_J60_Receive++;
            break;

        case (0x01 << 5) | 0x10 | 0x03: // 取块机械臂第二个关节J60失能指令包，ID为3
            if (RxMsg_CAN2[0] == 0)
                J60_Enable_State[2] = DISABLE_STATE;
            break;
        case (0x02 << 5) | 0x10 | 0x03: // 取块机械臂第二个关节J60使能指令包，ID为3
            if (RxMsg_CAN2[0] == 0)
                J60_Enable_State[2] = ENABLE_STATE;
            break;
        case (0x04 << 5) | 0x10 | 0x03: // 取块机械臂第二个关节J60数据包，ID为3
            YSC_UintsToFloats(RxMsg_CAN2, &BlockArm_Gimbal_J60Data);
            System_Monitor.Cnt_BlockArm_Gimbal_J60_Receive++;
            break;


        default:
            break;
        }
    }
}
