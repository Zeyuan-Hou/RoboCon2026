#include "servo.h"
/**
 * @brief:舵机控制函数(状态机)
 * @note:根据实际情况，分割状态，写死舵机编码器目标位置
 * @note
 */
uint16_t ser1=750;
uint16_t ser2=0;
uint16_t ser3=400;
uint16_t ser4=350;
 void servo_state_control(void)
{
    switch(Servo_State)
    {
      case LOAD_BALL:
        Servo_Set.Ref_Position[0] = ser1;
        Servo_Set.Ref_Position[1] = ser2;

        Write_Pos_SCS(1, &Servo_Set, uart4_tx_buffer);
        vTaskDelay(pdMS_TO_TICKS(3)); 
        Write_Pos_SCS(1, &Servo_Set, uart4_tx_buffer);
        vTaskDelay(pdMS_TO_TICKS(3)); 
        Write_Pos_SCS(2, &Servo_Set, uart4_tx_buffer);
        vTaskDelay(pdMS_TO_TICKS(3));        
        
          if(servo_judge())
          {
            Servo_Set.stable_state++;
          }
          else  Servo_Set.stable_state =0;
            
        break;
      
      case DRIB_BALL:
				Servo_Set.Ref_Position[0] = ser3;
        Servo_Set.Ref_Position[1] = ser4;
			
        Write_Pos_SCS(1, &Servo_Set, uart4_tx_buffer);
        vTaskDelay(pdMS_TO_TICKS(3)); 
        Write_Pos_SCS(1, &Servo_Set, uart4_tx_buffer);
        vTaskDelay(pdMS_TO_TICKS(3)); 
        Write_Pos_SCS(2, &Servo_Set, uart4_tx_buffer);
        vTaskDelay(pdMS_TO_TICKS(3));  
        
         if(servo_judge())
          {
            Servo_Set.stable_state++;
          }
          else  Servo_Set.stable_state =0;

        break;

     default:
        break;        
     }
}
/**
 * @brief  请求舵机数据反馈
 * @note   SCS215舵机不支持同步读指令，(解决：用FOR循环，一个一个读)
         且当延时相同时，会相互打断，读取2个反馈会丢失1个。（解决：增加读取延时vTaskDelay(pdMS_TO_TICKS(2));）
 */
void Get_Serve_Pos(void)
{
        /**
         * Read_SCS - 读取舵机数据
         * @param i+1         : 舵机 ID (1~5)
         * @param 0x02        : 指令码 (0x02 表示读取数据)
         * @param 0x38        : 读取地址 (0x38 代表当前角度寄存器)
         * @param 0x02        : 读取数据长度 (2 字节)
         * @param uart4_tx_buffer : 读取数据缓存
         */
         
        /*void Read_SCS(uint8_t ID, uint8_t Cmd, uint8_t Address,
             uint8_t ReadSize,volatile int8_t *buf)*/
        Read_SCS(1, 0x02, 0x38, 0x02, uart4_tx_buffer);
        vTaskDelay(pdMS_TO_TICKS(2));

        Read_SCS(1, 0x02, 0x38, 0x02, uart4_tx_buffer);
        vTaskDelay(pdMS_TO_TICKS(2));          
             
        Read_SCS(2, 0x02, 0x38, 0x02, uart4_tx_buffer);
        vTaskDelay(pdMS_TO_TICKS(2));
        
        //Write_Pos_SCS延时是3ms Read_SCS的延时是2ms


       // 计算舵机当前角度与目标角度的误差，并判断是否稳定
        for (uint8_t i = 0; i < 2; i++)
        {
            // 计算误差: 当前角度 - 目标角度
            Servo_Set.Servo_Stab.Error[i] =
            Servo_Set.Position[i] - Servo_Set.Ref_Position[i];

            // 判断误差是否在容许范围内，决定舵机是否稳定
            if (abs(Servo_Set.Servo_Stab.Error[i])
                 <= Servo_Set.Servo_Stab.StabDomain[i])
            {
                Servo_Set.Servo_Stab.Stab[i] = 1;  // 舵机已稳定
            }
            else
            {
                Servo_Set.Servo_Stab.Stab[i] = 0;  // 舵机未稳定
            }
        }
}

uint16_t Servo_cnt = 0;
/**
 * @brief 判断机械臂舵机族的稳定性
 * @details 该函数检查所有舵机的稳定状态，如果所有舵机都稳定，并且持续稳定时间超过设定值，则返回1
 * @return uint8_t 机械臂整体稳定标志
 *         - 1：机械臂所有舵机稳定，且稳定时间满足设定值
 *         - 0：机械臂未稳定，或者稳定时间未达标
 */
uint8_t servo_judge(void)
{
    uint8_t all_servo_normal = 0; // 机械臂稳定标志位，初始为0（未稳定）
    Servo_cnt++;                // 每次调用函数，增加稳定计时

    // 检查所有舵机是否都处于稳定状态（假设共有5个舵机）
    if (Servo_Set.Servo_Stab.Stab[0] &&
        Servo_Set.Servo_Stab.Stab[1])
    {
        // 如果所有舵机都稳定，检查稳定时间是否达到设定阈值
        if (Servo_cnt >= Servo_Set.Servo_Stab.Stab_MaxTime)
        {
            Servo_cnt = 0;      // 复位计时器，准备下一次稳定判断
            all_servo_normal = 1; // 机械臂整体稳定，设置标志位
        }
    }

    return all_servo_normal; // 返回总的稳定状态
}




/*舵机角度反馈*/
void servo_deal(void){

    switch (uart4_rx_buffer[2])   
      {
      case 0x01:  
        
        Servo_Set.Position[0]  = uart4_rx_buffer[5]<<8|uart4_rx_buffer[6];
        
        break;
        
      case 0x02:
        
        Servo_Set.Position[1]  = uart4_rx_buffer[5]<<8|uart4_rx_buffer[6];
        
        break;

      default:
      
        break;      
      }
}

/**
 * @brief: 读取舵机数据
 * @param: ID 舵机ID, Cmd 读取指令, 
           Address 读取地址, （数据读出段的首地址）
           ReadSize 读取数据长度（读取数据的长度）,
           buf 读取数据缓存
 * @note: 参考feetech SCS协议
 * @author: HYH
 */
void Read_SCS(uint8_t ID, uint8_t Cmd, uint8_t Address, 
              uint8_t ReadSize,  int8_t *buf)
{

  static SCS_Buf_TypeDef TxBufMsg;
  
  /*先存到TxBufMsg*/
  TxBufMsg.Head[0] = 0xFF; //帧头
  TxBufMsg.Head[1] = 0xFF;
  TxBufMsg.ID = ID;        //ID
     
  TxBufMsg.Length = 0x04;  //有效数据长度
    
  TxBufMsg.Cmd = Cmd;      //指令
  TxBufMsg.Param[0] =  Address; //参数1
  TxBufMsg.Param[1] = ReadSize; //参数2
  
  TxBufMsg.Sum = TxBufMsg.ID+
                 TxBufMsg.Length+
                 TxBufMsg.Cmd+
                 Address+  //param仅包含两个参数
                 ReadSize;      //计算出来应该是0XBE

  /*再存到buf中*/
  buf[0] = TxBufMsg.Head[0];
  buf[1] = TxBufMsg.Head[1];
  buf[2] = TxBufMsg.ID;
  
  buf[3] = TxBufMsg.Length;
  
  buf[4] = TxBufMsg.Cmd;
  buf[5] = TxBufMsg.Param[0];
  buf[6] = TxBufMsg.Param[1];
  buf[7] = ~TxBufMsg.Sum;

  __HAL_DMA_CLEAR_FLAG(&hdma_uart4_tx,DMA_FLAG_TCIF2_6);//清除DMA传输完成标志
  __HAL_DMA_DISABLE(&hdma_uart4_tx);
  
  HAL_UART_Transmit_DMA(&huart4, (uint8_t *)buf, 8);
  
  __HAL_DMA_ENABLE(&hdma_uart4_tx);

}


/**
 * @brief: 写入舵机数据
 * @param: ID 舵机ID, Servo 舵机结构体, buf 写入数据缓存
 * @note: 参考feetech SCS协议，写入舵机目标位置、时间、速度等数据
 * @author: HYH
 */
void Write_Pos_SCS(uint8_t ID, Servo* Servo,  int8_t* buf)
{
  //限幅
  Servo_Set.Ref_Position[0]=ClipShort(Servo_Set.Ref_Position[0],0,1000);
  Servo_Set.Ref_Position[1]=ClipShort(Servo_Set.Ref_Position[1],0,1000);
  
  // 声明一个静态的SCS协议数据包结构体，用于存储将要发送的数据
  static SCS_Buf_TypeDef TxBufMsg;

  // 填充数据包头部
  TxBufMsg.Head[0] = 0xFF;   // 数据包头部第一个字节，Feetech协议固定为0xFF
  TxBufMsg.Head[1] = 0xFF;   // 数据包头部第二个字节，Feetech协议固定为0xFF
  TxBufMsg.ID = ID;          // 设置舵机的ID

  TxBufMsg.Length = 0x09;    // 数据包的长度（不包括头部和校验和）

  TxBufMsg.Cmd = 0x03;       // 命令字节，0x03表示设置目标位置等参数

  // 填充目标位置数据
  TxBufMsg.Param[0] = 0x2A;  // 目标位置的起始地址（固定为0x2A）
  TxBufMsg.Param[1] = Servo->Ref_Position[ID-1] >> 8 & 0xFF;  // 目标位置的高字节
  TxBufMsg.Param[2] = Servo->Ref_Position[ID-1] & 0xFF;       // 目标位置的低字节

  // 填充目标时间数据
  TxBufMsg.Param[3] = Servo->Time[ID-1] >> 8 & 0xFF;         // 目标时间的高字节
  TxBufMsg.Param[4] = Servo->Time[ID-1] & 0xFF;              // 目标时间的低字节

  // 填充目标速度数据
  TxBufMsg.Param[5] = Servo->Ref_Speed[ID-1] >> 8 & 0xFF;    // 目标速度的高字节
  TxBufMsg.Param[6] = Servo->Ref_Speed[ID-1] & 0xFF;         // 目标速度的低字节

  // 计算校验和，所有字段相加求和
  TxBufMsg.Sum = TxBufMsg.ID + TxBufMsg.Length + TxBufMsg.Cmd +
                 TxBufMsg.Param[0] + TxBufMsg.Param[1] +
                 TxBufMsg.Param[2] + TxBufMsg.Param[3] +
                 TxBufMsg.Param[4] + TxBufMsg.Param[5] +
                 TxBufMsg.Param[6];

  // 将数据包内容写入到缓冲区buf
  buf[0] = TxBufMsg.Head[0];  // 头部第一个字节
  buf[1] = TxBufMsg.Head[1];  // 头部第二个字节
  buf[2] = TxBufMsg.ID;      // 舵机ID
  buf[3] = TxBufMsg.Length;  // 数据包长度
  buf[4] = TxBufMsg.Cmd;     // 命令字节
  buf[5] = TxBufMsg.Param[0]; // 目标位置起始地址
  buf[6] = TxBufMsg.Param[1]; // 目标位置的高字节
  buf[7] = TxBufMsg.Param[2]; // 目标位置的低字节
  buf[8] = TxBufMsg.Param[3]; // 目标时间的高字节
  buf[9] = TxBufMsg.Param[4]; // 目标时间的低字节
  buf[10] = TxBufMsg.Param[5]; // 目标速度的高字节
  buf[11] = TxBufMsg.Param[6]; // 目标速度的低字节
  buf[12] = ~TxBufMsg.Sum;   // 校验和（取反）

  // 清除DMA传输完成标志
  __HAL_DMA_CLEAR_FLAG(&hdma_uart4_tx, DMA_FLAG_TCIF2_6);

  // 禁用DMA，确保数据准备完成后再进行发送
  __HAL_DMA_DISABLE(&hdma_uart4_tx);

  // 启动DMA传输，使用USART2发送缓冲区中的数据
  HAL_UART_Transmit_DMA(&huart4, (uint8_t *)buf, 13);

  // 重新启用DMA
  __HAL_DMA_ENABLE(&hdma_uart4_tx);

}

