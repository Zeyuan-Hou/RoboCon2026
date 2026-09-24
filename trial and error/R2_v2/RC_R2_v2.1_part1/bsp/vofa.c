// #include "vofa.h"

// VOFA_DATA vofa_data;

// const uint8_t FRAME_TAIL[4] = {0x00, 0x00, 0x80, 0x7F}; // 固定终止符

// // 使用justfloat协议将数据发送至vofa+上位机
// void VOFA_transmit_data(float *data, uint8_t num)
// {
//    memcpy(vofa_data.ch_data, data, num * sizeof(float));

//    // 明确填充剩余通道为0
//    for (int i = num; i < CH_COUNT; i++){
//        vofa_data.ch_data[i] = 0.0f;
//    }

//    memcpy(vofa_data.tail, FRAME_TAIL, sizeof(FRAME_TAIL));

//    HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&vofa_data, sizeof(VOFA_DATA)); // 发送
// }

// void VOFA_pack_data(){ // 由CH_COUNT宏定义至多20位float，当前波特率921600理论上可发送40位float
//    vofa[0] = chassis_run.rightup.fpDes;
//    vofa[1] = chassis_run.rightup.fpU;
// }
