#include "can_bsp.h"

void CAN_INIT(void)
{
    bsp_fdcan1_init(); // 过滤器初始化
    bsp_fdcan2_init();
    bsp_fdcan3_init();
}

/******************************************************************
 * @brief 初始化配置FDCAN1 各项功能函数，开启FDCAN 收发
 *
 * @brief 过滤器模式为 掩码模式 0x0000 - 0xffff 绑定过滤参数到fifo0
 *
 * @param
 *
 * @return
 *******************************************************************/
void bsp_fdcan1_init(void)
{
    /* 配置RX滤波器 */
    FDCAN_FilterTypeDef FDCAN_RXFilter = {0};
    FDCAN_RXFilter.IdType = FDCAN_STANDARD_ID;             // 标准ID
    FDCAN_RXFilter.FilterIndex = 0;                        // 过滤器索引
    FDCAN_RXFilter.FilterType = FDCAN_FILTER_MASK;         // 过滤器类型
    FDCAN_RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器0关联到FIFO0
    FDCAN_RXFilter.FilterID1 = 0x0000;                     // 32位ID1
    FDCAN_RXFilter.FilterID2 = 0x0000;                     // 32位ID2
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_RXFilter) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    /* 开启CAN 收发中断 */
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
}
/******************************************************************
 * @brief 初始化配置FDCAN1 各项功能函数，开启FDCAN 收发
 *
 * @brief 过滤器模式为 掩码模式 0x0000 - 0xffff 绑定过滤参数到fifo1
 *
 * @param
 *
 * @return
 *******************************************************************/
void bsp_fdcan2_init(void)
{
    /* 配置RX滤波器 */
    FDCAN_FilterTypeDef FDCAN_RXFilter = {0};
    FDCAN_RXFilter.IdType = FDCAN_STANDARD_ID;             // 标准ID
    FDCAN_RXFilter.FilterIndex = 0;                        // 过滤器索引
    FDCAN_RXFilter.FilterType = FDCAN_FILTER_MASK;         // 过滤器类型
    FDCAN_RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器1关联到FIFO0
    FDCAN_RXFilter.FilterID1 = 0x0000;                     // 32位ID1
    FDCAN_RXFilter.FilterID2 = 0x0000;                     // 32位ID2
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN_RXFilter) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }
    /* 开启CAN 收发中断 */
    if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

void bsp_fdcan3_init(void)
{
    /* 配置RX滤波器 */
    FDCAN_FilterTypeDef FDCAN_RXFilter = {0};
    FDCAN_RXFilter.IdType = FDCAN_STANDARD_ID;             // 标准ID
    FDCAN_RXFilter.FilterIndex = 0;                        // 过滤器索引
    FDCAN_RXFilter.FilterType = FDCAN_FILTER_MASK;         // 过滤器类型
    FDCAN_RXFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤器1关联到FIFO0
    FDCAN_RXFilter.FilterID1 = 0x0000;                     // 32位ID1
    FDCAN_RXFilter.FilterID2 = 0x0000;                     // 32位ID2
    if (HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN_RXFilter) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }
    /* Start the FDCAN module */
    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK)
    {
        Error_Handler();
    }
    /* 开启CAN 收发中断 */
    if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

/******************************************************************
 * @brief FDCAN发送函数
 *
 * @brief函数使用方法备注
 *
 * @param can设备句柄
 *
 * @return
 *******************************************************************/

void CAN_SendStdData(FDCAN_HandleTypeDef *hfdcan, uint16_t ID, uint8_t *pData, uint16_t Len)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = ID;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = Len;

    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0)
        ; // 等待有发送邮箱可用
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, pData);
}

void CAN_SendCurrent(FDCAN_HandleTypeDef *hfdcan, uint32_t id, int16_t current1, int16_t current2, int16_t current3, int16_t current4)
{
    uint8_t TxCurrent[8];
    TxCurrent[0] = (current1 >> 8);
    TxCurrent[1] = current1;
    TxCurrent[2] = (current2 >> 8);
    TxCurrent[3] = current2;
    TxCurrent[4] = (current3 >> 8);
    TxCurrent[5] = current3;
    TxCurrent[6] = (current4 >> 8);
    TxCurrent[7] = current4;
    CAN_SendStdData(hfdcan, id, TxCurrent, 8);
}

void TravelSwitchDataDeal(const uint8_t travel_switch, uint8_t *key_states)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        key_states[i] = (travel_switch >> i) & 0x01;
    }
}

/******************************************************************
 * @brief FDCAN1 接收回调函数
 *
 * @brief
 *
 * @param
 *
 * @return
 *******************************************************************/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];

        // 使用传入的 hfdcan 句柄自动读取对应的硬件 FIFO
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            // 根据 Instance 指针识别是哪路 CAN
            if (hfdcan->Instance == FDCAN1)
            {
                switch (RxHeader.Identifier)
                {
                

                default:
                    break;
                }
            }
            else if (hfdcan->Instance == FDCAN2)
            {
                switch (RxHeader.Identifier)
                {
                
                default:
                    break;
                }
            }
            else if (hfdcan->Instance == FDCAN3)
            {
                switch (RxHeader.Identifier)
                {

                default:
                    break;
                }
            }
        }
    }
}
