#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

typedef struct
{
	uint16_t cob_id;	// msg id
	uint8_t rtr;			//帧模式
	uint8_t len;			//数据长度
	uint8_t data[8];	//数据域
}bsp_can_msg_t;

extern FDCAN_TxHeaderTypeDef TxHeader;
extern FDCAN_RxHeaderTypeDef RxHeader1;
extern FDCAN_RxHeaderTypeDef RxHeader2;
extern uint8_t FDCAN_TxData[8];
extern uint8_t FDCAN_RxData[8];

void CAN_INIT(void);
void bsp_fdcan1_init(void);
void bsp_fdcan2_init(void);
void CAN_SendStdData(FDCAN_HandleTypeDef* hfdcan, uint16_t ID, uint8_t *pData, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

