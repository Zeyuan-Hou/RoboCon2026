#ifndef _POSITIONSWITCH_H
#define _POSITIONSWITCH_H

//#define ScanKey(n)  GET_KEY##n   行程开关（光电门）
/*读到的数为1时，行程开关接GND，表示按下*/
#define GET_KEY1    (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)  // 如果PC10为高电平返回1，低电平返回0
#define GET_KEY2    (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == GPIO_PIN_SET)
#define GET_KEY3    (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_SET)
#define GET_KEY4    (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET)

#define GET_KEY5    (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_SET)
#define GET_KEY6    (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) == GPIO_PIN_SET)
#define GET_KEY7    (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11) == GPIO_PIN_SET)
#define GET_KEY8    (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10) == GPIO_PIN_SET)


void Scan_key(void);
uint8_t ScanKey(uint8_t KeyNum);

#endif
