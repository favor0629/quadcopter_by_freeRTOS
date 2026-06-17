#ifndef __BULUE_SERIAL_H
#define __BULUE_SERIAL_H

#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include <stdint.h>

extern char BlueSerial_RxPacket[];	// 接收数据包缓冲区
extern volatile uint8_t BlueSerial_RxFlag;	// 接收完成标志

void BlueSerial_Init(void);
void BlueSerial_SendByte(uint8_t Byte);
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length);
void BlueSerial_SendString(char *String);
void BlueSerial_SendNumber(uint32_t Number, uint8_t Length);
void BlueSerial_Printf(char *format, ...);
uint8_t BlueSerial_GetRxFlag(void);
char *BlueSerial_GetRxPacket(void);


#endif /* __BULUE_SERIAL_H */