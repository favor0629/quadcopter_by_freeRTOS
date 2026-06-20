#ifndef _REMOTE_H
#define _REMOTE_H

#include "stm32f10x.h" 

#include "ADC_DMA_Config.h"
#include "sys.h"
#include "nrf24l01.h"
#include <math.h>
#include "eeprom.h"
#include "delay.h"
#include "key.h"


typedef struct 
{
	uint16_t roll;
	uint16_t pitch;
	uint16_t thr;
	uint16_t yaw;	
	uint16_t AUX1;
	uint16_t AUX2;
	uint16_t AUX3;
	uint16_t AUX4;
}Remote_t;



extern volatile Remote_t Remote;

extern void  RC_INIT(void);

extern void RC_Analy(void);

void remote_key_scan();
#endif

