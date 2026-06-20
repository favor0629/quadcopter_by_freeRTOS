#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"
//#include "key.h"
//#include "remote.h"
#include "sys.h"


#define aux1 GPIO_Pin_15 //GPIOA
#define aux2 GPIO_Pin_11 //GPIOB
#define aux3 GPIO_Pin_14//GPIOC
#define aux4 GPIO_Pin_15//GPIOC
#define front GPIO_Pin_0 //GPIOB
#define back GPIO_Pin_1 //GPIOB
#define left GPIO_Pin_6 //GPIOA
#define right GPIO_Pin_7 //GPIOA	

void key_init(void);
	
uint8_t key(void);




#endif
