#ifndef __BEEP_H
#define __BEEP_H

#include "stm32f10x.h"

void BEEP_Init(void);  

void BEEP_Flash(uint16_t time);

void BEEP_Disable(void);

#endif
