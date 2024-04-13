/*
 * init.h
 *
 *  Created on: Mar 31, 2024
 *      Author: aordu
 */

#ifndef INC_INIT_H_
#define INC_INIT_H_

#include "stm32f4xx.h"


void ADC_Inj_Init(void);
void ADC_Reg_Init(void);
void ADC_2Ch_Reg_Init(void);
void ADC_2Ch_Inj_Init(void);
uint16_t ADC_Inj_Read(void);
uint16_t ADC_Reg_Read(void);
uint16_t ADC_2Ch_Reg_Read(uint8_t channel);
uint16_t ADC_2Ch_Inj_Read(uint8_t channel);

#endif /* INC_INIT_H_ */
