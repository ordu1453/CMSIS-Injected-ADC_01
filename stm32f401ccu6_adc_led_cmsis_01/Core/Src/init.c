/*
 * init.c
 *
 *  Created on: Mar 31, 2024
 *      Author: aordu
 */
#include "init.h"

void ADC_Inj_Init(void)
{
	// Включаем тактирование ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// Настраиваем пины ADC
	GPIOA->MODER |= GPIO_MODER_MODER0; // PA0 в аналоговом режиме
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0; // Без подтягивания

	ADC1->CR2 &= ~ADC_CR2_ADON; // запретить АЦП

	// выбор каналов
	ADC1->JSQR = 0; // 1 инжектированный канал
	ADC1->JSQR = 0;//ADC_JSQR_JSQ4_0; // 1 преобразование - канал 1

	ADC1->CR2 |= ADC_CR2_JEXTEN; // разрешение внешнего запуска для инжектированных каналов
	ADC1->CR2 |= ADC_CR2_JEXTSEL; // Выбираем внутренний триггер для инжектированных каналов

	ADC1->CR2 &= ~ADC_CR2_CONT; // запрет непрерывного режима
	ADC1->CR1 &= ~ADC_CR1_SCAN; // запрет режима сканирования

	ADC1->CR2 |= ADC_CR2_ADON; // разрешить АЦП
}

void ADC_Reg_Init(void)
{
  // Включаем тактирование ADC
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  // Настраиваем пины ADC
  GPIOA->MODER |= GPIO_MODER_MODER0; // PA0 в аналоговом режиме
  GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0; // Без подтягивания

  // Настраиваем ADC
  ADC1->CR2 &= ~ADC_CR2_ADON; // Выключаем ADC перед настройкой
  ADC1->SQR1 = 0; // Очищаем последовательность преобразований
  ADC1->SQR3 = 0;//ADC_SQR3_SQ1; // Выбираем канал 0 для преобразования
  ADC1->CR1 &= ~ADC_CR1_SCAN; // Отключаем скан-режим
  ADC1->CR2 &= ~ADC_CR2_CONT; // Отключаем непрерывный режим
  ADC1->CR1 &= ~ADC_CR1_RES; // Устанавливаем разрешение 12 бит
  ADC1->CR2 &= ~ADC_CR2_ALIGN; // Выравнивание по правому краю
  ADC1->SMPR2 &= ~ADC_SMPR2_SMP0; // Выбираем время выборки 239,5 цикла
  ADC1->CR2 |= ADC_CR2_ADON; // Включаем ADC
  HAL_Delay(10);
}

uint16_t ADC_Inj_Read(void)
{

	ADC1->CR2 |= ADC_CR2_JSWSTART; // запуск АЦП

	while(!(ADC1->SR & ADC_SR_JEOC)); // ожидание завершения преобразования

	uint16_t res = ADC1->JDR1;

	ADC1->SR &= ~ADC_SR_JEOC; // сброс флага

	return res;
}

uint16_t ADC_Reg_Read(void)
{
  // Запускаем преобразование
  ADC1->CR2 |= ADC_CR2_SWSTART;

  // Ждем, пока преобразование не будет завершено
  while (!(ADC1->SR & ADC_SR_EOC));
  // Читаем результат преобразования
  return ADC1->DR;
}

void ADC_2Ch_Reg_Init(void)
{
	// Включаем тактирование ADC
	  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	  // Настраиваем пины ADC
	  GPIOA->MODER |= (GPIO_MODER_MODER0 | GPIO_MODER_MODER1); // PA0, PA1 в аналоговом режиме
	  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1); // Без подтягивания

	  // Настраиваем ADC
	  ADC1->CR2 &= ~ADC_CR2_ADON; // Выключаем ADC перед настройкой
	  ADC1->CR1 |= ADC_CR1_SCAN; // Включаем скан-режим
	  ADC1->CR2 &= ~ADC_CR2_CONT; // Отключаем непрерывный режим
	  ADC1->CR1 &= ~ADC_CR1_RES; // Устанавливаем разрешение 12 бит
	  ADC1->CR2 &= ~ADC_CR2_ALIGN; // Выравнивание по правому краю
	  ADC1->SMPR2 |= (ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1); // Выбираем время выборки 3 цикла для всех каналов
	  ADC1->SQR1 = 0; // Очищаем последовательность преобразований
	  ADC1->SQR1 |= ADC_SQR1_L_0;
	  ADC1->SQR3 |= ADC_SQR3_SQ2_0; // Выбираем каналы 0 и 1 для преобразования
	  ADC1->CR2 |= ADC_CR2_ADON; // Включаем ADC

}

uint16_t ADC_2Ch_Reg_Read(uint8_t channel)
{
	// Проверяем, что номер канала корректен
	  if (channel > 1) return 0;

	  // Ждем, пока не завершится текущее преобразование
	  while (ADC1->CR2 & ADC_CR2_SWSTART);

	  // Запускаем преобразование выбранного канала
	  ADC1->SQR3 = (ADC1->SQR3 & ~0x1F) | (channel << 0);
	  ADC1->CR2 |= ADC_CR2_SWSTART;

	  // Ждем, пока преобразование не будет завершено
	  while (!(ADC1->SR & ADC_SR_EOC));

	  // Читаем результат преобразования
	  return ADC1->DR;
}


void ADC_2Ch_Inj_Init(void)
{
	 // Включаем тактирование ADC
	  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	  // Настраиваем пины ADC
	  GPIOA->MODER |= GPIO_MODER_MODER0 | GPIO_MODER_MODER1; // PA0 и PA1 в аналоговом режиме
	  GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR0 | GPIO_PUPDR_PUPDR1); // Без подтягивания

	  // Настраиваем ADC
	  ADC1->CR2 &= ~ADC_CR2_ADON; // Выключаем ADC перед настройкой
	  ADC1->CR1 |= ADC_CR1_SCAN; // Включаем скан-режим
	  ADC1->CR2 &= ~ADC_CR2_CONT; // Отключаем непрерывный режим
	  ADC1->CR1 &= ~ADC_CR1_RES; // Устанавливаем разрешение 12 бит
	  ADC1->CR2 &= ~ADC_CR2_ALIGN; // Выравнивание по правому краю
	  ADC1->SMPR2 |= (ADC_SMPR2_SMP0 | ADC_SMPR2_SMP1); // Выбираем время выборки 400 циклов
	  ADC1->JSQR = 0; // Очищаем последовательность преобразований
	  ADC1->JSQR |= (1<<20);
	  ADC1->JSQR|=  ADC_JSQR_JSQ4_0;//|=  ADC_JSQR_JSQ4_0; // Выбираем каналы 0 и 1 для преобразования
	  ADC1->CR2 |= ADC_CR2_JEXTEN; // разрешение внешнего запуска для инжектированных каналов
	  ADC1->CR2 |= ADC_CR2_JEXTSEL; // Выбираем внутренний триггер для инжектированных каналов
	  ADC1->CR2 |= ADC_CR2_JSWSTART; // Запускаем инжектированное преобразование
	  ADC1->CR2 |= ADC_CR2_ADON; // Включаем ADC
}

uint16_t ADC_2Ch_Inj_Read(uint8_t channel)
{
		ADC1->CR2 |= ADC_CR2_JSWSTART;
	// Ждем, пока не будет готов результат преобразования
	  while (!(ADC1->SR & ADC_SR_JEOC));

	  // Читаем результат преобразования
	  switch (channel)
	  {
	  	  case 0:
	  		  return ADC1->JDR1;
	  		  break;
	  	  case 1:
	  		  return ADC1->JDR2;
	  		  break;
	  }
	  ADC1->SR &= ~ADC_SR_JEOC; // сброс флага
	}








