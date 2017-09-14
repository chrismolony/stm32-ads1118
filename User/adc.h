#ifndef _ADC_H
#define _ADC_H
#include "stm32f10x.h"


#define		ADC_GPIO								GPIOC
#define		ADC_GPIO_CLK						RCC_APB2Periph_GPIOC
#define		ADC_Channel_10_GPIO			GPIO_Pin_0
#define		ADC_Channel_11_GPIO			GPIO_Pin_1
#define		ADC_Channel_12_GPIO			GPIO_Pin_2
#define		ADC_CLK									RCC_APB2Periph_ADC1
#define		ADC_DMA_Channel					DMA1_Channel1	
#define		ADC_DR_ADDRESS					(uint32_t)&ADC1->DR
#define		ADC_USE_MODEL						ADC1



void ADC_GPIO_Config(void);
void ADC_RCC_Config(void);
void ADC_DMA_Config(void);


void ADC_Config(void);
#endif
