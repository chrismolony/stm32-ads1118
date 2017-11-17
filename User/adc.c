/*************************
@StevenShi
ADC ¿¿ ¿¿DMA¿¿¿¿
*************************/
#include "adc.h"


uint16_t get_adc[3];
void ADC_GPIO_Config()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure PC.01 (ADC Channel10) as analog input -------------------------*/
	/* Configure PC.02 (ADC Channel11) as analog input -------------------------*/
	/* Configure PC.03 (ADC Channel12) as analog input -------------------------*/
	GPIO_InitStructure.GPIO_Pin = ADC_Channel_10_GPIO | ADC_Channel_11_GPIO | ADC_Channel_12_GPIO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(ADC_GPIO, &GPIO_InitStructure);
}
void ADC_RCC_Config()
{
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);//72/8=9Mhz
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(ADC_CLK | ADC_GPIO_CLK, ENABLE);
}
void ADC_DMA_Config()
{
	DMA_InitTypeDef DMA_InitStructure;
	/* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(ADC_DMA_Channel);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC_DR_ADDRESS;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&get_adc;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 3;//¿¿¿¿ 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//¿¿¿¿16bits
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//¿¿16bits
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(ADC_DMA_Channel, &DMA_InitStructure);
  
  /* Enable DMA1 channel1 */
  DMA_Cmd(ADC_DMA_Channel, ENABLE);
}
void ADC_Config()
{
	ADC_InitTypeDef ADC_InitStructure;  
	/* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//¿¿¿
  ADC_InitStructure.ADC_NbrOfChannel = 3;
  ADC_Init(ADC_USE_MODEL, &ADC_InitStructure);
	
  /* ADC1 regular channel14 configuration */ 
  ADC_RegularChannelConfig(ADC_USE_MODEL, ADC_Channel_10, 1, ADC_SampleTime_28Cycles5);
	ADC_RegularChannelConfig(ADC_USE_MODEL, ADC_Channel_11, 2, ADC_SampleTime_28Cycles5);
	ADC_RegularChannelConfig(ADC_USE_MODEL, ADC_Channel_12, 3, ADC_SampleTime_28Cycles5);
	
	ADC_GPIO_Config();
	ADC_RCC_Config();
	ADC_DMA_Config();
  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC_USE_MODEL, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC_USE_MODEL, ENABLE);

  /* Enable ADC1 reset calibration register */   
  ADC_ResetCalibration(ADC_USE_MODEL);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC_USE_MODEL));

  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC_USE_MODEL);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC_USE_MODEL));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC_USE_MODEL, ENABLE);
}
