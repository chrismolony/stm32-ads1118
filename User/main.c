
/*********************************************
STM32F103 ADC测试与ADS1118测试
@StevenShi
*********************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

#include "adc.h"
#include "ads1118.h"
#include "delay.h"
#include "remote_data_x.h"
#include "stdio.h"
#include "string.h"
												
extern uint16_t get_adc[3];
/**********************************************************************************************************************/
int main(void)
{
	double TempC,TempH_0_1[SAMPLECOUNTER],TempH_2_3[SAMPLECOUNTER];
	uint8_t i,counter_0_1 = SAMPLECOUNTER,counter_2_3 = SAMPLECOUNTER;
	volatile uint32_t config=0;
	double read_mv_0_1,read_mv_2_3;
	double TempH_0_1_filter,TempH_2_3_filter;
	
	
	delay_init(); 
	
	remote_data_x_usart_Init();
	
	SPI_config();
	
	ADC_Config();
	
	delay_ms(1000);
	
	while (1)
	{
		printf("\n\r--------------ADS1118测试---------------------\n ");
		//因为ADS1118内部温度传感器测试的是芯片内部温度，内部有功耗与发热，所以一般会比室温要高4-5摄氏度
		//实际测试滤波并不能去除热电偶温度不稳定现象
		remote_data_x_usart_write_enable();
		TempC = ads1118_get_temperature();
		printf("\n\rGet on chip temperature sensor: %0.2f (C)\n",TempC);
		counter_0_1 = SAMPLECOUNTER;
		delay_ms(200);//
		/**获取差分输入 AIN0 AIN1 热电偶值**/
		memset(TempH_0_1,0,SAMPLECOUNTER);
		for(i=0;i<SAMPLECOUNTER;i++){
			read_mv_0_1 = ads1118_get_differential_0_1_mv((uint8_t)DEFAULT_PGA);
			if(ads1118_get_temp_thermo_type_k(read_mv_0_1,TempC,&TempH_0_1[i])!=0)
				counter_0_1--;
		}
		DINT();
		for(i=0;i<counter_0_1;i++)
		TempH_0_1_filter = ads1118_median_average_filter(TempH_0_1);
		EINT();
		printf("\n\rGet hot junction temperature AIN0-1: %0.2f (C)\n",TempH_0_1_filter);
		
		delay_ms(10);
		
		/*****获取AIN2 AIN3 热电偶值***********/
		counter_2_3 = SAMPLECOUNTER;
		memset(TempH_2_3,0,SAMPLECOUNTER);
		TempH_2_3_filter = 0;
		
		for(i=0;i<SAMPLECOUNTER;i++){
			read_mv_2_3 = ads1118_get_differential_2_3_mv((uint8_t)DEFAULT_PGA);
			if(ads1118_get_temp_thermo_type_k(read_mv_2_3,TempC,&TempH_2_3[i])!=0)
				counter_2_3--;
		}
		DINT();
		for(i=0;i<counter_2_3;i++)
			TempH_2_3_filter = ads1118_median_average_filter(TempH_2_3);
		EINT();
		printf("\n\rGet hot junction temperature AIN2-3: %0.2f (C)\n",TempH_2_3_filter);
		//打印AD转换的值
		printf("\n\r----------------------------------------------\n ");	
		printf("\r\nADC10:0x%02x\n",get_adc[0]);
		printf("\r\nADC11:0x%02x\n",get_adc[1]);
		printf("\r\nADC12:0x%02x\n",get_adc[2]);
		printf("\n\r----------------------------------------------\n ");
		delay_ms(2000);
		
		
	}
}





#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
		/* User can add his own implementation to report the file name and line number,
			 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

		/* Infinite loop */
		while (1)
		{}
}

#endif

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
