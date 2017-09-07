

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

#include "ads1118.h"
#include "delay.h"
#include "remote_data_x.h"
#include "stdio.h"
																			

/**********************************************************************************************************************/
int main(void)
{
     
		float Temp; 
		uint8_t i;
		uint16_t	getvalue;
		volatile uint32_t config=0;

		//SystemInit();
		delay_init();                          

		remote_data_x_usart_Init();
		printf("\n\r--------------ADS1118测试---------------------\n ");
		//remote_data_x_usart_dma_ctl();//每使用一次printf就需要调用一次该函数
		delay_ms(1000);//1s
		SPI_config();
		while (1)
		{
				//remote_data_x_usart_read_enable();

				remote_data_x_usart_write_enable();
				Temp = ads1118_get_temperature();
				printf("\n\rGet from ADS1118 temperature sensor: %0.2f (C)\n",Temp);
				//remote_data_x_usart_dma_ctl();//每使用一次printf就需要调用一次该函数

				delay_ms(100);//
				for(i=4;i<8;i++){
						getvalue = ads1118_convert(i);
						printf("\n\rGet from ADS1118 channel AIN%d:0x%02x\n",i-4,getvalue);
						//remote_data_x_usart_dma_ctl();//每使用一次printf就需要调用一次该函数
						delay_ms(100);
				}
				delay_ms(1000);
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
