
#include "stm32f10x.h"
#include "delay.h"
#include "remote_data_x.h"


static __IO uint32_t usTicks;

// SysTick_Handler function will be called every 1 us
void SysTick_Handler()
{

if (usTicks != 0)
	{
		usTicks--;
	}
	//因为每次使用printf都需要调用一次该函数，索性将该函数放到sysTick中断中去
	//每1ms都检测printf的队列是否不为空，不为空就进行一次DMA传输
	//因此避免了每次调用printf都调用一次该函数
	remote_data_x_usart_dma_ctl();
}
//配置sysTick中断间隔为1ms
void delay_init(void)
{
	// Update SystemCoreClock value
	SystemCoreClockUpdate();
	// Configure the SysTick timer to overflow every 1 us
	//SysTick_Config(SystemCoreClock / 1000000);
	// Configure the SysTick timer to overflow every 1 ms
	SysTick_Config(SystemCoreClock / 1000);
}
/*
void delay_us(u32 us)
{
  // Reload us value
    usTicks = us;
    // Wait until usTick reach zero
    while (usTicks);
}


void delay_ms(u16 ms)
{
	// Wait until ms reach zero
	while (ms--)
	{
	// Delay 1ms
	delay_us(1000);
	}
}
*/
void delay_ms(u16 ms)
{
	// Reload us value
	usTicks = ms;
	// Wait until usTick reach zero
	while (usTicks);
}
