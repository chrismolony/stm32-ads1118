stm32f103 驱动 ads1118

Copyright StevenShi 


接口配置：

STM32F103       ADS1118

PB12  ------>   CS(pin2)

PB13  ------>   SCK(pin1)

PB14  ------>   DOUT(pin9)

PB15  ------>   DIN(pin10)

STM32F103串口

PA9- Tx PA10-Rx

说明：
1、 程序中实现了DMA模式下的printf函数功能，将要发送的数据放入队列中
CPU在sysTick中断中不断检测该队列是否不为空，不为空就搬运该数据
并将发送数据的首地址与长度给DMA，这样即可实现DMA模式下的printf

2、 ADS1118驱动参考了TI官方给出的msp430平台下的代码，并修改到STM32平台下，
实现了片内温度检测与4路模拟量输入检测，读出的值通过串口显示
如某次的测量显示：

3、 工程使用mdk 5.14

Get from ADS1118 temperature sensor: 27.84 (C)

Get from ADS1118 channel AIN0:0x00

Get from ADS1118 channel AIN1:0x00

Get from ADS1118 channel AIN2:0x00

Get from ADS1118 channel AIN3:0x00
