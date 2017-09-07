
#include "ads1118.h"
/*******************************************
  ADS1118驱动
	@StevenShi
*******************************************/

volatile uint8_t getdata1;
volatile uint8_t getdata2;

volatile uint16_t getdata;

ADS_InitTypeDef adsConfigReg;//ADS1118配置寄存器


void ads1118_config(void)
{
    
    adsConfigReg.stru.CNV_RDY_FL	=		DATA_READY ;           //low ,writing is no effort
    adsConfigReg.stru.NOP     		=  	DATA_VALID;
    adsConfigReg.stru.PULLUP  		=  	PULL_UP_DIS;
    adsConfigReg.stru.TS_MODE 		=  	ADC_MODE;
    adsConfigReg.stru.DR      		=  	DR_128_SPS;
    adsConfigReg.stru.MODE    		=  	SIGNLE_SHOT;
    adsConfigReg.stru.PGA     		=  	PGA_2048;
    adsConfigReg.stru.MUX     		=  	AINPN_0_1;
    adsConfigReg.stru.OS      		=  	SINGLE_CONVER_START;   //high
}
void SPI_config(void)
{
		SPI_InitTypeDef SPI_InitStructure;
		SPI_RCC_Configuration();
		SPI_GPIO_Configuration();
		SPI_NVIC_Configuration();


		/* Disable SPI_MASTER */
		SPI_Cmd(SPI_MASTER, DISABLE);
		/* SPI_MASTER configuration ------------------------------------------------*/
		SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
		SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
		SPI_InitStructure.SPI_CRCPolynomial = 7;
		SPI_Init(SPI_MASTER, &SPI_InitStructure);



		/* Enable SPI_MASTER TXE interrupt */
		//SPI_I2S_ITConfig(SPI_MASTER, SPI_I2S_IT_TXE, ENABLE);
		/* Enable SPI_SLAVE RXNE interrupt */
		// SPI_I2S_ITConfig(SPI_MASTER, SPI_I2S_IT_RXNE, ENABLE);


		/* Enable SPI_MASTER */
		SPI_Cmd(SPI_MASTER, ENABLE);
	
}


/**********************************************************************
@StevenShi 
获取片内温度
**********************************************************************/
float ads1118_get_temperature(void)
{
    uint16_t adc=0;
    float value=0;
    
   
    adsConfigReg.stru.NOP     =  DATA_VALID;
    adsConfigReg.stru.TS_MODE =  TEMPERATURE_MODE;
    adsConfigReg.stru.DR      =  DR_8_SPS;
    adsConfigReg.stru.MODE    =  SIGNLE_SHOT;
    adsConfigReg.stru.OS      =  SINGLE_CONVER_START;   
    
    
    ADS1118_ENABLE;
   
    adc = SPI_read_write_Reg(adsConfigReg.word);
    /*
    adc = SPI_read_write_Reg(0x853b);//这两种配置都可以读出片内温度
    adc = SPI_read_write_Reg(0x859b);
	  */
		
    //conver to temperture
    if(adc&0x8000)//温度负值处理
    {
        //-xx.xxx c
        adc>>=2;
        value=(0x3fff-adc+1)*(-0.03125);
    }
    else//温度正值处理
    {
        //+xx.xxx c
        adc>>=2;
        value=adc*0.03125;     
    }
		ADS1118_DISABLE;
    return value;
    
}


void ads1118_set_config_reg(ADS_InitTypeDef* ConfigReg)
{
    adsConfigReg.word=ConfigReg->word;
}


/***************************************************
单次转换函数，输入参数为通道
	AINPN_0_1 	= 	0x0,
	AINPN_0_3 	=   0x1,
	AINPN_1_3 	=   0x2,
	AINPN_2_3 	=   0x3,
	AINPN_0_GND	=  	0x4,
	AINPN_1_GND	=  	0x5,
	AINPN_2_GND	=  	0x6,
	AINPN_3_GND	=  	0x7
***************************************************/

uint16_t ads1118_convert(uint8_t channel)
{
		uint16_t adc=0;
		ADS_InitTypeDef ConfigReg,staus;

		ConfigReg.word = adsConfigReg.word;           //low ,writing is no effort


		ConfigReg.stru.NOP     =  DATA_VALID;
		ConfigReg.stru.TS_MODE =  ADC_MODE;
		ConfigReg.stru.MODE    =  SIGNLE_SHOT;
		ConfigReg.stru.MUX     =  channel;
		ConfigReg.stru.OS      =  SINGLE_CONVER_START;   //high

		adc = SPI_read_write_Reg(ConfigReg.word);
		//用于寄存器回读 可以不进行此操作 手册中提到在写入寄存器配置后紧接着写0即可回读数据
		staus.word  = (SPI_read_write_Byte(0x00)&0xff)<<8;
		staus.word  |= SPI_read_write_Byte(0x00)&0xff;


		//printf(" status  %04x   ",staus.word);
		return   adc;
}




/***********************************
SPI 写
***********************************/


uint8_t SPI_send_Byte(uint8_t byte)
{
 
   //等待发送缓存空
    while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
    {}
		//发送数据
    SPI_I2S_SendData(SPI2, byte);
    //等待接收数据 循环检测接收数据缓存区
    while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
    {}
		//返回读出的数据
    return SPI_I2S_ReceiveData(SPI2);
 
}
/**************************
16bit 模式读取 DIN接收寄存器配置，DOUT输出转换结果
**************************/
uint16_t SPI_read_write_Reg(uint16_t CofigReg)
{

    
    getdata1=SPI_send_Byte((uint8_t)(CofigReg>>8));
    getdata2=SPI_send_Byte((uint8_t)CofigReg);
 
    getdata= (uint16_t)getdata2|((uint16_t)getdata1<<8);
	
    return getdata;
}
/************************************
SPI 读
************************************/

uint16_t SPI_read_write_Byte(uint16_t TxData)
{    
  uint8_t Temp_Data;
  uint16_t Re_Data;

  Temp_Data = (TxData >> 8);
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}
  
  SPI_I2S_SendData(SPI2, Temp_Data); 
  
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){} 
  Re_Data = SPI_I2S_ReceiveData(SPI2);
  
  Temp_Data = ( TxData & 0xff );
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET){}
  SPI_I2S_SendData(SPI2, Temp_Data); 
  
  while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET){} 
  Re_Data = (Re_Data << 8) | SPI_I2S_ReceiveData(SPI2);
    
  return Re_Data; 
	}

void SPI_RCC_Configuration(void)
{
 
  /* Enable GPIO clock for SPI_MASTER */
  RCC_APB2PeriphClockCmd(SPI_MASTER_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

  /* Enable SPI_MASTER Periph clock */
  RCC_APB1PeriphClockCmd(SPI_MASTER_CLK, ENABLE);                           

}


void SPI_GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure SPI_MASTER pins-*/

	// Pin PB13 (SCLK) must be configured as as 50MHz push pull
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SPI_MASTER_PIN_SCK;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_MASTER_GPIO, &GPIO_InitStructure);

	// Pin PB14 (MISO) must be configured as as input pull-up
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SPI_MASTER_PIN_MISO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_MASTER_GPIO, &GPIO_InitStructure);

	// Pin PB15 (MOSI) must be configured as as 50MHz push pull
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SPI_MASTER_PIN_MOSI;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SPI_MASTER_GPIO, &GPIO_InitStructure);
	
	//SPI1 NSS 
	//GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SPI_MASTER_PIN_NSS;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SPI_MASTER_GPIO, &GPIO_InitStructure);

	GPIO_SetBits(SPI_MASTER_GPIO, SPI_MASTER_PIN_NSS); 

}


void SPI_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
	

  /* 1 bit for pre-emption priority, 3 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  /* Configure and enable SPI_MASTER interrupt -------------------------------*/
  NVIC_InitStructure.NVIC_IRQChannel = SPI_MASTER_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}



