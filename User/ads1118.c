
#include "ads1118.h"
#include "delay.h"
#include "stdio.h"
#include "string.h"

/*******************************************
  ADS1118驱动
	实际测试芯片内部温度会高于室温4-5摄氏度
	热电偶采集不准确，加入中位值滤波并不能有效
	去除该现象
	@StevenShi
*******************************************/

volatile uint8_t getdata1;
volatile uint8_t getdata2;

volatile uint16_t getdata;

ADS_InitTypeDef adsConfigReg;//ADS1118配置寄存器


void ads1118_config(void)
{
    

	adsConfigReg.stru.NOP						=	DATA_VALID;
	adsConfigReg.stru.PULLUP				=	PULL_UP_DIS;
	adsConfigReg.stru.TS_MODE				=	ADC_MODE;
	adsConfigReg.stru.DR						=	DR_128_SPS;
	adsConfigReg.stru.MODE					=	SIGNLE_SHOT;
	adsConfigReg.stru.PGA						=	PGA_2048;
	adsConfigReg.stru.MUX						=	AINPN_0_1;
	adsConfigReg.stru.OS						=	SINGLE_CONVER_START;   //high
	adsConfigReg.stru.RESV          = CONFIG_BIT_RESV;
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
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;//SCK空闲时低
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//第二个跳变沿采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
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
获取片内温度 采用16bits模式
**********************************************************************/
float ads1118_get_temperature(void)
{
	uint16_t adc=0;
	float value=0;
	
 
	adsConfigReg.stru.NOP     =  DATA_VALID;
	adsConfigReg.stru.TS_MODE =  TEMPERATURE_MODE;
	adsConfigReg.stru.DR      =  DR_860_SPS;
	adsConfigReg.stru.MODE    =  SIGNLE_SHOT;
	adsConfigReg.stru.OS      =  SINGLE_CONVER_START; 
	adsConfigReg.stru.PULLUP	= 	PULL_UP_EN;	
	adsConfigReg.stru.RESV		=		CONFIG_BIT_RESV;
	
	ADS1118_ENABLE;
	delay_us((uint32_t)100);
	adc = SPI_read_write_Reg(adsConfigReg.word);
	//adc = SPI_read_write_Reg(0x81f3);
	delay_us((uint32_t)100);
	ADS1118_DISABLE;
	delay_ms(1);
	
	ADS1118_ENABLE;
	adsConfigReg.stru.NOP     =  DATA_INVALID;
	adsConfigReg.stru.TS_MODE =  TEMPERATURE_MODE;
	adsConfigReg.stru.DR      =  DR_860_SPS;
	adsConfigReg.stru.MODE    =  SIGNLE_SHOT;
	adsConfigReg.stru.OS      =  SINGLE_CONVER_START; 
	adsConfigReg.stru.PULLUP	= 	PULL_UP_EN;	
	adsConfigReg.stru.RESV		=		CONFIG_BIT_RESV;
	delay_us((uint32_t)100);
	//等待DOUT给出数据有效信号-从高变低
	while(GPIO_ReadInputDataBit(SPI_MASTER_GPIO,SPI_MASTER_PIN_MISO));
	adc = SPI_read_write_Reg(adsConfigReg.word);
	//adc = SPI_read_write_Reg(0x81f3);
	delay_us((uint32_t)100);
	//
//	adc = SPI_read_write_Reg(0x853b);//这两种配置都可以读出片内温度
	//adc = SPI_read_write_Reg(0x859b);
	//
	ADS1118_DISABLE;
	
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
	
	return value;
    
}


void ads1118_set_config_reg(ADS_InitTypeDef* ConfigReg)
{
	adsConfigReg.word=ConfigReg->word;
}
//获取差分输入 AIN0 AIN1
uint16_t ads1118_get_differential_0_1(uint8_t PGA)
{
	uint16_t adc=0;

	ADS_InitTypeDef ConfigReg;

	ConfigReg.stru.NOP			=	DATA_VALID;
	ConfigReg.stru.TS_MODE	=	ADC_MODE;
	ConfigReg.stru.DR      =  DR_860_SPS;
	ConfigReg.stru.PGA      = PGA;
	ConfigReg.stru.MODE			=	SIGNLE_SHOT;
	ConfigReg.stru.OS				=	SINGLE_CONVER_START;   //high
	ConfigReg.stru.MUX			=	AINPN_0_1;
	ConfigReg.stru.PULLUP		= PULL_UP_EN;
	ConfigReg.stru.RESV			= CONFIG_BIT_RESV;
	ADS1118_ENABLE;
	delay_us((uint32_t)1);
	adc = SPI_read_write_Reg(ConfigReg.word);
	
	ADS1118_DISABLE;
	delay_ms(1);
	ADS1118_ENABLE;
	ConfigReg.stru.NOP			=	DATA_INVALID;
	ConfigReg.stru.TS_MODE	=	ADC_MODE;
	ConfigReg.stru.DR      =  DR_860_SPS;
	ConfigReg.stru.PGA      = PGA;
	ConfigReg.stru.MODE			=	SIGNLE_SHOT;
	ConfigReg.stru.OS				=	SINGLE_CONVER_START;   //high
	ConfigReg.stru.MUX			=	AINPN_0_1;
	ConfigReg.stru.PULLUP		= PULL_UP_EN;
	ConfigReg.stru.RESV			= CONFIG_BIT_RESV;
	delay_us((uint32_t)1);
	
	//等待DOUT给出数据有效信号-从高变低
	while(GPIO_ReadInputDataBit(SPI_MASTER_GPIO,SPI_MASTER_PIN_MISO));
	adc = SPI_read_write_Reg(ConfigReg.word);
	delay_us(1);
	ADS1118_DISABLE;
	return   adc;
}

double ads1118_get_differential_0_1_mv(uint8_t PGA)
{
	double t_mv=0.0;
	
	uint8_t i;
	uint16_t adc_raw[SAMPLECOUNTER],adc_raw_final;
	for(i=0;i<SAMPLECOUNTER;i++)
	adc_raw[i] = ads1118_get_differential_0_1(PGA);
	//printf("\n\rGet from ADS1118 channel AIN0-1:0x%02x\n",adc_raw);
	adc_raw_final = ads1118_median_filter(adc_raw);
	
	switch(PGA){
		case PGA_6144:
			t_mv =  adc_raw_final * ADS1118_CONST_6_144V_LSB_mV;
			break;
		case PGA_4096:
			t_mv =  adc_raw_final * ADS1118_CONST_4_096V_LSB_mV;
			break;
		case PGA_2048:
			t_mv =  adc_raw_final * ADS1118_CONST_2_048V_LSB_mV;
			break;
		case PGA_1024:
			t_mv =  adc_raw_final * ADS1118_CONST_1_024V_LSB_mV;
			break;
		case PGA_512:
			t_mv =  adc_raw_final * ADS1118_CONST_0_512V_LSB_mV;
			break;
		case PGA_256:
			t_mv =  adc_raw_final * ADS1118_CONST_0_256V_LSB_mV;
			break;
		default:
			break;
	}
	return t_mv;
}
//获取差分输入 AIN2 AIN3
uint16_t ads1118_get_differential_2_3(uint8_t PGA)
{
	uint16_t adc=0;

	ADS_InitTypeDef ConfigReg;

	ConfigReg.stru.NOP			=	DATA_VALID;
	ConfigReg.stru.TS_MODE	=	ADC_MODE;
	ConfigReg.stru.DR      =  DR_860_SPS;
	ConfigReg.stru.PGA      = PGA;
	ConfigReg.stru.MODE			=	SIGNLE_SHOT;
	ConfigReg.stru.OS				=	SINGLE_CONVER_START;   //high
	ConfigReg.stru.MUX			=	AINPN_2_3;
	ConfigReg.stru.PULLUP		= PULL_UP_EN;
	ConfigReg.stru.RESV			= CONFIG_BIT_RESV;
	ADS1118_ENABLE;
	delay_us((uint32_t)1);
	adc = SPI_read_write_Reg(ConfigReg.word);
	
	ADS1118_DISABLE;
	delay_ms(1);
	ADS1118_ENABLE;
	ConfigReg.stru.NOP			=	DATA_INVALID;
	ConfigReg.stru.TS_MODE	=	ADC_MODE;
	ConfigReg.stru.DR      =  DR_860_SPS;
	ConfigReg.stru.PGA      = PGA;
	ConfigReg.stru.MODE			=	SIGNLE_SHOT;
	ConfigReg.stru.OS				=	SINGLE_CONVER_START;   //high
	ConfigReg.stru.MUX			=	AINPN_2_3;
	ConfigReg.stru.PULLUP		= PULL_UP_EN;
	ConfigReg.stru.RESV			= CONFIG_BIT_RESV;
	delay_us((uint32_t)1);
	
	//等待DOUT给出数据有效信号-从高变低
	while(GPIO_ReadInputDataBit(SPI_MASTER_GPIO,SPI_MASTER_PIN_MISO));
	adc = SPI_read_write_Reg(ConfigReg.word);
	delay_us(1);
	ADS1118_DISABLE;
	return   adc;
}

double ads1118_get_differential_2_3_mv(uint8_t PGA)
{
	double t_mv=0.0;
	uint16_t adc_raw;
	adc_raw = ads1118_get_differential_2_3(PGA);
	
	switch(PGA){
		case PGA_6144:
			t_mv =  adc_raw * ADS1118_CONST_6_144V_LSB_mV;
			break;
		case PGA_4096:
			t_mv =  adc_raw * ADS1118_CONST_4_096V_LSB_mV;
			break;
		case PGA_2048:
			t_mv =  adc_raw * ADS1118_CONST_2_048V_LSB_mV;
			break;
		case PGA_1024:
			t_mv =  adc_raw * ADS1118_CONST_1_024V_LSB_mV;
			break;
		case PGA_512:
			t_mv =  adc_raw * ADS1118_CONST_0_512V_LSB_mV;
			break;
		case PGA_256:
			t_mv =  adc_raw * ADS1118_CONST_0_256V_LSB_mV;
			break;
		default:
			break;
	}
	return t_mv;
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
	
	ADS_InitTypeDef ConfigReg;

	ConfigReg.stru.NOP			=	DATA_VALID;
	ConfigReg.stru.TS_MODE	=	ADC_MODE;
	ConfigReg.stru.DR      =  DR_128_SPS;
	ConfigReg.stru.PGA      = PGA_2048;
	ConfigReg.stru.MODE			=	SIGNLE_SHOT;
	ConfigReg.stru.OS				=	SINGLE_CONVER_START;   //high
	ConfigReg.stru.MUX			=	channel;
	if(channel > 3)
		return 0;
	ADS1118_ENABLE;
	delay_us((uint32_t)1);
	adc = SPI_read_write_Reg(ConfigReg.word);
	ADS1118_DISABLE;
	delay_ms(1);
	
	ADS1118_ENABLE;
	ConfigReg.stru.NOP			=	DATA_INVALID;
	ConfigReg.stru.TS_MODE	=	ADC_MODE;
	ConfigReg.stru.DR      =  DR_128_SPS;
	ConfigReg.stru.PGA      = PGA_2048;
	ConfigReg.stru.MODE			=	SIGNLE_SHOT;
	ConfigReg.stru.OS				=	SINGLE_CONVER_START;   //high
	ConfigReg.stru.MUX			=	channel;
	delay_us((uint32_t)1);
	//等待DOUT给出数据有效信号-从高变低
	while(GPIO_ReadInputDataBit(SPI_MASTER_GPIO,SPI_MASTER_PIN_MISO));
	adc = SPI_read_write_Reg(ConfigReg.word);
	delay_us(1);
	ADS1118_DISABLE;
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
//温度电压值 K型热电偶
//第一列为温度 第二列为电压单位是mv
type_k_thermo_lookup_entry_table type_k_thermo_lookup[TYPE_K_THERMO_LOOKUP_SIZE] = {
{-200,-5.891},
{-100,-3.554},
{0,0},
{100,4.096},
{200,8.138},
{300,12.209},
{400,16.397},
{500,20.644},
{600,24.905},
{700,29.129},
{800,33.275},
{900,37.326},
{1000,41.276},
{1100,45.119},
{1200,48.838},
{1300,52.410}
};
int8_t  ads1118_get_temp_thermo_type_k( double input_voltage_mV, double input_cold_junction_C, double *output_hot_junction_C) {
    const type_k_thermo_lookup_entry_table  *type_k_thermo_lookup_table;
          uint8_t                type_k_thermo_lookup_size;

		uint32_t i=1;
    double total_mV;
		type_k_thermo_lookup_table = type_k_thermo_lookup;
		type_k_thermo_lookup_size  = TYPE_K_THERMO_LOOKUP_SIZE;
            

    // 查看数据是否超出范围
    if ( (input_cold_junction_C < type_k_thermo_lookup_table[0].temp_C) ||
         (input_cold_junction_C > type_k_thermo_lookup_table[type_k_thermo_lookup_size-1].temp_C) ) {
        *output_hot_junction_C = 0;
        return -1;
    }
    
   //查找冷端温度所在的位置
    while ( i<(type_k_thermo_lookup_size-1)) {
        if ( input_cold_junction_C <= type_k_thermo_lookup_table[i].temp_C ) {
            break;
        }
        i++;
    }
	//根据冷端温度的值，找出对应的电压值,并且计算总的电压值
		//该部分可以参考TI官方的方案
    total_mV = type_k_thermo_lookup_table[i-1].mV + 
          ( type_k_thermo_lookup_table[i].mV     - type_k_thermo_lookup_table[i-1].mV ) *
        ( ( input_cold_junction_C                   - type_k_thermo_lookup_table[i-1].temp_C ) / 
          ( type_k_thermo_lookup_table[i].temp_C - type_k_thermo_lookup_table[i-1].temp_C )  );
    
    total_mV += input_voltage_mV;
    
    //查看电压是否超出范围
    if ( (total_mV < type_k_thermo_lookup_table[0].mV) ||
         (total_mV > type_k_thermo_lookup_table[type_k_thermo_lookup_size-1].mV) ) {
        *output_hot_junction_C = 0;
        return -1;
    }

    //反向查表
    i=1;
    while ( i<(type_k_thermo_lookup_size-1) ) {
        if ( total_mV <= type_k_thermo_lookup_table[i].mV ) {
            break;
        }
        i++;
    }
		//将电压转换成温度
    *output_hot_junction_C = type_k_thermo_lookup_table[i-1].temp_C + 
          ( type_k_thermo_lookup_table[i].temp_C - type_k_thermo_lookup_table[i-1].temp_C ) *
        ( ( total_mV                      - type_k_thermo_lookup_table[i-1].mV ) / 
          ( type_k_thermo_lookup_table[i].mV     - type_k_thermo_lookup_table[i-1].mV )  );
    
    return 0;
}
//中位值滤波
//输入数组首地址
uint16_t ads1118_median_filter(uint16_t *pbuffer)  
{  
uint16_t value_buf_input[SAMPLECOUNTER];  
uint8_t i,j,temp;  
memcpy(value_buf_input,pbuffer,SAMPLECOUNTER);
for (j=0;j<=SAMPLECOUNTER;j++){  
   for (i=0;i<=SAMPLECOUNTER-j;i++){  
        if (value_buf_input[i] > value_buf_input[i+1])  
        {  
            temp = value_buf_input[i];  
            value_buf_input[i] = value_buf_input[i+1];  
            value_buf_input[i+1] = temp;  
        }  
    }  
}  
return value_buf_input[(SAMPLECOUNTER-1)/2]; //排序后取中间那个值 
}
//中位值平均滤波
double ads1118_median_average_filter(double *pbuffer) 
{ 
   uint8_t count,i,j; 
   double value_buf_input[SAMPLECOUNTER]; 
   double  sum=0,temp; 
   memcpy(value_buf_input,pbuffer,SAMPLECOUNTER);
	
   for (j=0;j<SAMPLECOUNTER-1;j++) 
   { 
      for (i=0;i<SAMPLECOUNTER-j;i++) 
      { 
         if ( value_buf_input[i]>value_buf_input[i+1] ) 
         { 
            temp = value_buf_input[i]; 
            value_buf_input[i] = value_buf_input[i+1];  
             value_buf_input[i+1] = temp; 
         } 
      } 
   } 
	 //去掉最小值与最大值 然后求平均值
   for(count=1;count<SAMPLECOUNTER-1;count++) 
      sum += value_buf_input[count]; 
   return (double)(sum/(SAMPLECOUNTER-2)); 
}
