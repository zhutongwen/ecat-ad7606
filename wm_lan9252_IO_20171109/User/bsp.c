/****************************************Copyright (c)****************************************************
**--------------File Info---------------------------------------------------------------------------------
** File name:               .c
** Descriptions:           ADC channel0 hardware driver
**
**--------------------------------------------------------------------------------------------------------
** Created by:              lyd
** Created date:            2015-05-01
** Version:                 v1.0
** Descriptions:            The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:             
** Modified date:           
** Version:                 
** Descriptions:            
**
*********************************************************************************************************/
#include "bsp.h"
	
/* Private variables ---------------------------------------------------------*/	
u16 uhADCxConvertedValue = 0;



/*********************************************************************************************************
* Function Name  : 函数：SystickInit
* Description    : 系统滴答定时器初始化
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*********************************************************************************************************/
void SystickInit(void)
{	
	SYSTICK_CURRENT = 0;	    	//初值设为0；
	SYSTICK_LOAD  	= 168000/8 - 1;;		//装载值为：17999；这样刚好2ms产出一次中断,
	SYSTICK_CTRLR   = 0x03; 		//开启计时器，开启系统时钟中断，时钟输入频率为：72MH/8=9MHZ
}	
/*********************************************************************************************************
* Function Name  : LedInit
* Description    : LED初始化
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*********************************************************************************************************/
void LedInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* LED_0 PB0 */ 
	/* LED_1 PB1 */ 
	/* LED_2 PB10 */ 
	/* LED_3 PB11 */ 
	/* LED_0 PB12 */ 
	/* LED_1 PB13 */ 
	/* LED_2 PB14 */ 
	/* LED_3 PB15 */ 	
	GPIO_InitStruct.GPIO_Pin = 	GPIO_Pin_0 |
								GPIO_Pin_1 | 
								GPIO_Pin_10 | 
								GPIO_Pin_11 |
								GPIO_Pin_12 | 
								GPIO_Pin_13 | 
								GPIO_Pin_14 | 
								GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	LED_0 = 1;
	LED_1 = 1;
	LED_2 = 1;
	LED_3 = 1;
	LED_4 = 1;
	LED_5 = 1;
	LED_6 = 1;
	LED_7 = 1;
}
/*********************************************************************************************************
* Function Name  : KeyInit
* Description    : 按键初始化
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*********************************************************************************************************/
void KeyInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* KEY0 PC0 */ 
	/* KEY1 PC1 */
	/* KEY2 PC2 */ 
	/* KEY3 PC3 */
	/* KEY4 PA6 */ 
	/* KEY5 PA7 */
	/* KEY6 PC4 */ 
	/* KEY7 PC5 */
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_0 | 
								GPIO_Pin_1 |
								GPIO_Pin_2 |
								GPIO_Pin_3 |
								GPIO_Pin_4 |
								GPIO_Pin_5; //
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin =  GPIO_Pin_6 | 
								GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/*********************************************************************************************************
* Function Name  : Uart2Init
* Description    : 串口2初始化
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*********************************************************************************************************/
void Uart2Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  USART_InitTypeDef USART_InitStruct;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,  ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

  /* USART2 Tx PB10 */	/* USART2 Rx PB11 */
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;//| GPIO_Pin_3;
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  USART_InitStruct.USART_BaudRate = 115200;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStruct.USART_Mode = USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStruct);
  USART_Cmd(USART2, ENABLE);

  USART_ClearFlag(USART2, USART_FLAG_TC);
}

/*********************************************************************************************************
* Function Name  : CRC_CHECK
* Description    : CRC校验，详情参考虚拟示波器软件(VisualScope_Cracked.exe)的help
* Input          : u16 *Buf 待校验的数组
* Input          : u8 CRC_CNT  数组中的元素数量
* Output         : None
* Return         : None
* Attention		 : None
*********************************************************************************************************/
static uint16_t CRC_CHECK(u16 *Buf, u8 CRC_CNT)
{
	uint16_t CRC_Temp;
	unsigned char i,j;
	CRC_Temp = 0xffff; 
	for (i=0;i<CRC_CNT; i++)
	{      
		CRC_Temp ^= Buf[i];
		for (j=0;j<16;j++) 
		{
			if (CRC_Temp & 0x01)
				CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
			else
				CRC_Temp = CRC_Temp >> 1;
		}
	}
	return(CRC_Temp);
} 

/*********************************************************************************************************
* Function Name  : OutPut_Data
* Description    : 向上位机发送数据。详情参考虚拟示波器软件(VisualScope_Cracked.exe)的help
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*********************************************************************************************************/
void OutPut_Data(u16 *OutData)
{
	u8 i;
	OutData[4] = CRC_CHECK(OutData,4);	
	for(i=0;i<5;i++)	  
	{
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);	//判断是否发送完毕
		USART_SendData(USART2,OutData[i]);	
		
		while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);	//判断是否发送完毕
		USART_SendData(USART2,OutData[i]>>8);
	}	
} 

/*********************************************************************************************************
* Function Name  : AdcInit
* Description    : ADC初始化.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*********************************************************************************************************/
void AdcInit(void)
{	
  ADC_InitTypeDef       ADC_InitStructure;
  ADC_CommonInitTypeDef ADC_CommonInitStructure;
  DMA_InitTypeDef       DMA_InitStructure;
  GPIO_InitTypeDef      GPIO_InitStructure;

  /* Enable ADCx, DMA and GPIO clocks ****************************************/ 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  /* DMA2 Stream0 channel2 configuration **************************************/
  DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&ADC1->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&uhADCxConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 1;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//工作在循环缓存模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA通道 x拥有高优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA2_Stream0, ENABLE);

  /* Configure ADC1 Channe6 pin as analog input ******************************/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* ADC Common Init **********************************************************/
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_CommonInitStructure.ADC_Prescaler =ADC_Prescaler_Div4;//ADC_Prescaler_Div2; 24M
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC3 Init ****************************************************************/
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel5 configuration **************************************/
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_3Cycles);

 /* Enable DMA request after last transfer (Single-ADC mode) */
  ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

  /* Enable ADC3 DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* Enable ADC3 */
  ADC_Cmd(ADC1, ENABLE);
	
//	DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);

 /* Start ADC Software Conversion */ 
  ADC_SoftwareStartConv(ADC1);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Spi1Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure; 
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);  
	
	//片选管脚，由软件控制
	SPI1_NSS = 1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//spi管脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI1); //打开引脚的复用功能
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI1);
	
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;   //作为主机使用
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;   //数据长度
    SPI_InitStructure.SPI_CPOL  = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;   //软件设置NSS功能
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1,&SPI_InitStructure);
    SPI_Cmd(SPI1,ENABLE);
}

void Ad7606Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //开启时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
//	#define ADC_OS0 		PEO(2)
//	#define ADC_OS1 		PEO(1)
//	#define ADC_OS2 		PBO(9)
//	#define ADC_CONVERT 	PBO(8)
//	#define ADC_REST 		PBO(7)
//	#define ADC_BUSY 		PBI(5)
    GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_7 |
									GPIO_Pin_8 |
									GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_1 |
									GPIO_Pin_2;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
//	OS2 OS1 OS2 ：组合状态选择过采样模式。
//	000表示无过采样，	最大200Ksps采样速率。
//	001表示2倍过采样， 	也就是硬件内部采集2个样本求平均
//	010表示4倍过采样， 	也就是硬件内部采集4个样本求平均
//	011表示8倍过采样， 	也就是硬件内部采集8个样本求平均
//	100表示16倍过采样， 也就是硬件内部采集16个样本求平均
//	101表示32倍过采样， 也就是硬件内部采集32个样本求平均
//	110表示64倍过采样， 也就是硬件内部采集64个样本求平均
	ADC_OS0 = 1;
	ADC_OS1 = 0;
	ADC_OS2 = 1;
	ADC_REST = 0;
	ADC_CONVERT = 1;
	Spi1Init();
}





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*********************************************************************************************************
* Function Name  : Tim2Init
* Description    : Tim2Init program.
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*********************************************************************************************************/
void Tim2Init(void)		//100us
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);

	TIM_DeInit(TIM2);
	TIM_TimeBaseStructure.TIM_Period= 0xFFFF;		 					/* ×Ô¶¯ÖØ×°ÔØ¼Ä´æÆ÷ÖÜÆÚµÄÖµ(¼ÆÊýÖµ) */
	TIM_TimeBaseStructure.TIM_Prescaler= (16800/2) - 1;				/* Ê±ÖÓÔ¤·ÖÆµÊý */
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 			/* ²ÉÑù·ÖÆµ */
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 		/* ÏòÉÏ¼ÆÊýÄ£Ê½ */
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM2, ENABLE);
}


/*********************************************************************************************************
* Function Name  : ExitPe0Init
* Description    : PE0外部中断初始化，下降沿触发。对应lan9252的 IRQ 中断
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*********************************************************************************************************/
void ExitPe0Init(void)
{	
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure PC0 pin as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd =GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Connect EXTI Line0 to PC0 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);

	/* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  				
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/*********************************************************************************************************
* Function Name  : ExitPd6Init
* Description    : PD6外部中断初始化，下降沿触发。对应lan9252的 SYNC0 中断
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*********************************************************************************************************/
void ExitPd6Init(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	/* Enable GPIOC clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure PC3 pin as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Connect EXTI Line3 to PC3 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource6);

	/* Configure EXTI Line3 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line3 Interrupt to the lowest priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  				
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

/*********************************************************************************************************
* Function Name  : ExitPa11Init
* Description    : PA11外部中断初始化，下降沿触发。对应lan9252的 SYNC1 中断
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*********************************************************************************************************/
void ExitPa11Init(void)
{
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;

	/* Enable GPIOC clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure PC1 pin as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Connect EXTI Line1 to PC1 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource11);

	/* Configure EXTI 1*/
	EXTI_InitStructure.EXTI_Line = EXTI_Line11;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line1 Interrupt to the lowest priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  				
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}



#define Bank1_SRAM1_ADDR    ((u32)( 0x60000000))
/*******************************************************************************
* Function Name  : FsmcInit
* Description    : 总线初始化，总线用于驱动lan9252
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void FsmcInit(void)
{
 	FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
	FSMC_NORSRAMTimingInitTypeDef  WriteTiming; 
	GPIO_InitTypeDef GPIO_InitStructure; 

	/* Enable GPIOs clock */
	RCC_AHB1PeriphClockCmd(	RCC_AHB1Periph_GPIOD | 
							RCC_AHB1Periph_GPIOE , ENABLE);

	/* Enable FSMC clock */
	RCC_AHB3PeriphClockCmd(RCC_AHB3Periph_FSMC, ENABLE); 
  
/*-- GPIOs Configuration -----------------------------------------------------*/
/*
	| PD0  <-> FSMC_D2  | 									         
	| PD1  <-> FSMC_D3  | 									
		| PD4  <-> FSMC_NOE | 			
		| PD5  <-> FSMC_NWE |
		| PD7  <-> FSMC_NE1 | 
	| PD8  <-> FSMC_D13 |									
	| PD9  <-> FSMC_D14 | 									 
	| PD10 <-> FSMC_D15 | 									 
	| PD14 <-> FSMC_D0  |
	| PD15 <-> FSMC_D1  | 	

		| PE3  <-> FSMC_A19   |
		| PE4  <-> FSMC_A20   |
		| PE5  <-> FSMC_A21   |
		| PE6  <-> FSMC_A22   |
	| PE7  <-> FSMC_D4   |
	| PE8  <-> FSMC_D5   |                  
	| PE9  <-> FSMC_D6   |                    
	| PE10 <-> FSMC_D7   |
	| PE11 <-> FSMC_D8   |
	| PE12 <-> FSMC_D9   |
	| PE13 <-> FSMC_D10  |
	| PE14 <-> FSMC_D11  |
	| PE15 <-> FSMC_D12  |
*/

	/* GPIOD configuration */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource4, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource7, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource10, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_0  | 
									GPIO_Pin_1  | 
									GPIO_Pin_4  | 
									GPIO_Pin_5  |
									GPIO_Pin_7  |
									GPIO_Pin_8  |		
									GPIO_Pin_9  |  
									GPIO_Pin_10 | 
									GPIO_Pin_14 | 
									GPIO_Pin_15;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* GPIOE configuration */
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource3 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource4 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource7 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource8 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource10 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource12 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14 , GPIO_AF_FSMC);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource15 , GPIO_AF_FSMC);

	GPIO_InitStructure.GPIO_Pin = 	GPIO_Pin_3 |
									GPIO_Pin_4 |
									GPIO_Pin_5 |
									GPIO_Pin_6 |
									GPIO_Pin_7 |
									GPIO_Pin_8  | 
									GPIO_Pin_9  | 
									GPIO_Pin_10 | 
									GPIO_Pin_11 |
									GPIO_Pin_12 | 
									GPIO_Pin_13 | 
									GPIO_Pin_14 | 
									GPIO_Pin_15;

	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	/*-- FSMC Configuration ------------------------------------------------------*/
	WriteTiming.FSMC_AddressSetupTime = 11;	 //??????(ADDSET)?1?HCLK 1/168M=6ns
	WriteTiming.FSMC_AddressHoldTime = 11;	 //??????(ADDHLD)??A???	
	WriteTiming.FSMC_DataSetupTime = 11;		 //??????(DATAST)?9?HCLK 6*9=54ns	 	 
	WriteTiming.FSMC_BusTurnAroundDuration = 11;
	WriteTiming.FSMC_CLKDivision = 11;
	WriteTiming.FSMC_DataLatency = 0;
	WriteTiming.FSMC_AccessMode = 0;	 	

	FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM1;//FSMC_Bank1_NORSRAM1;
	FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable; //????????
	FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;  //?????
	FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;  //16?????
	FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;  //??????
	FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;  
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
	FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
	FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
	FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
	FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
	FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &WriteTiming;
	FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &WriteTiming;
	FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 

	/*!< Enable FSMC Bank1_SRAM2 Bank */
	FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM1, ENABLE);  
}

u32 PMPReadDWord (u16 Address)
{    
    U32 res;	
	*(__IO uint16_t*) (Bank1_SRAM1_ADDR + (0x0<<20)) = Address;
	res.word[0] = *(__IO uint16_t*) (Bank1_SRAM1_ADDR + (0x2<<20));
	res.word[1] = *(__IO uint16_t*) (Bank1_SRAM1_ADDR + (0x3<<20));
    return res.dword;
}

void PMPWriteDWord (u16 Address, u32 Val)
{ 
	U32 res;
	res.dword=Val;
	*(__IO uint16_t*) (Bank1_SRAM1_ADDR + (0x0<<20)) = Address;
	*(__IO uint16_t*) (Bank1_SRAM1_ADDR + (0x1<<20)) = Address;
	*(__IO uint16_t*) (Bank1_SRAM1_ADDR + (0x2<<20)) = res.word[0];	
	*(__IO uint16_t*) (Bank1_SRAM1_ADDR + (0x3<<20)) = res.word[1];	
}

void PMPReadRegUsingCSR(u8 *ReadBuffer, u16 Address, u8 Count)
{
	U32 param32_1 = {0};
	u8 i = 0;
	param32_1.word[0] = Address;
    param32_1.byte[2] = Count;
    param32_1.byte[3] = ESC_READ_BYTE;
    PMPWriteDWord (CSR_CMD_REG, param32_1.dword);
    do
    {
        param32_1.dword = PMPReadDWord (ESC_CSR_CMD_REG);
    }while(param32_1.byte[3] & ESC_CSR_BUSY);

    param32_1.dword = PMPReadDWord (ESC_CSR_DATA_REG);
	for(i=0;i<Count;i++) ReadBuffer[i] = param32_1.byte[i];	
    return;
}

void PMPWriteRegUsingCSR( u8 *WriteBuffer, u16 Address, u8 Count)
{
    U32 param32_1 = {0};
    u8 i = 0;
		
    for(i=0;i<Count;i++)	param32_1.byte[i] = WriteBuffer[i];		
    PMPWriteDWord (ESC_CSR_DATA_REG, param32_1.dword);		
	param32_1.word[0] = Address;
    param32_1.byte[2] = Count;
    param32_1.byte[3] = ESC_WRITE_BYTE;
    PMPWriteDWord (0x304, param32_1.dword);
	do
	{
		param32_1.dword = PMPReadDWord (0x304);
	}while(param32_1.byte[3] & ESC_CSR_BUSY);
    return;
}

void PMPReadPDRamRegister(u8 *ReadBuffer, u16 Address, u16 Count)
{
    U32 param32_1 = {0};
    u8 i = 0,nlength, nBytePosition;
    u8 nReadSpaceAvblCount;

    /*Reset/Abort any previous commands.*/
    param32_1.dword = PRAM_RW_ABORT_MASK;   
	PMPWriteDWord (PRAM_READ_CMD_REG, param32_1.dword);

	/*The host should not modify this field unless the PRAM Read Busy
	(PRAM_READ_BUSY) bit is a 0.*/
	do
	{
		param32_1.dword = PMPReadDWord (PRAM_READ_CMD_REG);
	}while((param32_1.byte[3] & PRAM_RW_BUSY_8B));

	/*Write address and length in the EtherCAT Process RAM Read Address and
	* Length Register (ECAT_PRAM_RD_ADDR_LEN)*/
	param32_1.word[0] = Address;
	param32_1.word[1] = Count;
	PMPWriteDWord (PRAM_READ_ADDR_LEN_REG, param32_1.dword);
	param32_1.dword = PMPReadDWord (HBI_INDEXED_DATA2_REG );
	/*Set PRAM Read Busy (PRAM_READ_BUSY) bit(-EtherCAT Process RAM Read Command Register)
	*  to start read operatrion*/
	param32_1.dword = PRAM_RW_BUSY_32B; 
	PMPWriteDWord (PRAM_READ_CMD_REG, param32_1.dword);
	/*Read PRAM Read Data Available (PRAM_READ_AVAIL) bit is set*/
	do
	{
		param32_1.dword = PMPReadDWord (PRAM_READ_CMD_REG);
	}while(!(param32_1.byte[0] & IS_PRAM_SPACE_AVBL_MASK));

    nReadSpaceAvblCount = param32_1.byte[1] & PRAM_SPACE_AVBL_COUNT_MASK;
    /*Fifo registers are aliased address. In indexed it will read indexed data reg 0x04, but it will point to reg 0
     In other modes read 0x04 FIFO register since all registers are aliased*/    
    param32_1.dword = PMPReadDWord (PRAM_READ_FIFO_REG);
  
    nReadSpaceAvblCount--;
    nBytePosition = (Address & 0x03);
    nlength = (4-nBytePosition) > Count ? Count:(4-nBytePosition);
    memcpy(ReadBuffer+i ,&param32_1.byte[nBytePosition],nlength);
    Count-=nlength;
    i+=nlength;

    while(Count && nReadSpaceAvblCount)
    {
        param32_1.dword = PMPReadDWord (PRAM_READ_FIFO_REG);
        nlength = Count > 4 ? 4: Count;
        memcpy((ReadBuffer+i) ,&param32_1,nlength);
        i+=nlength;
        Count-=nlength;
        nReadSpaceAvblCount --;
        if (!nReadSpaceAvblCount)
        {
			param32_1.dword = PMPReadDWord (PRAM_READ_CMD_REG);
			nReadSpaceAvblCount = param32_1.byte[1] & PRAM_SPACE_AVBL_COUNT_MASK;
        }
    }
    return;
}
        
void PMPWritePDRamRegister(u8 *WriteBuffer, u16 Address, u16 Count)
{
    U32 param32_1 = {0};
    u8 i = 0,nlength, nBytePosition,nWrtSpcAvlCount;
    
	/*Reset or Abort any previous commands.*/
	param32_1.dword = PRAM_RW_ABORT_MASK;                                                 /*TODO:replace with #defines*/
	PMPWriteDWord (PRAM_WRITE_CMD_REG, param32_1.dword);
	/*Make sure there is no previous write is pending
	(PRAM Write Busy) bit is a 0 */
	do
	{
		param32_1.dword = PMPReadDWord (PRAM_WRITE_CMD_REG);
	}while((param32_1.byte[3] & PRAM_RW_BUSY_8B));

	/*Write Address and Length Register (ECAT_PRAM_WR_ADDR_LEN) with the
	starting byte address and length)*/
	param32_1.word[0] = Address;
	param32_1.word[1] = Count;

	PMPWriteDWord (PRAM_WRITE_ADDR_LEN_REG, param32_1.dword);
    /*write to the EtherCAT Process RAM Write Command Register (ECAT_PRAM_WR_CMD) with the  PRAM Write Busy
    (PRAM_WRITE_BUSY) bit set*/

    param32_1.dword = PRAM_RW_BUSY_32B; 
	PMPWriteDWord (PRAM_WRITE_CMD_REG, param32_1.dword);
	/*Read PRAM write Data Available (PRAM_READ_AVAIL) bit is set*/
	do
	{
		param32_1.dword = PMPReadDWord (PRAM_WRITE_CMD_REG);
	}while(!(param32_1.byte[0] & IS_PRAM_SPACE_AVBL_MASK));

    /*Check write data available count*/
    nWrtSpcAvlCount = param32_1.byte[1] & PRAM_SPACE_AVBL_COUNT_MASK;

    /*Write data to Write FIFO) */ 
    /*get the byte lenth for first read*/
    nBytePosition = (Address & 0x03);

    nlength = (4-nBytePosition) > Count ? Count:(4-nBytePosition);

    param32_1.dword = 0;
    memcpy(&param32_1.byte[nBytePosition],WriteBuffer+i, nlength);
    PMPWriteDWord (PRAM_WRITE_FIFO_REG,param32_1.dword);
    nWrtSpcAvlCount--;
    Count-=nlength;
    i+=nlength;

    while(nWrtSpcAvlCount && Count)
    {
        nlength = Count > 4 ? 4: Count;
        param32_1.dword = 0;
        memcpy(&param32_1, (WriteBuffer+i), nlength);
   
        PMPWriteDWord (PRAM_WRITE_FIFO_REG,param32_1.dword);
   
		i+=nlength;
        Count-=nlength;
        nWrtSpcAvlCount--;

        if (!nWrtSpcAvlCount)
        {
			param32_1.dword = PMPReadDWord (PRAM_WRITE_CMD_REG);
			/*Check write data available count*/
			nWrtSpcAvlCount = param32_1.byte[1] & PRAM_SPACE_AVBL_COUNT_MASK;
        }
    }
    return;
}
void PMPReadDRegister(u8 *ReadBuffer, u16 Address, u16 Count)
{
    if (Address >= 0x1000)
    {
         PMPReadPDRamRegister(ReadBuffer, Address,Count);
    }
    else
    {
         PMPReadRegUsingCSR(ReadBuffer, Address,Count);
    }
}
void PMPWriteRegister( u8 *WriteBuffer, u16 Address, u16 Count)
{ 
   if (Address >= 0x1000)
   {
		PMPWritePDRamRegister(WriteBuffer, Address,Count);
   }
   else
   {
		PMPWriteRegUsingCSR(WriteBuffer, Address,Count);
   }
}

//no more--------------------------------------------------------------

