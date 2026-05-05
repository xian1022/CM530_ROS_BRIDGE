/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : main.c
* Author             : danceww
* Version            : V0.0.1
* Date               : 08/23/2010
* Description        : Main program body
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PORT_SIG_MOT1P          GPIOA
#define PORT_SIG_MOT1M          GPIOA

#define PORT_ADC_SELECT0        GPIOC
#define PORT_ADC_SELECT1        GPIOC

#define PORT_LED_POWER			GPIOC

#define PIN_SIG_MOT1P           GPIO_Pin_0
#define PIN_SIG_MOT1M           GPIO_Pin_1

#define PIN_ADC_SELECT0         GPIO_Pin_1
#define PIN_ADC_SELECT1         GPIO_Pin_2
#define PIN_ADC0				GPIO_Pin_0
#define PIN_VDD_VOLT			GPIO_Pin_3

#define PIN_PC_TXD				GPIO_Pin_10

#define PIN_LED_POWER			GPIO_Pin_13

#define word 					u16
#define byte 					u8

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
vu32 							gwTimingDelay;
word 							IR_1;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void SysTick_Configuration(void);
void ADC_Configuration(void);
void USART_Configuration(u32);
void __ISR_DELAY(void);
void mDelay(u32);
void uDelay(u32);
void TxDWord16(word);
void TxDByte16(byte);
void TxDByte_PC(byte);

/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/


int main(void)
{
    /* System Clocks Configuration */
	RCC_Configuration();

	/* NVIC configuration */
	NVIC_Configuration();

	/* GPIO configuration */
	GPIO_Configuration();

	SysTick_Configuration();

	/* ADC configuration */
	ADC_Configuration();

	GPIO_ResetBits(PORT_SIG_MOT1P,PIN_SIG_MOT1P);
	GPIO_ResetBits(PORT_SIG_MOT1M,PIN_SIG_MOT1M);

	//GPIO_ResetBits(PORT_LED_POWER, PIN_LED_POWER);


	while(1)
	{
		GPIO_SetBits(PORT_SIG_MOT1P, PIN_SIG_MOT1P);
		GPIO_ResetBits(PORT_SIG_MOT1M, PIN_SIG_MOT1M);

		GPIO_ResetBits(PORT_ADC_SELECT0,PIN_ADC_SELECT0);
		GPIO_ResetBits(PORT_ADC_SELECT1,PIN_ADC_SELECT1);

		uDelay(30);

		/* Start ADC1,ADC2 Software Conversion */
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);

		//ADC_SoftwareStartConvCmd(ADC2, ENABLE);
		uDelay(5);

		IR_1 = (ADC_GetConversionValue(ADC1));

		GPIO_ResetBits(PORT_SIG_MOT1P, PIN_SIG_MOT1P);
		GPIO_ResetBits(PORT_SIG_MOT1M, PIN_SIG_MOT1M);

		TxDWord16(IR_1);

		TxDByte_PC('\r');
		TxDByte_PC('\n');

		mDelay(5);
	}
	return 0;
}

void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable HSE */
	RCC_HSEConfig(RCC_HSE_ON);

	/* Wait till HSE is ready */
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus == SUCCESS)
	{
		/* Enable Prefetch Buffer */
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

		/* Flash 2 wait state */
		FLASH_SetLatency(FLASH_Latency_2);

		/* HCLK = SYSCLK */
		RCC_HCLKConfig(RCC_SYSCLK_Div1);

		/* PCLK2 = HCLK */
		RCC_PCLK2Config(RCC_HCLK_Div1);

		/* PCLK1 = HCLK/2 */
		RCC_PCLK1Config(RCC_HCLK_Div2);

		/* PLLCLK = 8MHz * 9 = 72 MHz */
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);

		/* Enable PLL */
		RCC_PLLCmd(ENABLE);

		/* Wait till PLL is ready */
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
		}

		/* Select PLL as system clock source */
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

		/* Wait till PLL is used as system clock source */
		while(RCC_GetSYSCLKSource() != 0x08)
		{
		}
	}

	/* Enable peripheral clocks --------------------------------------------------*/

	/* Enable GPIOA, GPIOB, GPIOC and ADC1 clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
			               RCC_APB2Periph_ADC1, ENABLE);

	/* Enable USART3 clocks */
	RCC_APB1PeriphClockCmd ( RCC_APB1Periph_USART3 , ENABLE);

	PWR_BackupAccessCmd(ENABLE);
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NVIC_Configuration(void)
{
	#ifdef  VECT_TAB_RAM
		// Set the Vector Table base location at 0x20000000
		NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
	#else  // VECT_TAB_FLASH
		// Set the Vector Table base location at 0x08003000
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);
	#endif
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void GPIO_Configuration(void)
{


	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	// PORTA CONFIG
	GPIO_InitStructure.GPIO_Pin = 	 PIN_SIG_MOT1P | PIN_SIG_MOT1M;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// PORTB CONFIG
	GPIO_InitStructure.GPIO_Pin = PIN_PC_TXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// PORTC CONFIG
	GPIO_InitStructure.GPIO_Pin =   PIN_ADC_SELECT0 | PIN_ADC_SELECT1 |  PIN_LED_POWER;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  PIN_ADC0 | PIN_VDD_VOLT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void ADC_Configuration(void)
{
	ADC_InitTypeDef ADC_InitStructure;

	ADC_StructInit(&ADC_InitStructure);

	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;

	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channels configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1 , ADC_SampleTime_239Cycles5);
	//ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);

	/* Enable ADC1 DMA */
	//ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC1 reset calibration register */
	/* Check the end of ADC1 reset calibration register */
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));


	/* Start ADC1 calibration */
	/* Check the end of ADC1 calibration */
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));


	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void SysTick_Configuration(void)
{
	  /* SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default) */
	  SysTick_SetReload(9);

	  /* Enable SysTick interrupt */
	  SysTick_ITConfig(ENABLE);
}

void __ISR_DELAY(void)
{
	if (gwTimingDelay != 0x00)
		gwTimingDelay--;
}

void mDelay(u32 nTime)
{
	uDelay(nTime*1000);
}

void uDelay(u32 nTime)
{
	  /* Enable the SysTick Counter */
	  SysTick_CounterCmd(SysTick_Counter_Enable);

	  gwTimingDelay = nTime;

	  while(gwTimingDelay != 0);

	  /* Disable SysTick Counter */
	  SysTick_CounterCmd(SysTick_Counter_Disable);
	  /* Clear SysTick Counter */
	  SysTick_CounterCmd(SysTick_Counter_Clear);
}


void USART_Configuration(u32 baudrate)
{
	USART_InitTypeDef USART_InitStructure;

	USART_StructInit(&USART_InitStructure);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;

	USART_DeInit(USART3);
	mDelay(10);
	/* Configure the USART3 */
	USART_Init(USART3, &USART_InitStructure);

	/* Enable USART3 Receive and Transmit interrupts */
	//USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//USART_ITConfig(USART3, USART_IT_TC, ENABLE);

	/* Enable the USART3 */
	USART_Cmd(USART3, ENABLE);
}

void TxDWord16(word wSentData) {
	TxDByte16((wSentData >> 8) & 0xff);
	TxDByte16(wSentData & 0xff);
}

void TxDByte16(byte bSentData) {
	byte bTmp;

	bTmp = ((byte) (bSentData >> 4) & 0x0f) + (byte) '0';
	if (bTmp > '9')
		bTmp += 7;
	TxDByte_PC(bTmp);
	bTmp = (byte) (bSentData & 0x0f) + (byte) '0';
	if (bTmp > '9')
		bTmp += 7;
	TxDByte_PC(bTmp);
}

void TxDByte_PC(byte bTxdData)
{
	USART_SendData(USART3,bTxdData);
	while( USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET );
}
