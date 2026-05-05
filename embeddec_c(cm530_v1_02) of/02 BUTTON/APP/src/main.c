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
#define PORT_SW_UP				GPIOC
#define PORT_SW_DOWN			GPIOC
#define PORT_SW_RIGHT			GPIOA
#define PORT_SW_LEFT			GPIOA
#define PORT_SW_START			GPIOB

#define PORT_LED_AUX			GPIOB
#define PORT_LED_MANAGE			GPIOB
#define PORT_LED_PROGRAM		GPIOB
#define PORT_LED_PLAY			GPIOB
#define PORT_LED_POWER			GPIOC
#define PORT_LED_TX				GPIOC
#define PORT_LED_RX				GPIOC

#define PIN_SW_UP				GPIO_Pin_11
#define PIN_SW_DOWN				GPIO_Pin_10
#define PIN_SW_RIGHT			GPIO_Pin_14
#define PIN_SW_LEFT				GPIO_Pin_15
#define PIN_SW_START			GPIO_Pin_3

#define PIN_LED_AUX				GPIO_Pin_12
#define PIN_LED_MANAGE			GPIO_Pin_13
#define PIN_LED_PROGRAM			GPIO_Pin_14
#define PIN_LED_PLAY			GPIO_Pin_15
#define PIN_LED_POWER			GPIO_Pin_13
#define PIN_LED_TX				GPIO_Pin_14
#define PIN_LED_RX				GPIO_Pin_15

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);

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
	GPIO_Configuration();

	/* Configure the GPIO ports */
	NVIC_Configuration();

	GPIO_ResetBits(PORT_LED_POWER, PIN_LED_POWER);

	while(1)
	{
		if( GPIO_ReadInputDataBit(PORT_SW_UP, PIN_SW_UP) != SET )
			GPIO_ResetBits(PORT_LED_MANAGE, PIN_LED_MANAGE);

		else if( GPIO_ReadInputDataBit(PORT_SW_DOWN, PIN_SW_DOWN) != SET )
			GPIO_ResetBits(PORT_LED_AUX, PIN_LED_AUX);

		else if( GPIO_ReadInputDataBit(PORT_SW_LEFT, PIN_SW_LEFT) != SET )
			GPIO_ResetBits(PORT_LED_PROGRAM, PIN_LED_PROGRAM);

		else if( GPIO_ReadInputDataBit(PORT_SW_RIGHT, PIN_SW_RIGHT) != SET )
			GPIO_ResetBits(PORT_LED_PLAY, PIN_LED_PLAY);

		else if( GPIO_ReadInputDataBit(PORT_SW_START, PIN_SW_START) != SET )
		{
			GPIO_ResetBits(PORT_LED_MANAGE, PIN_LED_MANAGE);
			GPIO_ResetBits(PORT_LED_PROGRAM, PIN_LED_PROGRAM);
			GPIO_ResetBits(PORT_LED_PLAY, PIN_LED_PLAY);
			GPIO_ResetBits(PORT_LED_TX, PIN_LED_TX);
			GPIO_ResetBits(PORT_LED_RX, PIN_LED_RX);
			GPIO_ResetBits(PORT_LED_AUX, PIN_LED_AUX);
		}

		else
		{
			GPIO_SetBits(PORT_LED_MANAGE, PIN_LED_MANAGE);
			GPIO_SetBits(PORT_LED_PROGRAM, PIN_LED_PROGRAM);
			GPIO_SetBits(PORT_LED_PLAY, PIN_LED_PLAY);
			GPIO_SetBits(PORT_LED_TX, PIN_LED_TX);
			GPIO_SetBits(PORT_LED_RX, PIN_LED_RX);
			GPIO_SetBits(PORT_LED_AUX, PIN_LED_AUX);
		};
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

	/* Enable GPIOA, GPIOB, and GPIOC clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

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
	GPIO_InitStructure.GPIO_Pin = 	 PIN_SW_RIGHT | PIN_SW_LEFT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// PORTB CONFIG
	GPIO_InitStructure.GPIO_Pin = 	 PIN_SW_START;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = 	 PIN_LED_AUX | PIN_LED_MANAGE | PIN_LED_PROGRAM | PIN_LED_PLAY;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// PORTC CONFIG
	GPIO_InitStructure.GPIO_Pin = 	 PIN_SW_UP | PIN_SW_DOWN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = 	 PIN_LED_POWER | PIN_LED_TX | PIN_LED_RX;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}
