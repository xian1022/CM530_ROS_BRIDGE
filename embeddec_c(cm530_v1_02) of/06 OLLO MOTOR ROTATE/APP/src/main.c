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

#define PORT_LED_POWER			GPIOC

#define PIN_SIG_MOT1P           GPIO_Pin_0
#define PIN_SIG_MOT1M           GPIO_Pin_1

#define PIN_LED_POWER			GPIO_Pin_13

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
vu32 gwTimingDelay;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void SysTick_Configuration(void);
void __ISR_DELAY(void);
void mDelay(u32);

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

	GPIO_ResetBits(PORT_SIG_MOT1P,PIN_SIG_MOT1P);
	GPIO_ResetBits(PORT_SIG_MOT1M,PIN_SIG_MOT1M);

	GPIO_ResetBits(PORT_LED_POWER, PIN_LED_POWER);

	while(1)
	{
		GPIO_SetBits(PORT_SIG_MOT1P,PIN_SIG_MOT1P);
		GPIO_ResetBits(PORT_SIG_MOT1M,PIN_SIG_MOT1M);

		mDelay(5000);

		GPIO_ResetBits(PORT_SIG_MOT1P,PIN_SIG_MOT1P);
		GPIO_SetBits(PORT_SIG_MOT1M,PIN_SIG_MOT1M);

		mDelay(5000);

		GPIO_SetBits(PORT_SIG_MOT1P,PIN_SIG_MOT1P);
		GPIO_SetBits(PORT_SIG_MOT1M,PIN_SIG_MOT1M);

		mDelay(2000);
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

	/* Enable GPIOA, and GPIOC clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

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

	// PORTC CONFIG
	GPIO_InitStructure.GPIO_Pin =   PIN_LED_POWER;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void SysTick_Configuration(void)
{
	/* SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default) */
	SysTick_SetReload(9000);

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
	/* Enable the SysTick Counter */
	SysTick_CounterCmd(SysTick_Counter_Enable);

	gwTimingDelay = nTime;

	while(gwTimingDelay != 0);

	/* Disable SysTick Counter */
	SysTick_CounterCmd(SysTick_Counter_Disable);
	/* Clear SysTick Counter */
	SysTick_CounterCmd(SysTick_Counter_Clear);
}
