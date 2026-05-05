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
#define PORT_LED_POWER			GPIOC

#define PIN_PC_TXD				GPIO_Pin_10
#define PIN_PC_RXD              GPIO_Pin_11
#define PIN_LED_POWER			GPIO_Pin_13

#define USART_PC			    2

#define byte                    u8

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile byte 					gbPacketWritePointer;
volatile byte 					gbPacketReadPointer;
volatile byte 					gbpPacketDataBuffer[16+1+16];
volatile vu32                   gwTimingDelay;

byte 							ReceivedData;

u32   							Baudrate_PC = 57600;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void USART_Configuration(u32);
void SysTick_Configuration(void);
byte RxD8Interrupt(void);
byte RxDByte_PC(void);
void TxDString(byte*);
void TxDByte_PC(byte);
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

	/* Configure the GPIO ports */
	GPIO_Configuration();

        SysTick_Configuration();

	/* USART Configuration */
	USART_Configuration(57600);

	GPIO_ResetBits(PORT_LED_POWER, PIN_LED_POWER);

	while(1)
	{
		ReceivedData = RxDByte_PC();

		if(ReceivedData == 'w')
			TxDString("Welcome!\r\n");

	    else if(ReceivedData == 'g')
	    	TxDString("Goodbye!\r\n");
	}
	return 0;
}

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
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

	/* Enable GPIOC clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	/* Enable USART3 clock */
	RCC_APB1PeriphClockCmd ( RCC_APB1Periph_USART3, ENABLE);

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
	NVIC_InitTypeDef NVIC_InitStructure;

	#ifdef  VECT_TAB_RAM
		// Set the Vector Table base location at 0x20000000
		NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
	#else  // VECT_TAB_FLASH
		// Set the Vector Table base location at 0x08003000
		NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);
	//	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	#endif

	// Configure the NVIC Preemption Priority Bits
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
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

	// PORTB CONFIG
	GPIO_InitStructure.GPIO_Pin = PIN_PC_RXD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = PIN_PC_TXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// PORTC CONFIG
	GPIO_InitStructure.GPIO_Pin = PIN_LED_POWER;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
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
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_DeInit(USART3);
	mDelay(10);
	/* Configure the USART3 */
	USART_Init(USART3, &USART_InitStructure);

	/* Enable USART3 Receive and Transmit interrupts */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	/* Enable the USART3 */
	USART_Cmd(USART3, ENABLE);
}

/*__interrupt*/
void RxD1Interrupt(void)
{
	byte temp;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		temp = USART_ReceiveData(USART3);
		gbpPacketDataBuffer[gbPacketWritePointer] = temp;
		gbPacketWritePointer++;
		gbPacketWritePointer = gbPacketWritePointer & 0x1F;
	}
}

byte RxDByte_PC(void)
{
    byte temp;

	while(1)
	{
    	if(gbPacketReadPointer != gbPacketWritePointer) break;
	}

	temp = gbpPacketDataBuffer[gbPacketReadPointer];
	gbPacketReadPointer++;
	gbPacketReadPointer = gbPacketReadPointer & 0x1F;

	return temp;
}

void TxDString(byte *bData)
{
	while (*bData)
		TxDByte_PC(*bData++);
}

void TxDByte_PC(byte bTxdData)
{
	USART_SendData(USART3,bTxdData);
	while( USART_GetFlagStatus(USART3, USART_FLAG_TC)==RESET );
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
