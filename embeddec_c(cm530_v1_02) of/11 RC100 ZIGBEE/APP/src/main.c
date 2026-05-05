/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : main.c
* Author             : danceww
* Version            : V0.0.1
* Date               : 08/23/2010
* Description        : Main program body
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "zgb_hal.h"
#include "zigbee.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PORT_LED_AUX			GPIOB
#define PORT_LED_MANAGE			GPIOB
#define PORT_LED_PROGRAM		GPIOB
#define PORT_LED_PLAY			GPIOB
#define PORT_LED_POWER			GPIOC
#define PORT_LED_TX				GPIOC
#define PORT_LED_RX				GPIOC
#define PORT_ZIGBEE_TXD			GPIOC
#define PORT_ZIGBEE_RXD			GPIOD
#define PORT_ZIGBEE_RESET		GPIOA

#define PIN_LED_AUX				GPIO_Pin_12
#define PIN_LED_MANAGE			GPIO_Pin_13
#define PIN_LED_PROGRAM			GPIO_Pin_14
#define PIN_LED_PLAY			GPIO_Pin_15
#define PIN_LED_POWER			GPIO_Pin_13
#define PIN_LED_TX				GPIO_Pin_14
#define PIN_LED_RX				GPIO_Pin_15
#define PIN_ZIGBEE_TXD			GPIO_Pin_12
#define PIN_ZIGBEE_RXD			GPIO_Pin_2
#define PIN_PC_TXD				GPIO_Pin_10
#define PIN_PC_RXD              GPIO_Pin_11
#define PIN_ZIGBEE_RESET		GPIO_Pin_12

#define RC100_BTN_U				(1)
#define RC100_BTN_D				(2)
#define RC100_BTN_L				(4)
#define RC100_BTN_R				(8)
#define RC100_BTN_1				(16)
#define RC100_BTN_2				(32)
#define RC100_BTN_3				(64)
#define RC100_BTN_4				(128)
#define RC100_BTN_5				(256)
#define RC100_BTN_6				(512)

#define USART_ZIGBEE		 	1
#define USART_PC			    2

#define PACKET_DATA0    		2
#define INVERSE_PACKET_DATA0 	3
#define PACKET_DATA1    		4
#define INVERSE_PACKET_DATA1 	5
#define PACKET_LENGTH 			6

#define word                    u16
#define byte                    u8
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
volatile byte                   gbPacketWritePointer = 0; // PC, Wireless
volatile byte                   gbPacketReadPointer = 0;
volatile byte                   gbPacketPointer = 0;
volatile byte                   gbpPacketDataBuffer[16+1+16];
volatile byte                   gbpPacket[PACKET_LENGTH+2];
volatile byte                   gbNewPacket;
volatile word                   gwZigbeeRxData;

u32                             Baudrate_ZIGBEE = 57600;
vu32                            gwTimingDelay;
word                            RcvData;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void USART_Configuration(u8, u32);
void SysTick_Configuration(void);
void RxD2Interrupt(void);
void _ISR_DELAY(void);
void EnableZigbee(void);
void DisableZigbee(void);
byte CheckNewArrive(void);
void mDelay(u32);
void PrintCommStatus(int);
void PrintErrorCode(void);
void TxDByte_Zigbee(byte);
byte RxDByte_Zigbee(void);
void TxDString(byte*);
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

	/* Configure the GPIO ports */
	GPIO_Configuration();

	SysTick_Configuration();

	zgb_initialize(0);
	USART_Configuration(USART_PC, 57600);

	GPIO_ResetBits(PORT_LED_POWER, PIN_LED_POWER);

	while(1)
	{
		if(zgb_rx_check() == 1)
		{
			RcvData = zgb_rx_data();

			TxDWord16(RcvData);
			TxDByte_PC('\r');
			TxDByte_PC('\n');

			if(RcvData & RC100_BTN_1)
				GPIO_ResetBits(PORT_LED_MANAGE, PIN_LED_MANAGE);
			else
				GPIO_SetBits(PORT_LED_MANAGE, PIN_LED_MANAGE);

			if(RcvData & RC100_BTN_2)
				GPIO_ResetBits(PORT_LED_PROGRAM, PIN_LED_PROGRAM);
			else
				GPIO_SetBits(PORT_LED_PROGRAM, PIN_LED_PROGRAM);

			if(RcvData & RC100_BTN_3)
				GPIO_ResetBits(PORT_LED_PLAY, PIN_LED_PLAY);
			else
				GPIO_SetBits(PORT_LED_PLAY, PIN_LED_PLAY);
		}
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

	/* Enable GPIOB and GPIOC clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

	/* Enable UART5 and USART3 clocks */
	RCC_APB1PeriphClockCmd ( RCC_APB1Periph_UART5 | RCC_APB1Periph_USART3, ENABLE);

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
	#endif

	// Configure the NVIC Preemption Priority Bits
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
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

	// PORTA CONFIG
	GPIO_InitStructure.GPIO_Pin = 	PIN_ZIGBEE_RESET;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// PORTB CONFIG
	GPIO_InitStructure.GPIO_Pin = 	PIN_LED_AUX | PIN_LED_MANAGE | PIN_LED_PROGRAM | PIN_LED_PLAY;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_PC_RXD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_PC_TXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// PORTC CONFIG
	GPIO_InitStructure.GPIO_Pin =   PIN_LED_POWER |PIN_LED_TX | PIN_LED_RX ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  PIN_ZIGBEE_TXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// PORTD CONFIG
	GPIO_InitStructure.GPIO_Pin = PIN_ZIGBEE_RXD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

void USART_Configuration(u8 PORT, u32 baudrate)
{
	USART_InitTypeDef USART_InitStructure;

	USART_StructInit(&USART_InitStructure);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	if( PORT == USART_ZIGBEE )
	{
		USART_DeInit(UART5);
		mDelay(10);
		/* Configure the UART5 */
		USART_Init(UART5, &USART_InitStructure);

		/* Enable UART5 Receive and Transmit interrupts */
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

		/* Enable the UART5 */
		USART_Cmd(UART5, ENABLE);
	}

	else if( PORT == USART_PC )
	{

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
}

void EnableZigbee(void)
{
	USART_Configuration(USART_ZIGBEE, 57600);
	GPIO_ResetBits(PORT_ZIGBEE_RESET, PIN_ZIGBEE_RESET);
}

void DisableZigbee(void)
{
	USART_Cmd(USART3, DISABLE);
	GPIO_SetBits(PORT_ZIGBEE_RESET, PIN_ZIGBEE_RESET);
}

byte CheckNewArrive(void)
{
	if(gbPacketReadPointer != gbPacketWritePointer)
		return 1;
	else
		return 0;
}

void TxDByte_Zigbee(byte bTxdData)
{
	USART_SendData(USART1,bTxdData);
	while( USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET );
}

byte RxDByte_Zigbee(void)
{
	byte bTemp;

	while(1)
	{
		if(gbPacketReadPointer != gbPacketWritePointer) break;
	}

	bTemp = gbpPacketDataBuffer[gbPacketReadPointer];
	gbPacketReadPointer++;

	return bTemp;
}

void RxD2Interrupt(void)
{
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{
		word temp;
		temp = USART_ReceiveData(UART5);

		gbpPacketDataBuffer[gbPacketWritePointer] = temp;
		gbPacketWritePointer++;
		gbPacketWritePointer = gbPacketWritePointer & 0x1F;
	}
}

void __ISR_DELAY(void)
{
	if (gwTimingDelay != 0x00)
		gwTimingDelay--;
}

void TxDString(byte *bData)
{
	while (*bData)
		TxDByte_PC(*bData++);
}

void TxDWord16(word wSentData)
{
	TxDByte16((wSentData >> 8) & 0xff);
	TxDByte16(wSentData & 0xff);
}

void TxDByte16(byte bSentData)
{
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

void SysTick_Configuration(void)
{
	/* SysTick end of count event each 1ms with input clock equal to 9MHz (HCLK/8, default) */
	SysTick_SetReload(9000);

	/* Enable SysTick interrupt */
	SysTick_ITConfig(ENABLE);
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
