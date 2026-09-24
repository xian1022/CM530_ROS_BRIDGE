/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : main.c
* Author             : ROBOTIS / project adaptation
* Version            : V0.0.1
* Date               : 08/23/2010
* Description        : CM-530 v16 A/B AX position bridge using official SDK style
* 說明               : ROS 端送 AX-12A position 整數，CM-530 負責收命令、
*                      驗證參數、SYNC_WRITE 控制 AX-12A，並回 ACK/ERR。
* Note               : ROS sends AX position integers; CM-530 validates,
*                      sync-writes AX-12A motors, then replies ACK/ERR.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "dynamixel.h"
#include "dxl_hal.h"
#include "bridge.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define DEFAULT_BAUDNUM			1		/* AX-12A: 1 Mbps */
#define PC_RX_BUFFER_SIZE		128
#define PC_RX_BUFFER_MASK		(PC_RX_BUFFER_SIZE - 1)
/* 正式 ROS 對接必須關閉 echo，避免 ROS 收到自己送出的 command。 */
/* Formal ROS handoff: do not echo TX lines back to ROS. */

#define PORT_ENABLE_TXD			GPIOB
#define PORT_ENABLE_RXD			GPIOB
#define PORT_DXL_TXD			GPIOB
#define PORT_DXL_RXD			GPIOB

#define PIN_ENABLE_TXD			GPIO_Pin_4
#define PIN_ENABLE_RXD			GPIO_Pin_5
#define PIN_DXL_TXD				GPIO_Pin_6
#define PIN_DXL_RXD				GPIO_Pin_7
#define PIN_PC_TXD				GPIO_Pin_10
#define PIN_PC_RXD				GPIO_Pin_11

#define USART_DXL				0
#define USART_PC				2

#define word					u16
#define byte					u8

/* Private variables ---------------------------------------------------------*/
volatile byte					gbpRxInterruptBuffer[256]; /* DXL SDK buffer */
volatile byte					gbRxBufferWritePointer;
volatile byte					gbRxBufferReadPointer;

volatile byte					gbpPcRxInterruptBuffer[PC_RX_BUFFER_SIZE];
volatile byte					gbPcRxBufferWritePointer;
volatile byte					gbPcRxBufferReadPointer;
volatile byte					gbPcRxOverflow;

volatile vu32					gwTimingDelay;
volatile vu32					gw1msCounter;
vu16							CCR1_Val = 100; /* 1 ms */
vu32							capture = 0;

u32								Baudrate_DXL = 1000000;
u32								Baudrate_PC = 57600;

void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void USART1_Configuration(u32 baudrate);
void USART_Configuration(u8 PORT, u32 baudrate);
void SysTick_Configuration(void);
void Timer_Configuration(void);
void TimerInterrupt_1ms(void);
void RxD0Interrupt(void);
void RxD1Interrupt(void);
void __ISR_DELAY(void);
void DisableUSART1(void);
void ClearBuffer256(void);
byte CheckNewArrive(void);
byte CheckRxD_PC(void);
byte RxDByte_DXL(void);
byte RxDByte_PC(void);
void TxDByte_DXL(byte bTxdData);
void TxDByte_PC(byte bTxdData);
void TxDString(const char *bData);
void TxDInt(int value);
void mDelay(u32 nTime);
void StartDiscount(s32 StartTime);
byte CheckTimeOut(void);

/* A/B parser and independent motion state live in bridge.c. */
void BridgeOutput(const char *text) { TxDString(text); }

int main(void)
{
    byte ch;
    byte arrived;
    byte overflow;
    RCC_Configuration();
    NVIC_Configuration();
    GPIO_Configuration();
    SysTick_Configuration();
    Timer_Configuration();
    dxl_initialize(0, DEFAULT_BAUDNUM);
    USART_Configuration(USART_PC, Baudrate_PC);
    BridgeInit();
    BridgeEnableTorque();
    /* READY means initialization has finished; no command queue during it. */
    TxDString("READY\r\n");
    while (1) {
        /* Atomically take the overflow flag and a byte from the ISR queue. */
        USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);
        overflow = gbPcRxOverflow;
        gbPcRxOverflow = 0;
        arrived = CheckRxD_PC();
        ch = arrived ? RxDByte_PC() : 0;
        USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
        if (overflow) BridgeAbortLine();
        if (arrived) BridgeFeed(ch);
    }
    return 0;
}
void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;

	RCC_DeInit();
	RCC_HSEConfig(RCC_HSE_ON);
	HSEStartUpStatus = RCC_WaitForHSEStartUp();

	if(HSEStartUpStatus == SUCCESS)
	{
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		FLASH_SetLatency(FLASH_Latency_2);
		RCC_HCLKConfig(RCC_SYSCLK_Div1);
		RCC_PCLK2Config(RCC_HCLK_Div1);
		RCC_PCLK1Config(RCC_HCLK_Div2);
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		RCC_PLLCmd(ENABLE);

		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
		{
		}

		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		while(RCC_GetSYSCLKSource() != 0x08)
		{
		}
	}

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_USART3, ENABLE);
	PWR_BackupAccessCmd(ENABLE);
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

#ifdef VECT_TAB_RAM
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x3000);
#endif

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_ENABLE_TXD | PIN_ENABLE_RXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_DXL_RXD | PIN_PC_RXD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_DXL_TXD | PIN_PC_TXD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

	GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);
	GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);
}

void USART1_Configuration(u32 baudrate)
{
	USART_Configuration(USART_DXL, baudrate);
}

void USART_Configuration(u8 PORT, u32 baudrate)
{
	USART_InitTypeDef USART_InitStructure;

	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	if(PORT == USART_DXL)
	{
		USART_DeInit(USART1);
		mDelay(10);
		USART_Init(USART1, &USART_InitStructure);
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
		USART_Cmd(USART1, ENABLE);
	}
	else if(PORT == USART_PC)
	{
		USART_DeInit(USART3);
		mDelay(10);
		USART_Init(USART3, &USART_InitStructure);
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
		USART_Cmd(USART3, ENABLE);
	}
}

void Timer_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_DeInit(TIM2);

	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_PrescalerConfig(TIM2, 722, TIM_PSCReloadMode_Immediate);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Disable);

	TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void TimerInterrupt_1ms(void)
{
	if(TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		capture = TIM_GetCapture1(TIM2);
		TIM_SetCompare1(TIM2, capture + CCR1_Val);

		if(gw1msCounter > 0)
			gw1msCounter--;
	}
}

void DisableUSART1(void)
{
	USART_Cmd(USART1, DISABLE);
}

void ClearBuffer256(void)
{
	gbRxBufferReadPointer = 0;
	gbRxBufferWritePointer = 0;
}

byte CheckNewArrive(void)
{
	if(gbRxBufferReadPointer != gbRxBufferWritePointer)
		return 1;

	return 0;
}

byte CheckRxD_PC(void)
{
	if(gbPcRxBufferReadPointer != gbPcRxBufferWritePointer)
		return 1;

	return 0;
}

void TxDByte_DXL(byte bTxdData)
{
	GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);
	GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);

	USART_SendData(USART1, bTxdData);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{
	}

	GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);
	GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);
}

byte RxDByte_DXL(void)
{
	byte bTemp;

	while(1)
	{
		if(gbRxBufferReadPointer != gbRxBufferWritePointer)
			break;
	}

	bTemp = gbpRxInterruptBuffer[gbRxBufferReadPointer];
	gbRxBufferReadPointer++;
	return bTemp;
}

byte RxDByte_PC(void)
{
	byte bTemp;

	while(1)
	{
		if(gbPcRxBufferReadPointer != gbPcRxBufferWritePointer)
			break;
	}

	bTemp = gbpPcRxInterruptBuffer[gbPcRxBufferReadPointer];
	gbPcRxBufferReadPointer = (gbPcRxBufferReadPointer + 1) & PC_RX_BUFFER_MASK;
	return bTemp;
}

void TxDString(const char *bData)
{
	while(*bData)
		TxDByte_PC((byte)*bData++);
}

void TxDInt(int value)
{
	char buffer[12];
	int index = 0;
	int i;

	if(value == 0)
	{
		TxDByte_PC('0');
		return;
	}

	if(value < 0)
	{
		TxDByte_PC('-');
		value = -value;
	}

	while(value > 0 && index < 11)
	{
		buffer[index++] = (char)('0' + (value % 10));
		value /= 10;
	}

	for(i = index - 1; i >= 0; i--)
		TxDByte_PC((byte)buffer[i]);
}

void TxDByte_PC(byte bTxdData)
{
	USART_SendData(USART3, bTxdData);
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
	{
	}
}

void RxD0Interrupt(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
		gbpRxInterruptBuffer[gbRxBufferWritePointer++] = USART_ReceiveData(USART1);
}

void RxD1Interrupt(void)
{
    static byte discardUntilEol = 0;
    byte temp;
    byte next;
    if (USART_GetITStatus(USART3, USART_IT_RXNE) == RESET) return;
    temp = USART_ReceiveData(USART3);
    if (discardUntilEol) {
        if (temp != '\r' && temp != '\n') return;
        discardUntilEol = 0;
    }
    next = (gbPcRxBufferWritePointer + 1) & PC_RX_BUFFER_MASK;
    if (next == gbPcRxBufferReadPointer) {
        /* Flush damaged bytes; parser will discard through the next EOL. */
        gbPcRxOverflow = 1;
        gbPcRxBufferReadPointer = gbPcRxBufferWritePointer;
        if (temp != '\r' && temp != '\n') {
            discardUntilEol = 1;
            return;
        }
    }
    gbpPcRxInterruptBuffer[gbPcRxBufferWritePointer] = temp;
    gbPcRxBufferWritePointer = next;
}
void SysTick_Configuration(void)
{
	SysTick_SetReload(9000);
	SysTick_ITConfig(ENABLE);
}

void __ISR_DELAY(void)
{
	if(gwTimingDelay != 0x00)
		gwTimingDelay--;
}

void mDelay(u32 nTime)
{
	SysTick_CounterCmd(SysTick_Counter_Enable);
	gwTimingDelay = nTime;

	while(gwTimingDelay != 0)
	{
	}

	SysTick_CounterCmd(SysTick_Counter_Disable);
	SysTick_CounterCmd(SysTick_Counter_Clear);
}

void StartDiscount(s32 StartTime)
{
	gw1msCounter = StartTime;
}

byte CheckTimeOut(void)
{
	if(gw1msCounter == 0)
		return 1;

	return 0;
}
