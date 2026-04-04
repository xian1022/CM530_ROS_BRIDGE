/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : main.c
* Author             : danceww
* Version            : V0.0.1
* Date               : 08/23/2010
* Description        : Main program body
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "dynamixel.h"
#include "dxl_hal.h"

/* Private define ------------------------------------------------------------*/
#define P_GOAL_POSITION_L        30
#define P_GOAL_POSITION_H        31
#define P_PRESENT_POSITION_L     36
#define P_PRESENT_POSITION_H     37
#define P_MOVING                 46

#define PORT_ENABLE_TXD          GPIOB
#define PORT_ENABLE_RXD          GPIOB
#define PORT_DXL_TXD             GPIOB
#define PORT_DXL_RXD             GPIOB

#define PIN_ENABLE_TXD           GPIO_Pin_4
#define PIN_ENABLE_RXD           GPIO_Pin_5
#define PIN_DXL_TXD              GPIO_Pin_6
#define PIN_DXL_RXD              GPIO_Pin_7
#define PIN_PC_TXD               GPIO_Pin_10
#define PIN_PC_RXD               GPIO_Pin_11

#define USART_DXL                0
#define USART_PC                 2

#define word                     u16
#define byte                     u8

/* Private variables ---------------------------------------------------------*/
volatile byte gbpRxInterruptBuffer[256];
volatile byte gbRxBufferWritePointer, gbRxBufferReadPointer;
volatile vu32 gwTimingDelay, gw1msCounter;

u32  Baudrate_DXL = 1000000;
u32  Baudrate_PC  = 57600;
vu16 CCR1_Val     = 100;     /* 1ms */
vu32 capture      = 0;

word GoalPos[2]   = {200, 800};
word wPresentPos1 = 0;
word wPresentPos2 = 0;

byte id1          = 9;
byte id2          = 18;
byte CommStatus   = 0;
byte pcCmd        = 0;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void NVIC_Configuration(void);
void GPIO_Configuration(void);
void SysTick_Configuration(void);
void Timer_Configuration(void);
void TimerInterrupt_1ms(void);
void RxD0Interrupt(void);
void __ISR_DELAY(void);
void USART1_Configuration(u32 baudrate);
void USART_Configuration(u8 PORT, u32 baudrate);
void DisableUSART1(void);
void ClearBuffer256(void);
byte CheckNewArrive(void);
void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);
void TxDByte_DXL(byte bTxdData);
byte RxDByte_DXL(void);
void TxDString(const char *bData);
void TxDWord16(word wSentData);
void TxDByte16(byte bSentData);
void TxDByte_PC(byte bTxdData);
void mDelay(u32 nTime);
void StartDiscount(s32 StartTime);
byte CheckTimeOut(void);
byte CheckRxD_PC(void);
byte RxDByte_PC(void);

/*******************************************************************************
* Function Name  : main
* Description    : Main program
*******************************************************************************/
int main(void)
{
    RCC_Configuration();
    NVIC_Configuration();
    GPIO_Configuration();
    SysTick_Configuration();
    Timer_Configuration();

    /* DXL bus init: baud num = 1 */
    dxl_initialize(0, 1);

    /* PC serial init */
    USART_Configuration(USART_PC, Baudrate_PC);

    mDelay(100);

    TxDString("\r\nCM-530 ready.\r\n");
    TxDString("Dual motor mode: ID9 + ID18\r\n");
    TxDString("Send 'a' -> both 200\r\n");
    TxDString("Send 'b' -> both 800\r\n");

    while (1)
    {
        if (CheckRxD_PC())
        {
            pcCmd = RxDByte_PC();

            if (pcCmd == '\r' || pcCmd == '\n')
                continue;

            if (pcCmd == 'a' || pcCmd == 'A')
            {
                ClearBuffer256();
                dxl_write_word(id1, P_GOAL_POSITION_L, GoalPos[0]);
                CommStatus = dxl_get_result();

                if (CommStatus != COMM_RXSUCCESS)
                {
                    TxDString("ID9 write fail\r\n");
                    PrintCommStatus(CommStatus);
                    continue;
                }

                mDelay(20);

                ClearBuffer256();
                dxl_write_word(id2, P_GOAL_POSITION_L, GoalPos[0]);
                CommStatus = dxl_get_result();

                if (CommStatus != COMM_RXSUCCESS)
                {
                    TxDString("ID18 write fail\r\n");
                    PrintCommStatus(CommStatus);
                    continue;
                }

                TxDString("CMD A -> ID9,ID18 = ");
                TxDWord16(GoalPos[0]);
                TxDString("\r\n");
                PrintErrorCode();
            }
            else if (pcCmd == 'b' || pcCmd == 'B')
            {
                ClearBuffer256();
                dxl_write_word(id1, P_GOAL_POSITION_L, GoalPos[1]);
                CommStatus = dxl_get_result();

                if (CommStatus != COMM_RXSUCCESS)
                {
                    TxDString("ID9 write fail\r\n");
                    PrintCommStatus(CommStatus);
                    continue;
                }

                mDelay(20);

                ClearBuffer256();
                dxl_write_word(id2, P_GOAL_POSITION_L, GoalPos[1]);
                CommStatus = dxl_get_result();

                if (CommStatus != COMM_RXSUCCESS)
                {
                    TxDString("ID18 write fail\r\n");
                    PrintCommStatus(CommStatus);
                    continue;
                }

                TxDString("CMD B -> ID9,ID18 = ");
                TxDWord16(GoalPos[1]);
                TxDString("\r\n");
                PrintErrorCode();
            }
            else
            {
                TxDString("Use only 'a' or 'b'.\r\n");
                continue;
            }

            mDelay(100);

            ClearBuffer256();
            wPresentPos1 = dxl_read_word(id1, P_PRESENT_POSITION_L);
            CommStatus = dxl_get_result();

            if (CommStatus == COMM_RXSUCCESS)
            {
                TxDString("ID9  POS -> ");
                TxDWord16(wPresentPos1);
                TxDString("\r\n");
                PrintErrorCode();
            }
            else
            {
                TxDString("ID9 read fail\r\n");
                PrintCommStatus(CommStatus);
            }

            mDelay(20);

            ClearBuffer256();
            wPresentPos2 = dxl_read_word(id2, P_PRESENT_POSITION_L);
            CommStatus = dxl_get_result();

            if (CommStatus == COMM_RXSUCCESS)
            {
                TxDString("ID18 POS -> ");
                TxDWord16(wPresentPos2);
                TxDString("\r\n");
                PrintErrorCode();
            }
            else
            {
                TxDString("ID18 read fail\r\n");
                PrintCommStatus(CommStatus);
            }
        }
    }

    return 0;
}

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures the different system clocks.
*******************************************************************************/
void RCC_Configuration(void)
{
    ErrorStatus HSEStartUpStatus;

    RCC_DeInit();
    RCC_HSEConfig(RCC_HSE_ON);
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS)
    {
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        FLASH_SetLatency(FLASH_Latency_2);

        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        RCC_PCLK2Config(RCC_HCLK_Div1);
        RCC_PCLK1Config(RCC_HCLK_Div2);

        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
        RCC_PLLCmd(ENABLE);

        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
        {
        }

        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

        while (RCC_GetSYSCLKSource() != 0x08)
        {
        }
    }

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3 | RCC_APB1Periph_TIM2, ENABLE);

    PWR_BackupAccessCmd(ENABLE);
}

/*******************************************************************************
* Function Name  : NVIC_Configuration
* Description    : Configures Vector Table base location.
*******************************************************************************/
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

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : GPIO_Configuration
* Description    : Configures the different GPIO ports.
*******************************************************************************/
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

    GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);   /* TX Disable */
    GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);     /* RX Enable */
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

    if (PORT == USART_DXL)
    {
        USART_DeInit(USART1);
        mDelay(10);
        USART_Init(USART1, &USART_InitStructure);
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
        USART_Cmd(USART1, ENABLE);
    }
    else if (PORT == USART_PC)
    {
        USART_DeInit(USART3);
        mDelay(10);
        USART_Init(USART3, &USART_InitStructure);
        USART_Cmd(USART3, ENABLE);
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
    if (gbRxBufferReadPointer != gbRxBufferWritePointer)
        return 1;
    else
        return 0;
}

void TxDByte_DXL(byte bTxdData)
{
    GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);   /* RX Disable */
    GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);     /* TX Enable */

    USART_SendData(USART1, bTxdData);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {
    }

    GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);   /* TX Disable */
    GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);     /* RX Enable */
}

byte RxDByte_DXL(void)
{
    byte bTemp;

    while (1)
    {
        if (gbRxBufferReadPointer != gbRxBufferWritePointer)
            break;
    }

    bTemp = gbpRxInterruptBuffer[gbRxBufferReadPointer];
    gbRxBufferReadPointer++;

    return bTemp;
}

byte CheckRxD_PC(void)
{
    if (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET)
        return 1;
    else
        return 0;
}

byte RxDByte_PC(void)
{
    return (byte)USART_ReceiveData(USART3);
}

/* Print communication result */
void PrintCommStatus(int CommStatus)
{
    switch (CommStatus)
    {
    case COMM_TXFAIL:
        TxDString("COMM_TXFAIL: Failed transmit instruction packet!\n");
        break;

    case COMM_TXERROR:
        TxDString("COMM_TXERROR: Incorrect instruction packet!\n");
        break;

    case COMM_RXFAIL:
        TxDString("COMM_RXFAIL: Failed get status packet from device!\n");
        break;

    case COMM_RXWAITING:
        TxDString("COMM_RXWAITING: Now recieving status packet!\n");
        break;

    case COMM_RXTIMEOUT:
        TxDString("COMM_RXTIMEOUT: There is no status packet!\n");
        break;

    case COMM_RXCORRUPT:
        TxDString("COMM_RXCORRUPT: Incorrect status packet!\n");
        break;

    default:
        TxDString("This is unknown error code!\n");
        break;
    }
}

/* Print error bit of status packet */
void PrintErrorCode(void)
{
    if (dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
        TxDString("Input voltage error!\n");

    if (dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
        TxDString("Angle limit error!\n");

    if (dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
        TxDString("Overheat error!\n");

    if (dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
        TxDString("Out of range error!\n");

    if (dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
        TxDString("Checksum error!\n");

    if (dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
        TxDString("Overload error!\n");

    if (dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
        TxDString("Instruction code error!\n");
}

void TxDString(const char *bData)
{
    while (*bData)
        TxDByte_PC((byte)*bData++);
}

void TxDWord16(word wSentData)
{
    TxDByte16((wSentData >> 8) & 0xff);
    TxDByte16(wSentData & 0xff);
}

void TxDByte16(byte bSentData)
{
    byte bTmp;

    bTmp = ((byte)(bSentData >> 4) & 0x0f) + (byte)'0';
    if (bTmp > '9')
        bTmp += 7;
    TxDByte_PC(bTmp);

    bTmp = (byte)(bSentData & 0x0f) + (byte)'0';
    if (bTmp > '9')
        bTmp += 7;
    TxDByte_PC(bTmp);
}

void TxDByte_PC(byte bTxdData)
{
    USART_SendData(USART3, bTxdData);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
    {
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
    if (TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);

        capture = TIM_GetCapture1(TIM2);
        TIM_SetCompare1(TIM2, capture + CCR1_Val);

        if (gw1msCounter > 0)
            gw1msCounter--;
    }
}

void RxD0Interrupt(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
        gbpRxInterruptBuffer[gbRxBufferWritePointer++] = USART_ReceiveData(USART1);
}

void SysTick_Configuration(void)
{
    SysTick_SetReload(9000);
    SysTick_ITConfig(ENABLE);
}

void __ISR_DELAY(void)
{
    if (gwTimingDelay != 0x00)
        gwTimingDelay--;
}

void mDelay(u32 nTime)
{
    SysTick_CounterCmd(SysTick_Counter_Enable);

    gwTimingDelay = nTime;
    while (gwTimingDelay != 0)
    {
    }

    SysTick_CounterCmd(SysTick_Counter_Disable);
    SysTick_CounterCmd(SysTick_Counter_Clear);
}

void StartDiscount(s32 StartTime)
{
    gw1msCounter = StartTime;
}

u8 CheckTimeOut(void)
{
    if (gw1msCounter == 0)
        return 1;
    else
        return 0;
}