/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : main.c
* Author             : ROBOTIS / project adaptation
* Version            : V0.0.1
* Date               : 08/23/2010
* Description        : CM-530 ROS AX position bridge using official SDK style
* 說明               : ROS 端送 AX-12A position 整數，CM-530 負責收命令、
*                      驗證參數、SYNC_WRITE 控制 AX-12A，並回 ACK/ERR。
* Note               : ROS sends AX position integers; CM-530 validates,
*                      sync-writes AX-12A motors, then replies ACK/ERR.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "dynamixel.h"
#include "dxl_hal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define P_TORQUE_ENABLE			24
#define P_GOAL_POSITION_L		30
#define P_GOAL_SPEED_L			32

#define DEFAULT_BAUDNUM			1		/* AX-12A: 1 Mbps */
#define NUM_JOINTS				4
#define MAX_POSITION			1023
#define HOME_POSITION			512
#define PC_CMD_BUFFER_SIZE		96
#define PC_RX_BUFFER_SIZE		128
#define PC_RX_BUFFER_MASK		(PC_RX_BUFFER_SIZE - 1)
/* 正式 ROS 對接必須關閉 echo，避免 ROS 收到自己送出的 command。 */
/* Formal ROS handoff: do not echo TX lines back to ROS. */
#define PC_ECHO_ENABLE			0
#define APPLY_OK				1
#define APPLY_ERR_TORQUE		2
#define APPLY_ERR_TX			3

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

/* ROS/IK 輸出順序固定為 j1,j2,j3,j4；這裡映射到 AX-12A ID。 */
/* Fixed ROS joint order: j1->ID17, j2->ID3, j3->ID2, j4->ID7. */
byte							gJointId[NUM_JOINTS] = {17, 3, 2, 7};
byte							gTorqueReady[NUM_JOINTS] = {0, 0, 0, 0};
byte							gLastTorqueErrorId = 0;
/* STOP 會重送最後目標位置來保持姿態，不關 torque、不回 HOME。 */
/* STOP re-sends last goal to hold pose; it does not torque off or HOME. */
word							gLastPos[NUM_JOINTS] = {
									HOME_POSITION,
									HOME_POSITION,
									HOME_POSITION,
									HOME_POSITION
								};
int								gTrajActive = 0;
int								gTrajId = 0;
int								gExpectedPoints = 0;
int								gReceivedPoints = 0;

/* Private function prototypes -----------------------------------------------*/
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

byte ReadLineFromPC(char *line, int maxLen);
void ProcessCommandLine(char *line);
int SplitArgs(char *line, char *argv[], int maxArgs);
int ParseInt(const char *text, int *outValue);
int StrEqual(const char *a, const char *b);
void ToUpperInPlace(char *text);
void NormalizeLine(char *line);

byte EnableAllTorque(byte reportErrors);
byte SyncWritePositions(word pos[NUM_JOINTS]);
byte ApplyPositions(word pos[NUM_JOINTS]);
void SetAllPositions(word pos[NUM_JOINTS], word value);
void ReplyOk(const char *text);
void ReplyOkInt(const char *prefix, int value);
void ReplyErr(const char *text);
void ReplyErrId(const char *prefix, int id);
void ReplyApplyError(byte result);

/*******************************************************************************
* Function Name  : main
* Description    : Main program
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int main(void)
{
	char line[PC_CMD_BUFFER_SIZE];

	RCC_Configuration();
	NVIC_Configuration();
	GPIO_Configuration();
	SysTick_Configuration();
	Timer_Configuration();

	dxl_initialize(0, DEFAULT_BAUDNUM);
	USART_Configuration(USART_PC, Baudrate_PC);

	/* 正式版開機只輸出 READY；ROS 可讀掉或忽略這一行。 */
	/* Formal startup emits only READY; ROS may flush or ignore it. */
	TxDString("READY\r\n");
	/* 開機 torque enable 是 best-effort 且靜默，避免干擾第一筆 ACK。 */
	/* Startup torque enable is best-effort and silent. */
	EnableAllTorque(0);

	while(1)
	{
		if(gbPcRxOverflow)
		{
			gbPcRxOverflow = 0;
			ReplyErr("OVERFLOW");
		}

		if(ReadLineFromPC(line, PC_CMD_BUFFER_SIZE))
			ProcessCommandLine(line);
	}

	return 0;
}

void ProcessCommandLine(char *line)
{
	char *argv[8];
	int argc;
	int trajId;
	int jointCount;
	int pointCount;
	int seq;
	int dtMs;
	int posValue;
	int i;
	byte applyResult;
	word pos[NUM_JOINTS];

	/* 清理輸入: 容忍全形逗號、BOM，丟掉不支援的控制字元。 */
	/* Normalize input: tolerate full-width comma/BOM and drop noise. */
	NormalizeLine(line);
	argc = SplitArgs(line, argv, 8);
	if(argc <= 0)
		return;

	ToUpperInPlace(argv[0]);

	if(StrEqual(argv[0], "PING"))
	{
		/* 通訊健康檢查 / serial health check. */
		if(argc != 1)
			ReplyErr("BAD_ARG");
		else
			TxDString("PONG\r\n");
		return;
	}

	if(StrEqual(argv[0], "AX"))
	{
		/* 手動或單點測試: AX,512 或 AX,j1,j2,j3,j4。 */
		/* Manual/single-point test command. */
		if(argc == 2)
		{
			if(!ParseInt(argv[1], &posValue))
			{
				ReplyErr("BAD_ARG");
				return;
			}

			if(posValue < 0 || posValue > MAX_POSITION)
			{
				ReplyErr("RANGE");
				return;
			}

			SetAllPositions(pos, (word)posValue);
		}
		else if(argc == 5)
		{
			for(i = 0; i < NUM_JOINTS; i++)
			{
				if(!ParseInt(argv[i + 1], &posValue))
				{
					ReplyErr("BAD_ARG");
					return;
				}

				if(posValue < 0 || posValue > MAX_POSITION)
				{
					ReplyErr("RANGE");
					return;
				}
				pos[i] = (word)posValue;
			}
		}
		else
		{
			ReplyErr("BAD_ARG");
			return;
		}

		applyResult = ApplyPositions(pos);
		if(applyResult == APPLY_OK)
			ReplyOk("AX");
		else
			ReplyApplyError(applyResult);
		return;
	}

	if(StrEqual(argv[0], "BEGIN"))
	{
		/* 正式軌跡開始: joint_count 必須固定為 4。 */
		/* Formal trajectory start; joint_count must be 4. */
		if(argc != 4 ||
		   !ParseInt(argv[1], &trajId) ||
		   !ParseInt(argv[2], &jointCount) ||
		   !ParseInt(argv[3], &pointCount))
		{
			ReplyErr("BAD_ARG");
			return;
		}

		if(jointCount != NUM_JOINTS || pointCount <= 0)
		{
			ReplyErr("BAD_ARG");
			return;
		}

		gTrajActive = 1;
		gTrajId = trajId;
		gExpectedPoints = pointCount;
		gReceivedPoints = 0;
		ReplyOkInt("BEGIN", trajId);
		return;
	}

	if(StrEqual(argv[0], "PT"))
	{
		/* 軌跡點: 收到合法點後立即 SYNC_WRITE，成功後回 OK,PT。 */
		/* Trajectory point: sync-write immediately, then ACK on success. */
		if(argc != 7 ||
		   !ParseInt(argv[1], &seq) ||
		   !ParseInt(argv[2], &dtMs))
		{
			ReplyErr("BAD_ARG");
			return;
		}

		if(!gTrajActive || gReceivedPoints >= gExpectedPoints || dtMs < 0)
		{
			ReplyErr("BAD_TRAJ");
			return;
		}

		for(i = 0; i < NUM_JOINTS; i++)
		{
			if(!ParseInt(argv[i + 3], &posValue))
			{
				ReplyErr("BAD_ARG");
				return;
			}

			if(posValue < 0 || posValue > MAX_POSITION)
			{
				ReplyErr("RANGE");
				return;
			}
			pos[i] = (word)posValue;
		}

		applyResult = ApplyPositions(pos);
		if(applyResult != APPLY_OK)
		{
			ReplyApplyError(applyResult);
			return;
		}

		gReceivedPoints++;
		ReplyOkInt("PT", seq);
		return;
	}

	if(StrEqual(argv[0], "END"))
	{
		/* 軌跡結束: 必須收到 BEGIN 宣告數量的全部 PT。 */
		/* Trajectory end: all declared PT points must have arrived. */
		if(argc != 2 || !ParseInt(argv[1], &trajId))
		{
			ReplyErr("BAD_ARG");
			return;
		}

		if(!gTrajActive || trajId != gTrajId || gReceivedPoints != gExpectedPoints)
		{
			ReplyErr("BAD_TRAJ");
			return;
		}

		gTrajActive = 0;
		ReplyOkInt("END", trajId);
		return;
	}

	if(StrEqual(argv[0], "STOP"))
	{
		/* STOP 是保持位置，不是斷電急停。 */
		/* STOP holds position; it is not a torque-off emergency stop. */
		if(argc != 1)
		{
			ReplyErr("BAD_ARG");
			return;
		}

		gTrajActive = 0;
		applyResult = ApplyPositions(gLastPos);
		if(applyResult == APPLY_OK)
			ReplyOk("STOP");
		else
			ReplyApplyError(applyResult);
		return;
	}

	if(StrEqual(argv[0], "HOME"))
	{
		/* HOME 將四軸送回 AX position 512。 */
		/* HOME sends all joints to AX position 512. */
		if(argc != 1)
		{
			ReplyErr("BAD_ARG");
			return;
		}

		SetAllPositions(pos, HOME_POSITION);
		applyResult = ApplyPositions(pos);
		if(applyResult == APPLY_OK)
			ReplyOk("HOME");
		else
			ReplyApplyError(applyResult);
		return;
	}

	ReplyErr("BAD_CMD");
}

byte EnableAllTorque(byte reportErrors)
{
	int i;
	int result;
	byte allOk = 1;

	gLastTorqueErrorId = 0;

	for(i = 0; i < NUM_JOINTS; i++)
	{
		if(gTorqueReady[i])
			continue;

		dxl_write_byte(gJointId[i], P_TORQUE_ENABLE, 1);
		result = dxl_get_result();
		if(result != COMM_RXSUCCESS)
		{
			if(gLastTorqueErrorId == 0)
				gLastTorqueErrorId = gJointId[i];
			if(reportErrors)
				ReplyErrId("DXL_TORQUE", gJointId[i]);
			allOk = 0;
		}

		/*
		 * AX-12A 有時命令已執行但 status packet 沒回或延遲。
		 * v1 不做 present-position readback，因此 torque enable 採
		 * best-effort；後續動作主要看 SYNC_WRITE 是否送出成功。
		 *
		 * AX-12A may execute WRITE even when the status packet is missing or
		 * delayed. v1 does not use readback, so torque enable is best-effort
		 * and motion is verified by TX success only.
		 */
		gTorqueReady[i] = 1;
	}

	return allOk;
}

byte SyncWritePositions(word pos[NUM_JOINTS])
{
	int i;
	int result;

	/* 用 broadcast SYNC_WRITE 一次寫四顆 Goal Position。 */
	/* Broadcast SYNC_WRITE writes Goal Position to all four motors at once. */
	dxl_set_txpacket_id(BROADCAST_ID);
	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
	dxl_set_txpacket_parameter(0, P_GOAL_POSITION_L);
	dxl_set_txpacket_parameter(1, 2);

	for(i = 0; i < NUM_JOINTS; i++)
	{
		dxl_set_txpacket_parameter(2 + 3 * i, gJointId[i]);
		dxl_set_txpacket_parameter(2 + 3 * i + 1, dxl_get_lowbyte(pos[i]));
		dxl_set_txpacket_parameter(2 + 3 * i + 2, dxl_get_highbyte(pos[i]));
	}

	dxl_set_txpacket_length((2 + 1) * NUM_JOINTS + 4);
	dxl_txrx_packet();

	result = dxl_get_result();
	if(result == COMM_TXSUCCESS || result == COMM_RXSUCCESS)
		return 1;

	return 0;
}

byte ApplyPositions(word pos[NUM_JOINTS])
{
	int i;

	/* 每次動作前再嘗試 torque enable，但不讓 timeout 阻塞主流程。 */
	/* Retry torque enable before motion without blocking on status timeout. */
	EnableAllTorque(0);

	if(!SyncWritePositions(pos))
		return APPLY_ERR_TX;

	for(i = 0; i < NUM_JOINTS; i++)
		gLastPos[i] = pos[i];

	return APPLY_OK;
}

void SetAllPositions(word pos[NUM_JOINTS], word value)
{
	int i;
	for(i = 0; i < NUM_JOINTS; i++)
		pos[i] = value;
}

byte ReadLineFromPC(char *line, int maxLen)
{
	static int index = 0;
	byte ch;

	while(CheckRxD_PC())
	{
		ch = RxDByte_PC();

		/* 接受 LF 或 CRLF；空行忽略。 */
		/* Accept LF or CRLF; ignore empty lines. */
		if(ch == '\r' || ch == '\n')
		{
			if(index == 0)
				continue;

#if PC_ECHO_ENABLE
			TxDString("\r\n");
#endif
			line[index] = 0;
			index = 0;
			return 1;
		}

		/* RoboPlus 手測時支援退格；正式 ROS 通常不會送 backspace。 */
		/* Support backspace for RoboPlus manual testing. */
		if(ch == '\b' || ch == 0x7F)
		{
			if(index > 0)
			{
				index--;
#if PC_ECHO_ENABLE
				TxDByte_PC('\b');
				TxDByte_PC(' ');
				TxDByte_PC('\b');
#endif
			}
			continue;
		}

		if(index < maxLen - 1)
		{
#if PC_ECHO_ENABLE
			TxDByte_PC(ch);
#endif
			line[index++] = (char)ch;
		}
		else
		{
			index = 0;
#if PC_ECHO_ENABLE
			TxDString("\r\n");
#endif
			ReplyErr("OVERFLOW");
		}
	}

	return 0;
}

int SplitArgs(char *line, char *argv[], int maxArgs)
{
	int argc = 0;
	char *p = line;
	char *start;
	char *end;
	char *lineEnd;
	char *trimEnd;

	/* 先把逗號切成多段，再逐段 trim 空白。 */
	/* Split by comma first, then trim spaces for each token. */
	lineEnd = line;
	while(*lineEnd)
		lineEnd++;

	for(p = line; p < lineEnd; p++)
	{
		if(*p == ',')
			*p = 0;
	}

	p = line;
	while(p <= lineEnd && argc < maxArgs)
	{
		start = p;
		while(start < lineEnd && (*start == ' ' || *start == '\t'))
			start++;

		end = p;
		while(end < lineEnd && *end)
			end++;

		trimEnd = end;
		while(trimEnd > start && (trimEnd[-1] == ' ' || trimEnd[-1] == '\t'))
			trimEnd--;
		*trimEnd = 0;

		argv[argc++] = start;

		p = end + 1;
	}

	return argc;
}

void NormalizeLine(char *line)
{
	char *src = line;
	char *dst = line;
	byte b0;
	byte b1;
	byte b2;

	while(*src)
	{
		b0 = (byte)src[0];
		b1 = (byte)src[1];
		b2 = (byte)src[2];

		/* UTF-8 full-width comma: EF BC 8C */
		if(b0 == 0xEF && b1 == 0xBC && b2 == 0x8C)
		{
			*dst++ = ',';
			src += 3;
			continue;
		}

		/* Drop UTF-8 BOM if a terminal ever sends it. */
		if(b0 == 0xEF && b1 == 0xBB && b2 == 0xBF)
		{
			src += 3;
			continue;
		}

		if((*src >= 'A' && *src <= 'Z') ||
		   (*src >= 'a' && *src <= 'z') ||
		   (*src >= '0' && *src <= '9') ||
		   *src == ',' ||
		   *src == '+' ||
		   *src == '-' ||
		   *src == ' ' ||
		   *src == '\t')
		{
			*dst++ = *src;
		}

		src++;
	}

	*dst = 0;
}

int ParseInt(const char *text, int *outValue)
{
	int value = 0;
	int digitCount = 0;

	if(*text == '+')
		text++;

	if(*text == 0)
		return 0;

	while(*text)
	{
		if(*text < '0' || *text > '9')
			return 0;

		value = value * 10 + (*text - '0');
		digitCount++;
		text++;
	}

	if(digitCount == 0)
		return 0;

	*outValue = value;
	return 1;
}

int StrEqual(const char *a, const char *b)
{
	while(*a && *b)
	{
		if(*a != *b)
			return 0;
		a++;
		b++;
	}

	return (*a == 0 && *b == 0);
}

void ToUpperInPlace(char *text)
{
	while(*text)
	{
		if(*text >= 'a' && *text <= 'z')
			*text = *text - ('a' - 'A');
		text++;
	}
}

void ReplyOk(const char *text)
{
	TxDString("OK,");
	TxDString(text);
	TxDString("\r\n");
}

void ReplyOkInt(const char *prefix, int value)
{
	TxDString("OK,");
	TxDString(prefix);
	TxDByte_PC(',');
	TxDInt(value);
	TxDString("\r\n");
}

void ReplyErr(const char *text)
{
	TxDString("ERR,");
	TxDString(text);
	TxDString("\r\n");
}

void ReplyErrId(const char *prefix, int id)
{
	TxDString("ERR,");
	TxDString(prefix);
	TxDByte_PC(',');
	TxDInt(id);
	TxDString("\r\n");
}

void ReplyApplyError(byte result)
{
	if(result == APPLY_ERR_TORQUE)
		ReplyErrId("DXL_TORQUE", gLastTorqueErrorId);
	else
		ReplyErr("DXL_TX");
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
	byte temp;
	byte next;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		temp = USART_ReceiveData(USART3);
		next = (gbPcRxBufferWritePointer + 1) & PC_RX_BUFFER_MASK;

		if(next == gbPcRxBufferReadPointer)
		{
			gbPcRxOverflow = 1;
			return;
		}

		gbpPcRxInterruptBuffer[gbPcRxBufferWritePointer] = temp;
		gbPcRxBufferWritePointer = next;
	}
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
