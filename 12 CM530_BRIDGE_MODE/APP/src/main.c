/************************* (C) COPYRIGHT 2010 ROBOTIS **************************
* File Name          : main.c
* Author             : modified for PhantomX Pincher trajectory playback
* Version            : V2.0.0 (integer AX position version)
* Date               : 2026/04/05
* Description        : CM-530 trajectory player for 4x AX-12A (integer version)
*
* 中文說明：
* 本版本採用「整數 AX position位置」協定，避免舊 WinARM toolchain 的
* float / atof / printf 浮點格式化所造成的連結問題。
*
* ROS / PC 端輸入格式：
*   PING
*   HELP
*   BEGIN,<traj_id>,<joint_count>,<point_count>
*   PT,<seq>,<dt_ms>,<j1_pos>,<j2_pos>,<j3_pos>,<j4_pos>
*   END,<traj_id>
*   STOP
*   HOME
*
* 4 顆馬達 ID 從下至上對應：
*   j1 -> ID17
*   j2 -> ID3
*   j3 -> ID2
*   j4 -> ID7
*
* 英文說明：
* This version uses integer AX position protocol.
* ROS converts rad -> AX position first, then CM-530 directly writes
* Goal Position to AX-12A, reads Present Position back, and returns ACK.
******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "dynamixel.h" 
#include "dxl_hal.h"

#include <stdlib.h>
#include <string.h>

/* Private define ------------------------------------------------------------*/
/* AX-12A Control Table Address
 * AX-12A 控制表位址
 */
#define P_TORQUE_ENABLE         24
#define P_LED                   25
#define P_GOAL_POSITION_L       30
#define P_GOAL_POSITION_H       31
#define P_MOVING_SPEED_L        32
#define P_MOVING_SPEED_H        33
#define P_PRESENT_POSITION_L    36
#define P_PRESENT_POSITION_H    37
#define P_MOVING                46

/* CM-530 GPIO / UART Mapping
 * CM-530 GPIO / UART 腳位定義
 */
#define PORT_ENABLE_TXD         GPIOB
#define PORT_ENABLE_RXD         GPIOB
#define PORT_DXL_TXD            GPIOB
#define PORT_DXL_RXD            GPIOB

#define PIN_ENABLE_TXD          GPIO_Pin_4
#define PIN_ENABLE_RXD          GPIO_Pin_5
#define PIN_DXL_TXD             GPIO_Pin_6
#define PIN_DXL_RXD             GPIO_Pin_7
#define PIN_PC_TXD              GPIO_Pin_10
#define PIN_PC_RXD              GPIO_Pin_11

#define USART_DXL               0
#define USART_PC                2

/* DXL bus baud number used by old ROBOTIS SDK
 * ROBOTIS 舊 SDK 中 baudnum=1 對應 1 Mbps
 */
#define DXL_BAUDNUM             1

/* AX-12A Position Range
 * AX-12A 位置值範圍
 */
#define AX_CENTER_POS           512
#define AX_MIN_POS              0
#define AX_MAX_POS              1023

/* Fixed joint count for this project
 * 本專題固定控制 4 軸
 */
#define JOINT_COUNT_FIXED       4

/* UART line buffer size
 * 串口單行緩衝區大小
 */
#define RX_LINE_BUF_SIZE        128
#define MAX_CMD_ARGS            12

/* Final reach check
 * 最終到位判定參數
 */
#define FINAL_REACH_TOL         10
#define FINAL_REACH_TIMEOUT_MS  3000

/* Small delays between write/read
 * 每次 DXL 寫入 / 回讀之間的小延遲
 */
#define DXL_WRITE_GAP_MS        5
#define FEEDBACK_GAP_MS         5

/* HOME motion wait time
 * HOME 指令後等待時間
 */
#define HOME_MOVE_DELAY_MS      1000

#define word                    u16
#define byte                    u8

/* Private variables ---------------------------------------------------------*/
volatile byte gbpRxInterruptBuffer[256];
volatile byte gbRxBufferWritePointer, gbRxBufferReadPointer;
volatile vu32 gwTimingDelay, gw1msCounter;

u32  Baudrate_DXL = 1000000;
u32  Baudrate_PC  = 57600;   /* ROS / PC side baud rate */
vu16 CCR1_Val     = 100;     /* 1 ms tick */
vu32 capture      = 0;

byte CommStatus   = 0;

/* Models --------------------------------------------------------------------*/
/* Joint state
 * 每顆馬達的狀態
 */
typedef struct
{
    byte id;            /* AX-12A ID */
    word target_pos;    /* Target goal position / 目標位置 */
    word actual_pos;    /* Present position / 實際位置 */
    word home_pos;      /* Home position / HOME 位置 */
    word min_pos;       /* Safety minimum / 安全下限 */
    word max_pos;       /* Safety maximum / 安全上限 */
} JointState;

/* Trajectory state
 * 軌跡執行狀態
 */
typedef struct
{
    byte active;            /* Is a trajectory active? / 是否在軌跡模式 */
    byte began;             /* BEGIN received / 是否收到 BEGIN */
    byte ended;             /* END received / 是否收到 END */
    byte stop_requested;    /* STOP requested / 是否收到 STOP */

    byte traj_id;           /* Current trajectory id / 當前軌跡 ID */
    byte joint_count;       /* Must be 4 / 固定應為 4 */
    word point_count;       /* Total points / 總點數 */

    int expected_seq;       /* Expected sequence number / 預期 seq */
    int last_seq;           /* Last processed seq / 最後處理的 seq */
} TrajectoryState;

/* Global states -------------------------------------------------------------*/
JointState gJoints[JOINT_COUNT_FIXED];
TrajectoryState gTraj;

/* Private function prototypes -----------------------------------------------*/
/* Original / Low-Level Functions
 * 官方底層函式宣告
 */
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

/* App / User Logic
 * 應用層邏輯函式宣告
 */
void InitApplication(void);
void InitTrajectoryState(void);
void InitJointTable(void);
void PrintBanner(void);
void PrintHelp(void);

byte ReadLineFromPC(char *buf, int max_len);
int  SplitCSV(char *line, char *argv[], int max_args);
void ProcessCommandLine(char *line);

void HandlePing(void);
void HandleHelp(void);
void HandleBegin(char *argv[], int argc);
void HandlePoint(char *argv[], int argc);
void HandleEnd(char *argv[], int argc);
void HandleStop(void);
void HandleHome(void);

word ClampWord(word value, word min_v, word max_v);

byte EnableTorque(byte id, byte enable);
byte WriteGoalPosition(byte id, word pos);
byte ReadPresentPosition(byte id, word *pos);
byte MoveAllJoints(JointState joints[], int count);
byte ReadAllJoints(JointState joints[], int count);
byte AreAllJointsReached(JointState joints[], int count, word tol);
byte WaitAllJointsReachTarget(JointState joints[], int count, word tol, u32 timeout_ms);

void TxDDecU16(word value);
void TxDDecS32(int value);
void PrintTargetState(int seq, int dt_ms, JointState joints[], int count);
void PrintActualState(int seq, JointState joints[], int count);
void PrintFinalReached(JointState joints[], int count);
void PrintEndTag(void);
void PrintErrorMsg(const char *err);
void PrintAck1(const char *cmd, int v1);

/*******************************************************************************
* Function Name  : main
* Description    : Main program entry
*
* 中文：
* 1. 初始化時脈 / GPIO / UART / timer
* 2. 初始化 DXL 匯流排與 PC 串口
* 3. 顯示開機資訊與 HELP
* 4. 持續接收 ROS / PC 的 ASCII 指令
*******************************************************************************/
int main(void)
{
    char lineBuf[RX_LINE_BUF_SIZE];

    RCC_Configuration();
    NVIC_Configuration();
    GPIO_Configuration();
    SysTick_Configuration();
    Timer_Configuration();

    /* Initialize DYNAMIXEL bus
     * 初始化 DYNAMIXEL 匯流排
     */
    dxl_initialize(0, DXL_BAUDNUM);

    /* Initialize PC serial port
     * 初始化 PC/ROS 串口
     */
    USART_Configuration(USART_PC, Baudrate_PC);

    mDelay(100);

    InitApplication();
    PrintBanner();
    PrintHelp();

    while (1)
    {
        if (ReadLineFromPC(lineBuf, RX_LINE_BUF_SIZE))
        {
            ProcessCommandLine(lineBuf);
        }
    }

    return 0;
}

/*******************************************************************************
* InitApplication
* Description : Initialize application states and enable motor torque.
*
* 中文：初始化應用層狀態並打開 4 顆馬達 torque
*******************************************************************************/
void InitApplication(void)
{
    int i;

    InitTrajectoryState();
    InitJointTable();

    for (i = 0; i < JOINT_COUNT_FIXED; i++)
    {
        EnableTorque(gJoints[i].id, 1);
        mDelay(DXL_WRITE_GAP_MS);
    }
}

/*******************************************************************************
* InitTrajectoryState
* Description : Reset trajectory state machine.
*
* 中文：重設軌跡狀態機
*******************************************************************************/
void InitTrajectoryState(void)
{
    gTraj.active = 0;
    gTraj.began = 0;
    gTraj.ended = 0;
    gTraj.stop_requested = 0;
    gTraj.traj_id = 0;
    gTraj.joint_count = 0;
    gTraj.point_count = 0;
    gTraj.expected_seq = 0;
    gTraj.last_seq = -1;
}

/*******************************************************************************
* InitJointTable
* Description : Initialize 4 joints for PhantomX Pincher.
*
* 中文：
* 初始化 4 顆 AX-12A 的 ID 與安全範圍。
* 注意：home_pos / min_pos / max_pos 目前先用通用預設值，
* 實機上線後請依你們機構校正結果再調整。
*******************************************************************************/
void InitJointTable(void)
{
    /* j1 -> ID17 */
    gJoints[0].id         = 17;
    gJoints[0].target_pos = AX_CENTER_POS;
    gJoints[0].actual_pos = AX_CENTER_POS;
    gJoints[0].home_pos   = AX_CENTER_POS;
    gJoints[0].min_pos    = AX_MIN_POS;
    gJoints[0].max_pos    = AX_MAX_POS;

    /* j2 -> ID3 */
    gJoints[1].id         = 3;
    gJoints[1].target_pos = AX_CENTER_POS;
    gJoints[1].actual_pos = AX_CENTER_POS;
    gJoints[1].home_pos   = AX_CENTER_POS;
    gJoints[1].min_pos    = AX_MIN_POS;
    gJoints[1].max_pos    = AX_MAX_POS;

    /* j3 -> ID2 */
    gJoints[2].id         = 2;
    gJoints[2].target_pos = AX_CENTER_POS;
    gJoints[2].actual_pos = AX_CENTER_POS;
    gJoints[2].home_pos   = AX_CENTER_POS;
    gJoints[2].min_pos    = AX_MIN_POS;
    gJoints[2].max_pos    = AX_MAX_POS;

    /* j4 -> ID7 */
    gJoints[3].id         = 7;
    gJoints[3].target_pos = AX_CENTER_POS;
    gJoints[3].actual_pos = AX_CENTER_POS;
    gJoints[3].home_pos   = AX_CENTER_POS;
    gJoints[3].min_pos    = AX_MIN_POS;
    gJoints[3].max_pos    = AX_MAX_POS;
}

/*******************************************************************************
* PrintBanner
* Description : Print startup banner.
*
* 中文：開機後列印版本資訊
*******************************************************************************/
void PrintBanner(void)
{
    TxDString("\r\n========================================\r\n");
    TxDString("CM-530 PhantomX Pincher Trajectory Player\r\n");
    TxDString("Integer AX Position Version\r\n");
    TxDString("Joint Order: j1->ID17, j2->ID3, j3->ID2, j4->ID7\r\n");
    TxDString("PC Serial : ROS -> COM4 @ 57600 (PC side)\r\n");
    TxDString("========================================\r\n");
}

/*******************************************************************************
* PrintHelp
* Description : Print command help.
*
* 中文：列印支援指令說明
*******************************************************************************/
void PrintHelp(void)
{
    TxDString("Commands:\r\n");
    TxDString("PING\r\n");
    TxDString("HELP\r\n");
    TxDString("BEGIN,<traj_id>,<joint_count>,<point_count>\r\n");
    TxDString("PT,<seq>,<dt_ms>,<j1_pos>,<j2_pos>,<j3_pos>,<j4_pos>\r\n");
    TxDString("END,<traj_id>\r\n");
    TxDString("STOP\r\n");
    TxDString("HOME\r\n");
}

/*******************************************************************************
* ReadLineFromPC
* Description : Read one ASCII line from PC UART until '\n'.
*
* 中文：
* 從 PC/ROS 串口讀取一整行 ASCII 指令，直到收到 '\n'
* 成功讀到完整一行時回傳 1，否則回傳 0
*******************************************************************************/
byte ReadLineFromPC(char *buf, int max_len)
{
    static int idx = 0;
    byte ch;

    while (CheckRxD_PC())
    {
        ch = RxDByte_PC();

        /* Ignore carriage return
         * 忽略 '\r'
         */
        if (ch == '\r')
            continue;

        /* End of line
         * 一行結束
         */
        if (ch == '\n')
        {
            buf[idx] = '\0';
            idx = 0;
            return 1;
        }

        if (idx < (max_len - 1))
        {
            buf[idx++] = (char)ch;
        }
        else
        {
            /* Buffer overflow
             * 緩衝區溢位，丟棄本行
             */
            idx = 0;
            PrintErrorMsg("ERR,LINE_OVERFLOW");
            return 0;
        }
    }

    return 0;
}

/*******************************************************************************
* SplitCSV
* Description : Split a line by comma.
*
* 中文：以逗號切割指令字串
*******************************************************************************/
int SplitCSV(char *line, char *argv[], int max_args)
{
    int argc;
    char *tok;

    argc = 0;
    tok = strtok(line, ",");

    while (tok != 0 && argc < max_args)
    {
        argv[argc++] = tok;
        tok = strtok(0, ",");
    }

    return argc;
}

/*******************************************************************************
* ProcessCommandLine
* Description : Dispatch command by first token.
*
* 中文：根據第一個 token 分派到不同命令處理函式
*******************************************************************************/
void ProcessCommandLine(char *line)
{
    char work[RX_LINE_BUF_SIZE];
    char *argv[MAX_CMD_ARGS];
    int argc;

    if (line[0] == '\0')
        return;

    strncpy(work, line, RX_LINE_BUF_SIZE - 1);
    work[RX_LINE_BUF_SIZE - 1] = '\0';

    argc = SplitCSV(work, argv, MAX_CMD_ARGS);

    if (argc <= 0)
        return;

    if (strcmp(argv[0], "PING") == 0)
    {
        HandlePing();
    }
    else if (strcmp(argv[0], "HELP") == 0)
    {
        HandleHelp();
    }
    else if (strcmp(argv[0], "BEGIN") == 0)
    {
        HandleBegin(argv, argc);
    }
    else if (strcmp(argv[0], "PT") == 0)
    {
        HandlePoint(argv, argc);
    }
    else if (strcmp(argv[0], "END") == 0)
    {
        HandleEnd(argv, argc);
    }
    else if (strcmp(argv[0], "STOP") == 0)
    {
        HandleStop();
    }
    else if (strcmp(argv[0], "HOME") == 0)
    {
        HandleHome();
    }
    else
    {
        PrintErrorMsg("ERR,UNKNOWN_CMD");
    }
}

/*******************************************************************************
* HandlePing
* Description : Test connectivity between PC and CM-530.
*
* 中文：測試 PC/ROS 與 CM-530 是否連通
*******************************************************************************/
void HandlePing(void)
{
    TxDString("PONG\r\n");
}

/*******************************************************************************
* HandleHelp
* Description : Print help again.
*
* 中文：重新列印 HELP
*******************************************************************************/
void HandleHelp(void)
{
    PrintHelp();
}

/*******************************************************************************
* HandleBegin
* Format      : BEGIN,<traj_id>,<joint_count>,<point_count>
*
* 中文：
* 宣告一條新軌跡開始，建立軌跡狀態
*******************************************************************************/
void HandleBegin(char *argv[], int argc)
{
    int traj_id;
    int joint_count;
    int point_count;

    if (argc != 4)
    {
        PrintErrorMsg("ERR,FIELD_COUNT,BEGIN");
        return;
    }

    traj_id = atoi(argv[1]);
    joint_count = atoi(argv[2]);
    point_count = atoi(argv[3]);

    if (joint_count != JOINT_COUNT_FIXED)
    {
        PrintErrorMsg("ERR,JOINT_COUNT");
        return;
    }

    InitTrajectoryState();

    gTraj.active = 1;
    gTraj.began = 1;
    gTraj.ended = 0;
    gTraj.stop_requested = 0;
    gTraj.traj_id = (byte)traj_id;
    gTraj.joint_count = (byte)joint_count;
    gTraj.point_count = (word)point_count;
    gTraj.expected_seq = 0;
    gTraj.last_seq = -1;

    PrintAck1("BEGIN", traj_id);
}

/*******************************************************************************
* HandlePoint
* Format      : PT,<seq>,<dt_ms>,<j1_pos>,<j2_pos>,<j3_pos>,<j4_pos>
*
* 中文：
* 這是整數版軌跡播放的核心函式
* 1. 檢查 BEGIN 是否已收到
* 2. 檢查欄位數與 seq
* 3. 解析 4 個 AX position 整數
* 4. clamp 到安全範圍
* 5. 印出 target
* 6. 下發到 4 顆 AX-12A
* 7. 等待 dt_ms
* 8. 回讀 actual position
* 9. 印出 actual
* 10. 回 OK,PT,<seq>
*******************************************************************************/
void HandlePoint(char *argv[], int argc)
{
    int seq;
    int dt_ms;
    int i;
    int pos_i;

    if (!gTraj.active)
    {
        PrintErrorMsg("ERR,NO_BEGIN");
        return;
    }

    if (argc != 7)
    {
        PrintErrorMsg("ERR,FIELD_COUNT,PT");
        return;
    }

    if (gTraj.stop_requested)
    {
        PrintErrorMsg("ERR,STOP_REQUESTED");
        return;
    }

    seq = atoi(argv[1]);
    dt_ms = atoi(argv[2]);

    /* Sequence must be continuous
     * seq 必須連續遞增
     */
    if (seq != gTraj.expected_seq)
    {
        PrintErrorMsg("ERR,SEQ");
        return;
    }

    if (dt_ms < 0)
    {
        PrintErrorMsg("ERR,DT");
        return;
    }

    /* Parse 4 target positions
     * 解析 4 顆馬達的目標位置
     */
    for (i = 0; i < JOINT_COUNT_FIXED; i++)
    {
        pos_i = atoi(argv[3 + i]);

        /* Clamp to AX range 0~1023 first
         * 先限制在 AX 官方範圍 0~1023
         */
        if (pos_i < 0)
            pos_i = 0;
        if (pos_i > 1023)
            pos_i = 1023;

        /* Then clamp to joint safety range
         * 再限制在該關節安全範圍
         */
        gJoints[i].target_pos = ClampWord((word)pos_i, gJoints[i].min_pos, gJoints[i].max_pos);
    }

    PrintTargetState(seq, dt_ms, gJoints, JOINT_COUNT_FIXED);

    /* Write positions to all joints
     * 寫入 4 顆馬達目標位置
     */
    if (!MoveAllJoints(gJoints, JOINT_COUNT_FIXED))
    {
        PrintErrorMsg("ERR,DXL_WRITE_FAIL");
        return;
    }

    /* Follow the trajectory timing rhythm
     * 依照 dt_ms 節奏前進
     */
    if (dt_ms > 0)
        mDelay((u32)dt_ms);

    /* Read actual positions once per point
     * 每個軌跡點回讀一次實際位置
     */
    if (!ReadAllJoints(gJoints, JOINT_COUNT_FIXED))
    {
        PrintErrorMsg("ERR,DXL_READ_FAIL");
        return;
    }

    PrintActualState(seq, gJoints, JOINT_COUNT_FIXED);

    gTraj.last_seq = seq;
    gTraj.expected_seq = seq + 1;

    PrintAck1("PT", seq);
}

/*******************************************************************************
* HandleEnd
* Format      : END,<traj_id>
*
* 中文：
* 收到 END 後，進行最後一點的嚴格到位確認
*******************************************************************************/
void HandleEnd(char *argv[], int argc)
{
    int traj_id;

    if (argc != 2)
    {
        PrintErrorMsg("ERR,FIELD_COUNT,END");
        return;
    }

    if (!gTraj.active)
    {
        PrintErrorMsg("ERR,NO_BEGIN");
        return;
    }

    traj_id = atoi(argv[1]);

    if ((byte)traj_id != gTraj.traj_id)
    {
        PrintErrorMsg("ERR,TRAJ_ID");
        return;
    }

    /* Final strict reach check
     * 最終嚴格到位確認
     */
    if (WaitAllJointsReachTarget(gJoints, JOINT_COUNT_FIXED,
                                 FINAL_REACH_TOL, FINAL_REACH_TIMEOUT_MS))
    {
        if (ReadAllJoints(gJoints, JOINT_COUNT_FIXED))
        {
            PrintFinalReached(gJoints, JOINT_COUNT_FIXED);
        }
        PrintEndTag();
    }
    else
    {
        PrintErrorMsg("ERR,FINAL_REACH_TIMEOUT");
        return;
    }

    gTraj.active = 0;
    gTraj.ended = 1;

    PrintAck1("END", traj_id);
}

/*******************************************************************************
* HandleStop
* Description : Stop current trajectory.
*
* 中文：停止目前軌跡
*******************************************************************************/
void HandleStop(void)
{
    gTraj.stop_requested = 1;
    gTraj.active = 0;
    TxDString("OK,STOP\r\n");
}

/*******************************************************************************
* HandleHome
* Description : Move all joints back to home_pos.
*
* 中文：
* 讓 4 顆馬達回到 home_pos
* 注意：home_pos 目前預設 512，實機應再依需求校正
*******************************************************************************/
void HandleHome(void)
{
    int i;

    InitTrajectoryState();

    for (i = 0; i < JOINT_COUNT_FIXED; i++)
    {
        gJoints[i].target_pos = gJoints[i].home_pos;
    }

    PrintTargetState(0, HOME_MOVE_DELAY_MS, gJoints, JOINT_COUNT_FIXED);

    if (!MoveAllJoints(gJoints, JOINT_COUNT_FIXED))
    {
        PrintErrorMsg("ERR,HOME_WRITE_FAIL");
        return;
    }

    mDelay(HOME_MOVE_DELAY_MS);

    if (!ReadAllJoints(gJoints, JOINT_COUNT_FIXED))
    {
        PrintErrorMsg("ERR,HOME_READ_FAIL");
        return;
    }

    PrintActualState(0, gJoints, JOINT_COUNT_FIXED);
    TxDString("OK,HOME\r\n");
}

/*******************************************************************************
* ClampWord
* Description : Clamp value into [min_v, max_v].
*
* 中文：將數值限制在指定範圍內
*******************************************************************************/
word ClampWord(word value, word min_v, word max_v)
{
    if (value < min_v)
        return min_v;
    if (value > max_v)
        return max_v;
    return value;
}

/*******************************************************************************
* EnableTorque
* Description : Enable or disable AX-12A torque.
*
* 中文：開啟或關閉某顆馬達 torque
*******************************************************************************/
byte EnableTorque(byte id, byte enable)
{
    ClearBuffer256();
    dxl_write_byte(id, P_TORQUE_ENABLE, enable);
    CommStatus = dxl_get_result();

    if (CommStatus != COMM_RXSUCCESS)
    {
        PrintCommStatus(CommStatus);
        return 0;
    }

    return 1;
}

/*******************************************************************************
* WriteGoalPosition
* Description : Write Goal Position to one AX-12A.
*
* 中文：對單顆 AX-12A 寫入 Goal Position
*******************************************************************************/
byte WriteGoalPosition(byte id, word pos)
{
    ClearBuffer256();
    dxl_write_word(id, P_GOAL_POSITION_L, pos);
    CommStatus = dxl_get_result();

    if (CommStatus != COMM_RXSUCCESS)
    {
        PrintCommStatus(CommStatus);
        return 0;
    }

    return 1;
}

/*******************************************************************************
* ReadPresentPosition
* Description : Read Present Position from one AX-12A.
*
* 中文：讀回單顆 AX-12A 的 Present Position
*******************************************************************************/
byte ReadPresentPosition(byte id, word *pos)
{
    ClearBuffer256();
    *pos = dxl_read_word(id, P_PRESENT_POSITION_L);
    CommStatus = dxl_get_result();

    if (CommStatus != COMM_RXSUCCESS)
    {
        PrintCommStatus(CommStatus);
        return 0;
    }

    return 1;
}

/*******************************************************************************
* MoveAllJoints
* Description : Write target position to all 4 joints in sequence.
*
* 中文：
* 依序寫入 4 顆馬達目標位置
* 這裡不是 sync write，但對舊 SDK 與除錯較穩定
*******************************************************************************/
byte MoveAllJoints(JointState joints[], int count)
{
    int i;

    for (i = 0; i < count; i++)
    {
        if (!WriteGoalPosition(joints[i].id, joints[i].target_pos))
        {
            return 0;
        }

        mDelay(DXL_WRITE_GAP_MS);
    }

    return 1;
}

/*******************************************************************************
* ReadAllJoints
* Description : Read actual position from all 4 joints.
*
* 中文：依序讀回 4 顆馬達實際位置
*******************************************************************************/
byte ReadAllJoints(JointState joints[], int count)
{
    int i;
    word pos;

    pos = 0;

    for (i = 0; i < count; i++)
    {
        if (!ReadPresentPosition(joints[i].id, &pos))
        {
            return 0;
        }

        joints[i].actual_pos = pos;
        mDelay(FEEDBACK_GAP_MS);
    }

    return 1;
}

/*******************************************************************************
* AreAllJointsReached
* Description : Check whether all joints are within tolerance.
*
* 中文：檢查所有馬達是否都進入容差範圍
*******************************************************************************/
byte AreAllJointsReached(JointState joints[], int count, word tol)
{
    int i;
    int diff;

    for (i = 0; i < count; i++)
    {
        diff = (int)joints[i].target_pos - (int)joints[i].actual_pos;
        if (diff < 0)
            diff = -diff;

        if (diff > (int)tol)
            return 0;
    }

    return 1;
}

/*******************************************************************************
* WaitAllJointsReachTarget
* Description : Wait until all joints reach target or timeout.
*
* 中文：
* 在 timeout 期間內持續輪詢，直到全部到位
* 建議只在 END 最後一點使用，避免中間每點都停住
*******************************************************************************/
byte WaitAllJointsReachTarget(JointState joints[], int count, word tol, u32 timeout_ms)
{
    StartDiscount(timeout_ms);

    while (!CheckTimeOut())
    {
        if (gTraj.stop_requested)
            return 0;

        if (!ReadAllJoints(joints, count))
            return 0;

        if (AreAllJointsReached(joints, count, tol))
            return 1;

        mDelay(20);
    }

    return 0;
}

/*******************************************************************************
* Decimal print helpers
* Description : Simple integer printing helpers without sprintf().
*
* 中文：
* 不使用 sprintf，避免舊工具鏈的浮點 / libc 負擔
*******************************************************************************/
void TxDDecU16(word value)
{
    char buf[6];
    int i = 0;
    int j;

    if (value == 0)
    {
        TxDByte_PC('0');
        return;
    }

    while (value > 0 && i < 5)
    {
        buf[i++] = (char)('0' + (value % 10));
        value /= 10;
    }

    for (j = i - 1; j >= 0; j--)
        TxDByte_PC((byte)buf[j]);
}

void TxDDecS32(int value)
{
    if (value < 0)
    {
        TxDByte_PC('-');
        value = -value;
    }

    TxDDecU16((word)value);
}

/*******************************************************************************
* PrintTargetState
* Description : Print target positions of one trajectory point.
*
* 中文：列印某個 PT 點的目標位置
*******************************************************************************/
void PrintTargetState(int seq, int dt_ms, JointState joints[], int count)
{
    int i;

    TxDString("[TARGET][SEQ=");
    TxDDecS32(seq);
    TxDString("][DT=");
    TxDDecS32(dt_ms);
    TxDString("ms]\r\n");

    for (i = 0; i < count; i++)
    {
        TxDString("ID");
        TxDDecU16(joints[i].id);
        TxDString(" TARGET_POS=");
        TxDDecU16(joints[i].target_pos);
        TxDString("\r\n");
    }
}

/*******************************************************************************
* PrintActualState
* Description : Print actual positions after one trajectory point.
*
* 中文：列印某個 PT 點回讀到的實際位置
*******************************************************************************/
void PrintActualState(int seq, JointState joints[], int count)
{
    int i;

    TxDString("[ACTUAL][SEQ=");
    TxDDecS32(seq);
    TxDString("]\r\n");

    for (i = 0; i < count; i++)
    {
        TxDString("ID");
        TxDDecU16(joints[i].id);
        TxDString(" ACT_POS=");
        TxDDecU16(joints[i].actual_pos);
        TxDString("\r\n");
    }
}

/*******************************************************************************
* PrintFinalReached
* Description : Print final positions after END reach confirmation.
*
* 中文：最終到位後列印各關節最後位置
*******************************************************************************/
void PrintFinalReached(JointState joints[], int count)
{
    int i;

    TxDString("[FINAL_REACHED]\r\n");

    for (i = 0; i < count; i++)
    {
        TxDString("ID");
        TxDDecU16(joints[i].id);
        TxDString(" FINAL_POS=");
        TxDDecU16(joints[i].actual_pos);
        TxDString("\r\n");
    }
}

/*******************************************************************************
* PrintEndTag
* Description : Print END tag after final reach.
*
* 中文：所有馬達到位後列印 END
*******************************************************************************/
void PrintEndTag(void)
{
    TxDString("END\r\n");
}

/*******************************************************************************
* PrintErrorMsg
* Description : Print error message with newline.
*
* 中文：列印錯誤訊息
*******************************************************************************/
void PrintErrorMsg(const char *err)
{
    TxDString(err);
    TxDString("\r\n");
}

/*******************************************************************************
* PrintAck1
* Description : Print OK,<cmd>,<value>
*
* 中文：列印 ACK 格式
*******************************************************************************/
void PrintAck1(const char *cmd, int v1)
{
    TxDString("OK,");
    TxDString(cmd);
    TxDString(",");
    TxDDecS32(v1);
    TxDString("\r\n");
}

/* =============================================================================
 * Original ROBOTIS SDK low-level functions
 * 以下為官方 SDK 底層函式，保留原本結構並加入簡短註解
 * =============================================================================
 */

/*******************************************************************************
* Function Name  : RCC_Configuration
* Description    : Configures system clocks.
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
* Description    : Configures interrupts.
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
* Description    : Configures GPIOs for DXL and PC UART.
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

    GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);   /* TX disable */
    GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);     /* RX enable  */
}

void USART1_Configuration(u32 baudrate)
{
    USART_Configuration(USART_DXL, baudrate);
}

/*******************************************************************************
* Function Name  : USART_Configuration
* Description    : Configure USART1 (DXL) or USART3 (PC).
*******************************************************************************/
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

/*******************************************************************************
* Function Name  : TxDByte_DXL
* Description    : Send one byte to DXL half-duplex bus.
*******************************************************************************/
void TxDByte_DXL(byte bTxdData)
{
    GPIO_ResetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);   /* RX disable */
    GPIO_SetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);     /* TX enable  */

    USART_SendData(USART1, bTxdData);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {
    }

    GPIO_ResetBits(PORT_ENABLE_TXD, PIN_ENABLE_TXD);   /* TX disable */
    GPIO_SetBits(PORT_ENABLE_RXD, PIN_ENABLE_RXD);     /* RX enable  */
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

/*******************************************************************************
* Function Name  : PrintCommStatus
* Description    : Print DXL communication result.
*******************************************************************************/
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
        TxDString("COMM_RXWAITING: Now receiving status packet!\n");
        break;
    case COMM_RXTIMEOUT:
        TxDString("COMM_RXTIMEOUT: There is no status packet!\n");
        break;
    case COMM_RXCORRUPT:
        TxDString("COMM_RXCORRUPT: Incorrect status packet!\n");
        break;
    default:
        TxDString("Unknown communication error code!\n");
        break;
    }
}

/*******************************************************************************
* Function Name  : PrintErrorCode
* Description    : Print AX-12A error bits.
*******************************************************************************/
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

/*******************************************************************************
* Function Name  : Timer_Configuration
* Description    : Configure timer for 1ms counting.
*******************************************************************************/
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

byte CheckTimeOut(void)
{
    if (gw1msCounter == 0)
        return 1;
    else
        return 0;
}