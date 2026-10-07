#ifndef CM530_BRIDGE_H
#define CM530_BRIDGE_H

#include "ax12.h"
#define BRIDGE_ARMS AX12_ARMS
#define BRIDGE_JOINTS AX12_JOINTS
#define BRIDGE_LINE_SIZE 96

/* Feed one byte at a time from USART3. No hardware dependencies here. */
/* Call after LED/DXL/UART/timer init: red standby, torque-off, then READY,5
 * or ERR,INIT_FAILED. Failure latches motion rejection until restart. */
void BridgeInit(void);
void BridgeFeed(unsigned char ch);
/* RX loss: discard the damaged line through its next delimiter. */
void BridgeAbortLine(void);
/* Implemented by the platform adapter (or the host test). */
void BridgeOutput(const char *text);
void BridgeSetArmLed(int arm, int moving);

#endif
