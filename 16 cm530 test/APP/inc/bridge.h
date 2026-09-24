#ifndef CM530_BRIDGE_H
#define CM530_BRIDGE_H

#define BRIDGE_ARMS 2
#define BRIDGE_JOINTS 4
#define BRIDGE_LINE_SIZE 96

/* Feed one byte at a time from USART3. No hardware dependencies here. */
void BridgeInit(void);
void BridgeEnableTorque(void);
void BridgeFeed(unsigned char ch);
/* RX loss: discard the damaged line through its next delimiter. */
void BridgeAbortLine(void);
/* Implemented by the platform adapter (or the host test). */
void BridgeOutput(const char *text);

#endif
