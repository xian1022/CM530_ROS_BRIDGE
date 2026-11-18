#ifndef CM530_ARM_LED_H
#define CM530_ARM_LED_H

/* Call after RCC/GPIO setup, before BridgeInit. POWER is left untouched. */
void ArmLedInit(void);
/* arm: 0 or 1; moving: 0 = red standby, 1 = green host-reported moving. */
void ArmLedSet(int arm, int moving);

#endif
