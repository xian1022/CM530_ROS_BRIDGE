#ifndef CM530_AX12_H
#define CM530_AX12_H

#define AX12_ARMS 2
#define AX12_JOINTS 4
/* No state here: results describe transport, not physical arrival/torque. */
int Ax12WriteGoals(int arm, const unsigned short *positions);
int Ax12WriteTorque(int arm, int enabled);

#endif
