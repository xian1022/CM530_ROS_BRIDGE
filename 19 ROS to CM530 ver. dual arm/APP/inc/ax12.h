#ifndef CM530_AX12_H
#define CM530_AX12_H

#define AX12_ARMS 2
#define AX12_JOINTS 4
/* No state here: results describe transport, not physical arrival/torque. */
/* Goal Position and Moving Speed (1..1023, never 0) in one SYNC_WRITE. */
int Ax12WriteGoals(int arm, const unsigned short *positions, const unsigned short *speeds);
int Ax12WriteTorque(int arm, int enabled);
/* v19: Compliance Margin and Slope (CW and CCW alike, registers 26-29) in one
 * SYNC_WRITE, then Punch (48-49) in a second. Values are not range checked
 * here; the bridge does that. Success is transport only: read back with
 * Ax12ReadCompliance. These are RAM registers, reset at every power-up. */
int Ax12WriteCompliance(int arm, const unsigned short *margins,
                        const unsigned short *slopes, const unsigned short *punches);

typedef struct {
    const char *code;
    int id;
    int motorError;
} Ax12ReadError;
/* v19: one read of registers 36-41 per motor (Present Position, Speed and
 * Load). Success commits all four positions and loads (raw 0..2047, bit 10
 * set = CW direction). Failure leaves both arrays unchanged. */
int Ax12ReadPositions(int arm, unsigned short *positions, unsigned short *loads,
                      Ax12ReadError *error);
/* v19: registers 26-49 per motor. Fails with DXL_ASYM when CW and CCW differ. */
int Ax12ReadCompliance(int arm, unsigned short *margins, unsigned short *slopes,
                       unsigned short *punches, Ax12ReadError *error);
/* v19: Present Voltage (0.1 V) and Temperature (deg C), registers 42-43. */
int Ax12ReadDiag(int arm, unsigned short *voltages, unsigned short *temperatures,
                 Ax12ReadError *error);

#endif
