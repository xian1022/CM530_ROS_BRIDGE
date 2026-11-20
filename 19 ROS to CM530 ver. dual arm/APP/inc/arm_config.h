#ifndef CM530_ARM_CONFIG_H
#define CM530_ARM_CONFIG_H
/* Raw AX-12A joint positions, j1..j4. Edit here and rebuild after calibration.
 * HOME is a preset, not sensor-based homing or a collision-free path.
 * v18: the values the board answered to GET_HOME on 2026-11-13 (the flashed
 * firmware had been changed from the 90/590,567,712,825 committed in v17). */
static const unsigned short home_A[4] = {256, 512, 512, 512};
static const unsigned short home_C[4] = {256, 512, 512, 512};
/* AX-12A Moving Speed (register 32) for HOME, HOLD and a position-only AX:
 * about 0.111 rpm per unit, so 100 is about 67 deg/s. Never 0, which means
 * maximum speed without speed control. */
#define ARM_DEFAULT_SPEED 100
#endif
