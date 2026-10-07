#ifndef CM530_ARM_CONFIG_H
#define CM530_ARM_CONFIG_H
/* Raw AX-12A joint positions, j1..j4. Edit here and rebuild after calibration.
 * HOME is a preset, not sensor-based homing or a collision-free path. */
static const unsigned short home_A[4] = {512, 512, 512, 512};
static const unsigned short home_C[4] = {512, 512, 512, 512};
#endif
