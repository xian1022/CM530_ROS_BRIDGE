#ifndef CM530_ARM_CONFIG_H
#define CM530_ARM_CONFIG_H
/* Provisional, uncalibrated AX-12A observation poses for arm1/A and arm2/C.
 * Assume j1..j4 are ordered bottom to top; calibrate each arm separately.
 * Camera direction is not guaranteed. Edit here and rebuild after calibration.
 * HOME is a preset, not sensor-based homing or a collision-free path. */
static const unsigned short home_A[4] = {256, 512, 512, 512};
static const unsigned short home_C[4] = {256, 512, 512, 512};
#endif
