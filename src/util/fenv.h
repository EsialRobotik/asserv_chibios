/*
 * Minimal fenv.h stub for ChibiOS/newlib (no FP exception hardware).
 * The FP exception functions are no-ops on Cortex-M.
 */

#ifndef _FENV_H_STUB
#define _FENV_H_STUB

#define FE_INVALID    0x01
#define FE_DIVBYZERO  0x04
#define FE_OVERFLOW   0x08
#define FE_UNDERFLOW  0x10

static inline int feclearexcept(int excepts) { (void)excepts; return 0; }
static inline int fetestexcept(int excepts)  { (void)excepts; return 0; }

#endif
