#ifndef __EXCEPTION_VECTORS_H__
#define __EXCEPTION_VECTORS_H__

#if !defined(__ASSEMBLER__)

#if defined(__cplusplus)

extern "C" {
#endif

/**
 * Helper function for handling the assertion.
 */
extern void dbg_assert(const char* assertion, const char* file, unsigned line, const char* func, const char* reason);

#if defined(__cplusplus)
}
#endif // __cplusplus

#define chDbgCheck(c) chDbgAssert(c, "invalid parameter")

#define chDbgAssert(c, r)                                        \
    do {                                                         \
        if (CH_DBG_ENABLE_ASSERTS != FALSE) {                    \
            if (!(c)) {                                          \
                dbg_assert(#c, __FILE__, __LINE__, __func__, r); \
            }                                                    \
        }                                                        \
    } while (false)

#endif

#endif // __EXCEPTION_VECTORS_H__
