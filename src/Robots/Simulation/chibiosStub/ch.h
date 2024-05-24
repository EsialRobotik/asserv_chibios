#ifndef CH_H
#define CH_H

#include <cstdint>
#include <cassert>
#include <chrono>
#include <thread>
#include <ctime>

static auto t_start = std::chrono::high_resolution_clock::now();


typedef uint32_t time_usecs_t;

typedef uint32_t systime_t;


static inline systime_t chVTGetSystemTime(void) {
    const auto t_end = std::chrono::high_resolution_clock::now();
    double delta = std::chrono::duration<double, std::milli>(t_end - t_start).count();

    return (uint32_t)delta;
}

#define TIME_US2I(usecs) (usecs)

#define chDbgAssert(c,r) assert(c && r)


static inline void chSysLock(void) {}
static inline void chSysUnlock(void) {}

static inline void chThdSleepUntil(systime_t ){ std::this_thread::sleep_for(std::chrono::milliseconds(1)); };

#endif /* CHTIME_H */
