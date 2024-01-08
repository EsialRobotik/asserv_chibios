#include "ch.h"
#include "chibiOsAllocatorWrapper.h"

static bool heapAllocation = true;

void deactivateHeapAllocation()
{
    heapAllocation = false;
}

void* operator new(size_t size)
{
//    chDbgAssert(heapAllocation, "heap allocation disabled during run time...");
    return chHeapAlloc(nullptr, size);
}

void* operator new[](size_t size)
{
//    chDbgAssert(heapAllocation, "heap allocation disabled during run time...");
    return chHeapAlloc(nullptr, size);
}

void* operator new[](size_t size, stkalign_t alignment)
{
//    chDbgAssert(heapAllocation, "heap allocation disabled during run time...");
    return chHeapAllocAligned(nullptr, size, alignment);
}

void* operator new(size_t size, stkalign_t alignment)
{
//    chDbgAssert(heapAllocation, "heap allocation disabled during run time...");
    return chHeapAllocAligned(nullptr, size, alignment);
}

void operator delete(void* ptr) noexcept
{
    chHeapFree(ptr);
}

void operator delete[](void* ptr) noexcept
{
    chHeapFree(ptr);
}

void operator delete(void* ptr, size_t sz)
{
    (void) sz;
    chHeapFree(ptr);
}

void operator delete[](void* ptr, size_t sz)
{
    (void) sz;
    chHeapFree(ptr);
}

/*
 * link time optimizations (LTO) remove this function if they are called only from a lib (ex: libg.a through sscanf)
 * So use  __attribute__((used)) to force the linker to keeps theses functions
 */
//extern "C" void *malloc(unsigned int size) __attribute__((used));
//extern "C" void free(void *alloc) __attribute__((used));
//
//extern "C" void *malloc(unsigned int size)
//{
//    chDbgAssert(heapAllocation, "heap allocation disabled during run time...");
//    return chHeapAlloc(NULL, size);
//}
//
//extern "C" void free(void *alloc)
//{
//    chHeapFree(alloc);
//}

