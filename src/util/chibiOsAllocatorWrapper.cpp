#include "ch.h"
#include "chibiOsAllocatorWrapper.h"

static bool heapAllocation = true;

void deactivateHeapAllocation()
{
	heapAllocation = false;
}

void* operator new(size_t size)
{
    chDbgAssert(heapAllocation, "heap allocation disabled during run time...");
    return chHeapAlloc(nullptr, size);
}

void* operator new[](size_t size)
{
    chDbgAssert(heapAllocation, "heap allocation disabled during run time...");
    return chHeapAlloc(nullptr, size);
}

void* operator new[](size_t size, stkalign_t alignment)
{
    chDbgAssert(heapAllocation, "heap allocation disabled during run time...");
    return chHeapAllocAligned(nullptr, size, alignment);
}

void* operator new(size_t size, stkalign_t alignment)
{
    chDbgAssert(heapAllocation, "heap allocation disabled during run time...");
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
    (void)sz;
    chHeapFree(ptr);
}

void operator delete[](void* ptr, size_t sz)
{
    (void)sz;
    chHeapFree(ptr);
}


extern "C" void *malloc(int size)
{
    chDbgAssert(heapAllocation, "heap allocation disabled during run time...");
    return chHeapAlloc(NULL, size);
}

extern "C" void free(void *alloc)
{
    chHeapFree(alloc);
}



