#pragma once

#include "common.h"

namespace muli
{

constexpr int32 stackSize = 100 * 1024;
constexpr int32 maxStackEntries = 32;

struct StackEntry
{
    int8* data;
    int32 size;
    bool mallocUsed;
};

// Stack allocator used for transient, predictable allocations.
// You muse nest allocate/free pairs
class StackAllocator
{
public:
    StackAllocator();
    ~StackAllocator();

    void* Allocate(int32 size);
    void Free(void* p);
    void Clear();

    int32 GetAllocation() const;
    int32 GetMaxAllocation() const;

private:
    int8* stack[stackSize];
    int32 index;

    int32 allocation;
    int32 maxAllocation;

    StackEntry entries[maxStackEntries];
    int32 entryCount;
};

inline int32 StackAllocator::GetAllocation() const
{
    return allocation;
}

inline int32 StackAllocator::GetMaxAllocation() const
{
    return maxAllocation;
}

} // namespace muli
