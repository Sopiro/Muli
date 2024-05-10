#pragma once

#include "allocator.h"

namespace muli
{

// You muse nest allocate/free pairs
class LinearAllocator : public Allocator
{
public:
    LinearAllocator(int32 initialCapacity = 10 * 1024);
    ~LinearAllocator();

    virtual void* Allocate(int32 size) override;
    virtual void Free(void* p, int32 size) override;
    virtual void Clear() override;

    bool GrowMemory();

    int32 GetCapacity() const;
    int32 GetAllocation() const;
    int32 GetMaxAllocation() const;

private:
    struct MemoryEntry
    {
        int8* data;
        int32 size;
        bool mallocUsed;
    };

    MemoryEntry* entries;
    int32 entryCount;
    int32 entryCapacity;

    int8* mem;
    int32 capacity;
    int32 index;

    int32 allocation;
    int32 maxAllocation;
};

inline int32 LinearAllocator::GetCapacity() const
{
    return capacity;
}

inline int32 LinearAllocator::GetAllocation() const
{
    return allocation;
}

inline int32 LinearAllocator::GetMaxAllocation() const
{
    return maxAllocation;
}

} // namespace muli
