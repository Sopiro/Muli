#include "muli/stack_allocator.h"

namespace muli
{

StackAllocator::StackAllocator()
    : index{ 0 }
    , allocation{ 0 }
    , maxAllocation{ 0 }
    , entryCount{ 0 }
{
}

StackAllocator::~StackAllocator()
{
    MuliAssert(index == 0 && entryCount == 0);
}

void* StackAllocator::Allocate(int32 size)
{
    MuliAssert(entryCount < max_stack_entries && "Increase the maxStackEntries");

    StackEntry* entry = entries + entryCount;
    entry->size = size;

    if (index + size > stack_size)
    {
        entry->data = (int8*)muli::Alloc(size);
        entry->mallocUsed = true;
    }
    else
    {
        entry->data = stack + index;
        entry->mallocUsed = false;
        index += size;
    }

    allocation += size;
    if (allocation > maxAllocation)
    {
        maxAllocation = allocation;
    }

    ++entryCount;

    return entry->data;
}

void StackAllocator::Free(void* p, int32 size)
{
    MuliNotUsed(size);
    MuliAssert(entryCount > 0);

    StackEntry* entry = entries + entryCount - 1;
    MuliAssert(entry->data == p);
    MuliAssert(entry->size == size);

    if (entry->mallocUsed)
    {
        muli::Free(p);
    }
    else
    {
        index -= entry->size;
    }

    allocation -= entry->size;
    --entryCount;
}

void StackAllocator::Clear()
{
    index = 0;
    allocation = 0;
    maxAllocation = 0;
    entryCount = 0;
}

} // namespace muli