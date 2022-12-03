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
    muliAssert(index == 0 && entryCount == 0);
}

void* StackAllocator::Allocate(int32 size)
{
    muliAssert(entryCount < maxStackEntries && "Increase the maxStackEntries");

    StackEntry* entry = entries + entryCount;
    entry->size = size;

    if (index + size > stackSize)
    {
        entry->data = (int8*)malloc(size);
        entry->mallocUsed = true;
    }
    else
    {
        entry->data = (int8*)stack + index;
        entry->mallocUsed = false;
        index += size;
    }

    allocation += size;
    maxAllocation = Max(maxAllocation, allocation);

    ++entryCount;

    return entry->data;
}

void StackAllocator::Free(void* p, int32 size)
{
    muliAssert(entryCount > 0);

    StackEntry* entry = entries + entryCount - 1;
    muliAssert(entry->data == p);
    muliAssert(entry->size == size * 8);

    if (entry->mallocUsed)
    {
        free(p);
    }
    else
    {
        index -= entry->size;
    }

    allocation -= entry->size;
    --entryCount;

    p = nullptr;
}

void StackAllocator::Clear()
{
    index = 0;
    allocation = 0;
    maxAllocation = 0;
    entryCount = 0;
}

} // namespace muli