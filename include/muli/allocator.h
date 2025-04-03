#pragma once

#include "common.h"
#include "types.h"

namespace muli
{

// default alloc/dealloc funcitons

inline void* Alloc(int32 size)
{
    return std::malloc(size);
}

inline void Free(void* mem)
{
    std::free(mem);
}

struct Block
{
    Block* next;
};

struct Chunk
{
    int32 capacity;
    int32 blockSize;
    Block* blocks;
    Chunk* next;
};

class Allocator
{
public:
    Allocator() = default;
    virtual ~Allocator() = default;

    virtual void* Allocate(int32 size) = 0;
    virtual void Free(void* p, int32 size) = 0;
    virtual void Clear() = 0;
};

} // namespace muli
