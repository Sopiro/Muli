#pragma once

#include "common.h"
#include "util.h"

namespace muli
{

struct Block
{
    Block* next;
};

struct Chunk
{
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
