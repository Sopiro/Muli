#pragma once

#include "common.h"

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

constexpr int32 maxBlockSize = 1024;
constexpr int32 blockUnit = 8;
constexpr int32 blockSizeCount = maxBlockSize / blockUnit;

class BlockAllocator
{
public:
    BlockAllocator();
    ~BlockAllocator();

    void* Allocate(int32 size);
    void Free(void* p, int32 size);
    void Clear();

    int32 GetBlockCount() const;
    int32 GetChunkCount() const;

private:
    int32 blockCount;
    int32 chunkCount;

    Chunk* chunks;
    Block* freeList[blockSizeCount];
};

inline int32 BlockAllocator::GetBlockCount() const
{
    return blockCount;
}

inline int32 BlockAllocator::GetChunkCount() const
{
    return chunkCount;
}

} // namespace muli
