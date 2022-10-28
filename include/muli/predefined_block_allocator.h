#pragma once

#include "common.h"

namespace muli
{

struct Chunk;
struct Block;

constexpr int32 predefinedBlockSizeCount = 16;

class PredefinedBlockAllocator
{
public:
    PredefinedBlockAllocator();
    ~PredefinedBlockAllocator();

    void* Allocate(int32 size);
    void Free(void* p, int32 size);
    void Clear();

    int32 GetBlockCount() const;
    int32 GetChunkCount() const;

private:
    int32 blockCount;
    int32 chunkCount;

    Chunk* chunks;
    Block* freeList[predefinedBlockSizeCount];
};

inline int32 PredefinedBlockAllocator::GetBlockCount() const
{
    return blockCount;
}

inline int32 PredefinedBlockAllocator::GetChunkCount() const
{
    return chunkCount;
}

} // namespace muli