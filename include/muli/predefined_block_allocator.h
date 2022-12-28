#pragma once

#include "allocator.h"

namespace muli
{

struct Chunk;
struct Block;

constexpr int32 predefined_block_size_count = 16;

class PredefinedBlockAllocator : public Allocator
{
public:
    PredefinedBlockAllocator();
    ~PredefinedBlockAllocator();

    virtual void* Allocate(int32 size) override;
    virtual void Free(void* p, int32 size) override;
    virtual void Clear() override;

    int32 GetBlockCount() const;
    int32 GetChunkCount() const;

private:
    int32 blockCount;
    int32 chunkCount;

    Chunk* chunks;
    Block* freeList[predefined_block_size_count];
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