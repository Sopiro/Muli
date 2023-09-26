#pragma once

#include "allocator.h"

namespace muli
{

class BlockAllocator : public Allocator
{
    inline static constexpr int32 max_block_size = 1024;
    inline static constexpr int32 block_unit = 8;
    inline static constexpr int32 block_size_count = max_block_size / block_unit;

public:
    BlockAllocator();
    ~BlockAllocator();

    virtual void* Allocate(int32 size) override;
    virtual void Free(void* p, int32 size) override;
    virtual void Clear() override;

    int32 GetBlockCount() const;
    int32 GetChunkCount() const;

private:
    int32 blockCount;
    int32 chunkCount;

    Chunk* chunks;
    Block* freeList[block_size_count];
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
