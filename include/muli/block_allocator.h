#pragma once

#include "allocator.h"

namespace muli
{

class BlockAllocator : public Allocator
{
public:
    static constexpr inline int32 max_block_size = 1024;
    static constexpr inline int32 block_unit = 8;
    static constexpr inline int32 block_size_count = max_block_size / block_unit;

    BlockAllocator(int32 initialChunkSize = 16 * 1024);
    ~BlockAllocator();

    virtual void* Allocate(int32 size) override;
    virtual void Free(void* p, int32 size) override;
    virtual void Clear() override;
    void Clear(int32 initialChunkSize);

    int32 GetBlockCount() const;
    int32 GetChunkCount() const;

    int32 GetChunkSize(int32 size) const;

private:
    int32 blockCount;
    int32 chunkCount;

    int32 chunkSizes[block_size_count];
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
