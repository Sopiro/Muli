#pragma once

#include "allocator.h"

namespace muli
{

class PredefinedBlockAllocator : public Allocator
{
    static inline int32 default_block_sizes[14] = {
        16,  // 0
        32,  // 1
        64,  // 2
        96,  // 3
        128, // 4
        160, // 5
        192, // 6
        224, // 7
        256, // 8
        320, // 9
        384, // 10
        448, // 11
        512, // 12
        640, // 13
    };

public:
    PredefinedBlockAllocator(int32 initialChunkSize = 16 * 1024, std::span<int32> blockSizes = default_block_sizes);
    ~PredefinedBlockAllocator();

    virtual void* Allocate(int32 size) override;
    virtual void Free(void* p, int32 size) override;
    virtual void Clear() override;

    int32 GetBlockCount() const;
    int32 GetChunkCount() const;

    int32 GetBlockSizeCount() const;

private:
    struct SizeMap
    {
        SizeMap(std::span<int32> blockSizes)
            : sizes(blockSizes.begin(), blockSizes.end())
            , values(blockSizes.back() + 1)
        {
            int32 j = 0;
            values[0] = 0;
            int32 maxBlockSize = sizes.back();
            for (int32 i = 1; i <= maxBlockSize; ++i)
            {
                if (i <= blockSizes[j])
                {
                    values[i] = j;
                }
                else
                {
                    ++j;
                    values[i] = j;
                }
            }
        }

        int32 MaxBlockSize() const
        {
            return sizes.back();
        }

        std::vector<int32> sizes, values;
    };

    SizeMap sizeMap;

    int32 blockCount;
    int32 chunkCount;

    int32 chunkSize;
    Chunk* chunks;
    Block** freeList;
};

inline int32 PredefinedBlockAllocator::GetBlockCount() const
{
    return blockCount;
}

inline int32 PredefinedBlockAllocator::GetChunkCount() const
{
    return chunkCount;
}

inline int32 PredefinedBlockAllocator::GetBlockSizeCount() const
{
    return sizeMap.sizes.size();
}

} // namespace muli