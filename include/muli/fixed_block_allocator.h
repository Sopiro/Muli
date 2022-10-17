#pragma once

#include "common.h"

namespace muli
{

template <int32 blockSize, int32 blockCapacity = 32>
class FixedBlockAllocator
{
public:
    FixedBlockAllocator()
    {
        blockCount = 0;
        chunkCount = 0;
        chunks = nullptr;
        freeList = nullptr;
    }

    ~FixedBlockAllocator()
    {
        Clear();
    }

    void* Allocate()
    {
        if (freeList == nullptr)
        {
            assert(blockCount == 0 || blockCapacity == blockCount / chunkCount);

            Block* blocks = (Block*)malloc(blockCapacity * blockSize);
            memset(blocks, 0, blockCapacity * blockSize);

            // Build a linked list for the free list.
            for (int32 i = 0; i < blockCapacity - 1; ++i)
            {
                Block* block = (Block*)((int8*)blocks + blockSize * i);
                Block* next = (Block*)((int8*)blocks + blockSize * (i + 1));
                block->next = next;
            }
            Block* last = (Block*)((int8*)blocks + blockSize * (blockCapacity - 1));
            last->next = nullptr;

            Chunk* newChunk = (Chunk*)malloc(sizeof(Chunk));
            newChunk->blockSize = blockSize;
            newChunk->blocks = blocks;
            newChunk->next = chunks;
            chunks = newChunk;
            ++chunkCount;

            freeList = newChunk->blocks;
        }

        void* block = freeList;
        freeList = freeList->next;
        ++blockCount;

        return block;
    }

    void Free(void* p)
    {
        assert(0 < blockCount && 0 < chunkCount);

#if defined(_DEBUG)
        // Verify the memory address and size is valid.
        bool found = false;

        Chunk* chunk = chunks;
        while (chunk)
        {
            if ((int8*)chunk->blocks <= (int8*)p && (int8*)p + blockSize <= (int8*)chunk->blocks + blockSize * blockCapacity)
            {
                found = true;
                break;
            }
            chunk = chunk->next;
        }

        assert(found);
#endif

        Block* block = (Block*)p;
        block->next = freeList;
        freeList = block;
        --blockCount;
    }

    void Clear()
    {
        Chunk* chunk = chunks;
        while (chunk)
        {
            Chunk* c0 = chunk;
            chunk = c0->next;
            free(c0->blocks);
            free(c0);
        }

        chunks = nullptr;
        freeList = nullptr;
    }

    int32 GetChunkCount() const
    {
        return chunkCount;
    }

    int32 GetBlockCount() const
    {
        return blockCount;
    }

private:
    int32 chunkCount;
    int32 blockCount;
    Chunk* chunks;
    Block* freeList;
};

} // namespace muli
