#pragma once

#include "allocator.h"
#include "common.h"

namespace muli
{

template <int32 blockSize>
class FixedBlockAllocator : public Allocator
{
public:
    FixedBlockAllocator(int32 initialBlockCapacity = 64)
        : blockCapacity{ initialBlockCapacity }
        , blockCount{ 0 }
        , chunkCount{ 0 }
        , chunks{ nullptr }
        , freeList{ nullptr }
    {
    }

    ~FixedBlockAllocator()
    {
        Clear();
    }

    void* Allocate(int32 size = blockSize) override
    {
        muliAssert(size == blockSize);

        if (size > blockSize)
        {
            return muli::Alloc(size);
        }

        if (freeList == nullptr)
        {
            muliAssert(blockCount == 0 || blockCapacity == blockCount / chunkCount);

            blockCapacity += blockCapacity / 2;
            Block* blocks = (Block*)muli::Alloc(blockCapacity * blockSize);
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

            Chunk* newChunk = (Chunk*)muli::Alloc(sizeof(Chunk));
            newChunk->capacity = blockCapacity;
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

    void Free(void* p, int32 size = blockSize) override
    {
        muliAssert(0 < blockCount && 0 < chunkCount);

        if (size > blockSize)
        {
            muli::Free(p);
            return;
        }

#if defined(_DEBUG)
        // Verify the memory address and size is valid.
        bool found = false;

        Chunk* chunk = chunks;
        while (chunk)
        {
            if ((int8*)chunk->blocks <= (int8*)p && (int8*)p + blockSize <= (int8*)chunk->blocks + blockSize * chunk->capacity)
            {
                found = true;
                break;
            }
            chunk = chunk->next;
        }

        muliAssert(found);
#endif

        Block* block = (Block*)p;
        block->next = freeList;
        freeList = block;
        --blockCount;
    }

    void Clear() override
    {
        Chunk* chunk = chunks;
        while (chunk)
        {
            Chunk* c0 = chunk;
            chunk = c0->next;
            muli::Free(c0->blocks);
            muli::Free(c0);
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
    int32 blockCapacity;
    int32 chunkCount;
    int32 blockCount;
    Chunk* chunks;
    Block* freeList;
};

} // namespace muli
