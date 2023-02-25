#include "muli/block_allocator.h"

namespace muli
{

static constexpr int32 chunk_size = 16 * 1024;

BlockAllocator::BlockAllocator()
    : blockCount{ 0 }
    , chunkCount{ 0 }
    , chunks{ nullptr }
{
    memset(freeList, 0, sizeof(freeList));
}

BlockAllocator::~BlockAllocator()
{
    Clear();
}

void* BlockAllocator::Allocate(int32 size)
{
    if (size == 0)
    {
        return nullptr;
    }

#if 0
    muliAssert(size <= maxBlockSize);
#else
    if (size > max_block_size)
    {
        return muli::Alloc(size);
    }
#endif

    muliAssert(0 < size && size <= max_block_size);

    int32 blockSize = size;
    int32 index = size / block_unit;
    int32 mod = size % block_unit;
    if (mod != 0)
    {
        blockSize += block_unit - mod;
    }
    else
    {
        --index;
    }
    int32 blockCapacity = chunk_size / blockSize;

    muliAssert(0 <= index && index <= block_size_count);

    if (freeList[index] == nullptr)
    {
        Block* blocks = (Block*)muli::Alloc(chunk_size);

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
        newChunk->blockSize = blockSize;
        newChunk->blocks = blocks;
        newChunk->next = chunks;
        chunks = newChunk;
        ++chunkCount;

        freeList[index] = newChunk->blocks;
    }

    Block* block = freeList[index];
    freeList[index] = block->next;
    ++blockCount;

    return block;
}

void BlockAllocator::Free(void* p, int32 size)
{
    if (size == 0)
    {
        return;
    }

    if (size > max_block_size)
    {
        muli::Free(p);
        return;
    }

    int32 blockSize = size;
    int32 index = size / block_unit;
    int32 mod = size % block_unit;
    if (mod != 0)
    {
        blockSize += block_unit - mod;
    }
    else
    {
        --index;
    }

    muliAssert(0 <= index && index <= block_size_count);

#if defined(_DEBUG)
    // Verify the memory address and size is valid.
    bool found = false;

    Chunk* chunk = chunks;
    while (chunk)
    {
        if (chunk->blockSize != blockSize)
        {
            muliAssert((int8*)p + blockSize <= (int8*)chunk->blocks || (int8*)chunk->blocks + chunk_size <= (int8*)p);
        }
        else
        {
            if (((int8*)chunk->blocks <= (int8*)p && (int8*)p + blockSize <= (int8*)chunk->blocks + chunk_size))
            {
                found = true;
                break;
            }
        }

        chunk = chunk->next;
    }

    muliAssert(found);
#endif

    Block* block = (Block*)p;
    block->next = freeList[index];
    freeList[index] = block;
    --blockCount;
}

void BlockAllocator::Clear()
{
    Chunk* chunk = chunks;
    while (chunk)
    {
        Chunk* c0 = chunk;
        chunk = c0->next;
        muli::Free(c0->blocks);
        muli::Free(c0);
    }

    blockCount = 0;
    chunkCount = 0;
    chunks = nullptr;
    memset(freeList, 0, sizeof(freeList));
}

} // namespace muli
