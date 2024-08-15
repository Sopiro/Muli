#include "muli/block_allocator.h"

namespace muli
{

BlockAllocator::BlockAllocator(int32 initialChunkSize)
    : blockCount{ 0 }
    , chunkCount{ 0 }
    , chunks{ nullptr }
{
    memset(freeList, 0, sizeof(freeList));

    for (int32 i = 0; i < block_size_count; ++i)
    {
        chunkSizes[i] = initialChunkSize;
    }
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
    if (size > max_block_size)
    {
        return muli::Alloc(size);
    }

    MuliAssert(0 < size && size <= max_block_size);

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

    MuliAssert(0 <= index && index <= block_size_count);

    if (freeList[index] == nullptr)
    {
        // Increase chunk size by half
        chunkSizes[index] += chunkSizes[index] / 2;

        int32 chunkSize = chunkSizes[index];
        int32 blockCapacity = chunkSize / blockSize;

        Block* blocks = (Block*)muli::Alloc(chunkSize);

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

    MuliAssert(0 <= index && index <= block_size_count);

#if defined(_DEBUG)
    // Verify the memory address and size is valid.
    bool found = false;

    Chunk* chunk = chunks;
    while (chunk)
    {
        int32 chunkSize = chunk->capacity * chunk->blockSize;
        if (chunk->blockSize != blockSize)
        {
            MuliAssert((int8*)p + blockSize <= (int8*)chunk->blocks || (int8*)chunk->blocks + chunkSize <= (int8*)p);
        }
        else
        {
            if (((int8*)chunk->blocks <= (int8*)p && (int8*)p + blockSize <= (int8*)chunk->blocks + chunkSize))
            {
                found = true;
                break;
            }
        }

        chunk = chunk->next;
    }

    MuliAssert(found);
#else
    MuliNotUsed(blockSize);
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

void BlockAllocator::Clear(int32 initialChunkSize)
{
    Clear();

    for (int32 i = 0; i < block_size_count; ++i)
    {
        chunkSizes[i] = initialChunkSize;
    }
}

int32 BlockAllocator::GetChunkSize(int32 size) const
{
    int32 index = size / block_unit;
    if (size % block_unit == 0)
    {
        --index;
    }

    return chunkSizes[index];
}

} // namespace muli
