#include "muli/block_allocator.h"

namespace muli
{

static constexpr int32 chunkSize = 16 * 1024;

BlockAllocator::BlockAllocator()
{
    blockCount = 0;
    chunkCount = 0;
    chunks = nullptr;
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
    if (size > maxBlockSize)
    {
        return malloc(size);
    }

    assert(0 < size && size <= maxBlockSize);

    int32 blockSize = size;
    int32 index = size / blockUnit;
    int32 mod = size % blockUnit;
    if (mod != 0)
    {
        blockSize += blockUnit - mod;
    }
    else
    {
        --index;
    }
    int32 blockCapacity = chunkSize / blockSize;

    assert(0 <= index && index <= blockSizeCount);

    if (freeList[index] == nullptr)
    {
        Block* blocks = (Block*)malloc(chunkSize);

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

    if (size > maxBlockSize)
    {
        free(p);
        return;
    }

    int32 blockSize = size;
    int32 index = size / blockUnit;
    int32 mod = size % blockUnit;
    if (mod != 0)
    {
        blockSize += blockUnit - mod;
    }
    else
    {
        --index;
    }

    assert(0 <= index && index <= blockSizeCount);

#if defined(_DEBUG)
    // Verify the memory address and size is valid.
    bool found = false;

    Chunk* chunk = chunks;
    while (chunk)
    {
        if (chunk->blockSize != blockSize)
        {
            assert((int8*)p + blockSize <= (int8*)chunk->blocks || (int8*)chunk->blocks + chunkSize <= (int8*)p);
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

    assert(found);
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
        free(c0->blocks);
        free(c0);
    }

    blockCount = 0;
    chunkCount = 0;
    chunks = nullptr;
    memset(freeList, 0, sizeof(freeList));
}

} // namespace muli
