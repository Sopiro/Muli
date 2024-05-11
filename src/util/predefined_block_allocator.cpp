#include "muli/predefined_block_allocator.h"

namespace muli
{

PredefinedBlockAllocator::PredefinedBlockAllocator(int32 initialChunkSize, std::span<int32> blockSizes)
    : sizeMap(std::move(blockSizes))
    , blockCount{ 0 }
    , chunkCount{ 0 }
    , chunkSize{ initialChunkSize }
    , chunks{ nullptr }
{
    freeList = (Block**)muli::Alloc(sizeMap.sizes.size() * sizeof(Block*));
    memset(freeList, 0, sizeMap.sizes.size() * sizeof(Block*));
}

PredefinedBlockAllocator::~PredefinedBlockAllocator()
{
    Clear();
    muli::Free(freeList);
}

void* PredefinedBlockAllocator::Allocate(int32 size)
{
    if (size == 0)
    {
        return nullptr;
    }
    if (size > sizeMap.MaxBlockSize())
    {
        return muli::Alloc(size);
    }

    int32 index = sizeMap.values[size];
    muliAssert(0 <= index && index <= sizeMap.sizes.size());

    if (freeList[index] == nullptr)
    {
        chunkSize += chunkSize / 2;

        Block* blocks = (Block*)muli::Alloc(chunkSize);
        int32 blockSize = sizeMap.sizes[index];
        int32 blockCapacity = chunkSize / blockSize;

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

void PredefinedBlockAllocator::Free(void* p, int32 size)
{
    if (size == 0)
    {
        return;
    }

    if (size > sizeMap.MaxBlockSize())
    {
        muli::Free(p);
        return;
    }

    muliAssert(0 < size && size <= sizeMap.MaxBlockSize());

    int32 index = sizeMap.values[size];
    muliAssert(0 <= index && index <= sizeMap.sizes.size());

#if defined(_DEBUG)
    // Verify the memory address and size is valid.
    int32 blockSize = sizeMap.sizes[index];
    bool found = false;

    Chunk* chunk = chunks;
    while (chunk)
    {
        int32 currentChunkSize = chunk->blockSize * chunk->capacity;
        if (chunk->blockSize != blockSize)
        {
            muliAssert((int8*)p + blockSize <= (int8*)chunk->blocks || (int8*)chunk->blocks + currentChunkSize <= (int8*)p);
        }
        else
        {
            if (((int8*)chunk->blocks <= (int8*)p && (int8*)p + blockSize <= (int8*)chunk->blocks + currentChunkSize))
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

void PredefinedBlockAllocator::Clear()
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
    memset(freeList, 0, sizeMap.sizes.size() * sizeof(Block*));
}

} // namespace  muli
