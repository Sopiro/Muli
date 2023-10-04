#include "muli/predefined_block_allocator.h"

namespace muli
{

//                   Sizes
// Circle           : 32
// Capsule          : 56
// Polygon          : 184
// Collider         : 80
// RigidBody        : 192
// Contact          : 616
// Manifold         : 64

// Angle joint      : 160
// Distance joint   : 200
// Grab joint       : 200
// Line joint       : 200
// Motor joint      : 248
// Prismatic joint  : 224
// Pulley joint     : 232
// Revolute joint   : 208
// Weld joint       : 240

// Predefined block sizes
static constexpr int32 block_sizes[PredefinedBlockAllocator::block_size_count] = {
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
    416, // 11
    448, // 12
    512, // 13
    640, // 14
    704, // 15
    // 1024, // 16 for debug
};

static constexpr int32 chunk_size = 16 * 1024;
static constexpr int32 max_block_size = block_sizes[PredefinedBlockAllocator::block_size_count - 1];

struct SizeMap
{
    SizeMap()
    {
        int32 j = 0;
        values[0] = 0;
        for (int32 i = 1; i <= max_block_size; ++i)
        {
            if (i <= block_sizes[j])
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

    int32 values[max_block_size + 1];
};

static const SizeMap size_map;

PredefinedBlockAllocator::PredefinedBlockAllocator()
    : blockCount{ 0 }
    , chunkCount{ 0 }
    , chunks{ nullptr }
{
    memset(freeList, 0, sizeof(freeList));
}

PredefinedBlockAllocator::~PredefinedBlockAllocator()
{
    Clear();
}

void* PredefinedBlockAllocator::Allocate(int32 size)
{
    if (size == 0)
    {
        return nullptr;
    }
    if (size > max_block_size)
    {
        muliAssert(false);
        return muli::Alloc(size);
    }

    int32 index = size_map.values[size];
    muliAssert(0 <= index && index <= block_size_count);

    if (freeList[index] == nullptr)
    {
        Block* blocks = (Block*)muli::Alloc(chunk_size);
        int32 blockSize = block_sizes[index];
        int32 blockCapacity = chunk_size / blockSize;

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

void PredefinedBlockAllocator::Free(void* p, int32 size)
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

    muliAssert(0 < size && size <= max_block_size);

    int32 index = size_map.values[size];
    muliAssert(0 <= index && index <= block_size_count);

#if defined(_DEBUG)
    // Verify the memory address and size is valid.
    int32 blockSize = block_sizes[index];
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
    memset(freeList, 0, sizeof(freeList));
}

} // namespace  muli
