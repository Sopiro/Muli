#include "muli/pool_allocator.h"
#include "muli/math.h"

namespace muli
{

PoolAllocator::PoolAllocator(int32 defaultChunkByteSize)
    : defaultChunkByteSize{ defaultChunkByteSize }
{
}

PoolAllocator::~PoolAllocator()
{
    Clear();
}

void PoolAllocator::Clear()
{
    for (Pool& pool : pools)
    {
        Chunk* chunk = pool.chunks;
        while (chunk)
        {
            Chunk* c0 = chunk;
            chunk = c0->next;
            muli::Free(c0->blocks);
            muli::Free(c0);
        }

        pool.chunks = nullptr;
        pool.freeList = nullptr;
        pool.allocationCount = 0;
        pool.chunkCapacity = pool.initialChunkCapacity;
    }
}

PoolAllocator::PoolId PoolAllocator::CreatePool(int32 elementSize, int32 chunkCapacity, int32 alignment)
{
    MuliAssert(elementSize > 0);
    MuliAssert(alignment > 0);
    MuliAssert((alignment & (alignment - 1)) == 0);
    MuliAssert(alignment <= int32(alignof(std::max_align_t)));

    int32 stride = Max(elementSize, int32(sizeof(Block)));
    int32 blockAlignment = Max(alignment, int32(alignof(Block)));
    int32 mask = blockAlignment - 1;
    stride = (stride + mask) & ~mask;

    if (chunkCapacity <= 0)
    {
        chunkCapacity = Max(defaultChunkByteSize / stride, 1);
    }

    Pool pool{};
    pool.stride = stride;
    pool.initialChunkCapacity = chunkCapacity;
    pool.chunkCapacity = chunkCapacity;

    PoolId poolId = int32(pools.size());
    pools.push_back(pool);

    return poolId;
}

void* PoolAllocator::AllocateFromPool(PoolId poolId)
{
    MuliAssert(0 <= poolId && poolId < int32(pools.size()));

    Pool& pool = pools[poolId];
    if (pool.freeList == nullptr)
    {
        GrowPool(poolId);
    }

    Block* block = pool.freeList;
    pool.freeList = block->next;
    ++pool.allocationCount;

    return block;
}

void PoolAllocator::FreeFromPool(PoolId poolId, void* p)
{
    if (p == nullptr)
    {
        return;
    }

    MuliAssert(0 <= poolId && poolId < int32(pools.size()));

    Pool& pool = pools[poolId];
    MuliAssert(pool.allocationCount > 0);

    Block* block = (Block*)p;
    block->next = pool.freeList;
    pool.freeList = block;
    --pool.allocationCount;
}

PoolAllocator::PoolId PoolAllocator::GetPool(int32 elementSize, int32 alignment, const void* key, int32 chunkCapacity)
{
    for (TypePoolEntry entry : typePools)
    {
        if (entry.key == key)
        {
            if (chunkCapacity > 0)
            {
                MuliAssert(pools[entry.poolId].initialChunkCapacity == chunkCapacity);
            }
            return entry.poolId;
        }
    }

    PoolId poolId = CreatePool(elementSize, chunkCapacity, alignment);
    typePools.push_back(TypePoolEntry{ key, poolId });
    return poolId;
}

void PoolAllocator::GrowPool(PoolId poolId)
{
    Pool& pool = pools[poolId];
    int32 chunkCapacity = pool.chunkCapacity;

    Block* blocks = (Block*)muli::Alloc(chunkCapacity * pool.stride);
    memset(blocks, 0, chunkCapacity * pool.stride);

    for (int32 i = 0; i < chunkCapacity - 1; ++i)
    {
        Block* block = (Block*)((int8*)blocks + pool.stride * i);
        Block* next = (Block*)((int8*)blocks + pool.stride * (i + 1));
        block->next = next;
    }

    Block* last = (Block*)((int8*)blocks + pool.stride * (chunkCapacity - 1));
    last->next = nullptr;

    Chunk* chunk = (Chunk*)muli::Alloc(sizeof(Chunk));
    chunk->capacity = chunkCapacity;
    chunk->blockSize = pool.stride;
    chunk->blocks = blocks;
    chunk->next = pool.chunks;
    pool.chunks = chunk;

    pool.freeList = blocks;
    pool.chunkCapacity += std::max(pool.chunkCapacity / 2, 1);
}

} // namespace muli
