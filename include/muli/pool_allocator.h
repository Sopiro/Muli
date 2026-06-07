#pragma once

#include "allocator.h"

namespace muli
{

class PoolAllocator
{
public:
    using PoolId = int32;

    PoolAllocator(int32 defaultChunkByteSize = 16 * 1024);
    ~PoolAllocator();

    void Clear();

    template <typename T>
    PoolId Register(int32 chunkCapacity = 0);

    template <typename T>
    T* Allocate();

    template <typename T>
    void Free(T* p);

    template <typename T, typename... Args>
    T* New(Args&&... args);

    template <typename T>
    void Delete(T* p);

    int32 GetPoolCount() const;

private:
    struct Pool
    {
        int32 stride;
        int32 initialChunkCapacity;
        int32 chunkCapacity;
        int32 allocationCount;
        Chunk* chunks;
        Block* freeList;
    };

    struct TypePoolEntry
    {
        const void* key;
        PoolId poolId;
    };

    template <typename T>
    PoolId GetPool();

    PoolId CreatePool(int32 elementSize, int32 chunkCapacity = 0, int32 alignment = alignof(std::max_align_t));

    void* AllocateFromPool(PoolId poolId);
    void FreeFromPool(PoolId poolId, void* p);

    PoolId GetPool(int32 elementSize, int32 alignment, const void* key, int32 chunkCapacity);
    void GrowPool(PoolId poolId);

    int32 defaultChunkByteSize;

    std::vector<Pool> pools;
    std::vector<TypePoolEntry> typePools;
};

inline int32 PoolAllocator::GetPoolCount() const
{
    return int32(pools.size());
}

template <typename T>
inline PoolAllocator::PoolId PoolAllocator::Register(int32 chunkCapacity)
{
    static int32 typeKey;
    return GetPool(sizeof(T), alignof(T), &typeKey, chunkCapacity);
}

template <typename T>
inline T* PoolAllocator::Allocate()
{
    return (T*)AllocateFromPool(GetPool<T>());
}

template <typename T>
inline void PoolAllocator::Free(T* p)
{
    FreeFromPool(GetPool<T>(), p);
}

template <typename T, typename... Args>
inline T* PoolAllocator::New(Args&&... args)
{
    return new (Allocate<T>()) T(std::forward<Args>(args)...);
}

template <typename T>
inline void PoolAllocator::Delete(T* p)
{
    if (p == nullptr)
    {
        return;
    }

    p->~T();
    Free(p);
}

template <typename T>
inline PoolAllocator::PoolId PoolAllocator::GetPool()
{
    return Register<T>();
}

} // namespace muli
