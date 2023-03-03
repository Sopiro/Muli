#pragma once

#include "common.h"
#include "util.h"

namespace muli
{

// Inspired by b2GrowableStack in box2d code
template <typename T, int32 N>
class GrowableArray
{
public:
    GrowableArray()
        : array{ stackArray }
        , count{ 0 }
        , capacity{ N }
    {
    }

    ~GrowableArray() noexcept
    {
        if (array != stackArray)
        {
            muli::Free(array);
            array = nullptr;
        }
    }

    GrowableArray(const GrowableArray& other) noexcept
    {
        operator=(other);
    }

    GrowableArray& operator=(const GrowableArray& other) noexcept
    {
        muliAssert(this != &other);
        if (other.array == other.stackArray)
        {
            array = stackArray;
            memcpy(stackArray, other.stackArray, other.count * sizeof(T));
        }
        else
        {
            array = (T*)muli::Alloc(other.capacity * sizeof(T));
            memcpy(array, other.array, other.count * sizeof(T));
        }

        capacity = other.capacity;
        count = other.count;

        return *this;
    }

    GrowableArray(GrowableArray&& other) noexcept
    {
        operator=(std::move(other));
    }

    GrowableArray& operator=(GrowableArray&& other) noexcept
    {
        muliAssert(this != &other);
        if (other.array == other.stackArray)
        {
            array = stackArray;
            memcpy(stackArray, other.stackArray, other.count * sizeof(T));
        }
        else
        {
            array = other.array;
            other.array = other.stackArray;
            other.count = 0;
            other.capacity = N;
        }

        capacity = other.capacity;
        count = other.count;

        return *this;
    }

    template <typename... Args>
    T& Emplace(Args&&... args)
    {
        if (count == capacity)
        {
            T* old = array;
            capacity *= 2;

            array = (T*)muli::Alloc(capacity * sizeof(T));
            memcpy(array, old, count * sizeof(T));

            if (old != stackArray)
            {
                muli::Free(old);
            }
        }

        return *new (array + count++) T{ std::forward<Args>(args)... };
    }

    void Push(const T& data)
    {
        Emplace(data);
    }

    void Push(T&& data)
    {
        Emplace(std::move(data));
    }

    T Pop()
    {
        muliAssert(count > 0);
        --count;
        return array[count];
    }

    T& Back() const
    {
        return array[count - 1];
    }

    // O(n)
    void Insert(int32 index, const T& data)
    {
        muliAssert(index <= count);

        if (count == capacity)
        {
            T* old = array;
            capacity *= 2;

            array = (T*)muli::Alloc(capacity * sizeof(T));
            memcpy(array, old, count * sizeof(T));

            if (old != stackArray)
            {
                muli::Free(old);
            }
        }

        int32 ptr = count;
        while (index != ptr)
        {
            array[ptr] = array[ptr - 1];
            --ptr;
        }

        array[index] = data;
        ++count;
    }

    // O(n)
    void Remove(int32 index)
    {
        int32 ptr = index;
        while (ptr != count)
        {
            array[ptr] = array[ptr + 1];
            ++ptr;
        }

        --count;
    }

    // O(1)
    void RemoveSwap(int32 index)
    {
        array[index] = array[count - 1];
        --count;
    }

    int32 Count() const
    {
        return count;
    }

    void Clear()
    {
        count = 0;
    }

    void Reset()
    {
        if (array != stackArray)
        {
            muli::Free(array);
        }
        array = stackArray;
        count = 0;
    }

    int32 Capacity() const
    {
        return capacity;
    }

    T& At(int32 index) const
    {
        return array[index];
    }

    T& operator[](int32 index) const
    {
        return array[index];
    }

private:
    T* array;
    T stackArray[N];
    int32 count;
    int32 capacity;
};

} // namespace muli
