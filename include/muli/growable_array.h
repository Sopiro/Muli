#pragma once

#include "allocator.h"

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

    ~GrowableArray()
    {
        if (array != stackArray)
        {
            muli::Free(array);
            array = nullptr;
        }
    }

    GrowableArray(const GrowableArray& other)
    {
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
    }

    GrowableArray& operator=(const GrowableArray& other)
    {
        MuliAssert(this != &other);

        if (array != stackArray)
        {
            muli::Free(array);
        }

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

    GrowableArray(GrowableArray&& other)
    {
        if (other.array == other.stackArray)
        {
            array = stackArray;
            memcpy(stackArray, other.stackArray, other.count * sizeof(T));
        }
        else
        {
            array = other.array;
        }

        capacity = other.capacity;
        count = other.count;

        other.array = other.stackArray;
        other.count = 0;
        other.capacity = N;
    }

    GrowableArray& operator=(GrowableArray&& other)
    {
        MuliAssert(this != &other);

        if (array != stackArray)
        {
            muli::Free(array);
        }

        if (other.array == other.stackArray)
        {
            array = stackArray;
            memcpy(stackArray, other.stackArray, other.count * sizeof(T));
        }
        else
        {
            array = other.array;
        }

        capacity = other.capacity;
        count = other.count;

        other.array = other.stackArray;
        other.count = 0;
        other.capacity = N;

        return *this;
    }

    template <typename... Args>
    T& EmplaceBack(Args&&... args)
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

    void PushBack(const T& data)
    {
        EmplaceBack(data);
    }

    void PushBack(T&& data)
    {
        EmplaceBack(std::move(data));
    }

    T PopBack()
    {
        MuliAssert(count > 0);
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
        MuliAssert(index <= count);

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
