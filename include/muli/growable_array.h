#pragma once

#include "common.h"

namespace muli
{

// Inspired by b2GrowableStack in box2d code
template <typename T, uint32 N>
class GrowableArray
{
public:
    GrowableArray()
    {
        array = stack_array;
        count = 0;
        capacity = N;
    }

    ~GrowableArray()
    {
        if (array != stack_array)
        {
            free(array);
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
        if (other.array == other.stack_array)
        {
            array = stack_array;
            memcpy(stack_array, other.stack_array, other.count * sizeof(T));
        }
        else
        {
            array = (T*)malloc(other.capacity * sizeof(T));
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
        if (other.array == other.stack_array)
        {
            array = stack_array;
            memcpy(stack_array, other.stack_array, other.count * sizeof(T));
        }
        else
        {
            array = other.array;
            other.array = other.stack_array;
            other.count = 0;
            other.capacity = N;
        }

        capacity = other.capacity;
        count = other.count;

        return *this;
    }

    void Push(const T& data)
    {
        if (count == capacity)
        {
            T* old = array;
            capacity *= 2;

            array = (T*)malloc(capacity * sizeof(T));
            memcpy(array, old, count * sizeof(T));

            if (old != stack_array)
            {
                free(old);
            }
        }

        array[count] = data;
        ++count;
    }

    T Pop()
    {
        assert(count > 0);
        --count;
        return array[count];
    }

    T& Back() const
    {
        return array[count - 1];
    }

    // O(n)
    void Insert(uint32 index, const T& data)
    {
        assert(index <= count);

        if (count == capacity)
        {
            T* old = array;
            capacity *= 2;

            array = (T*)malloc(capacity * sizeof(T));
            memcpy(array, old, count * sizeof(T));

            if (old != stack_array)
            {
                free(old);
            }
        }

        uint32 ptr = count;
        while (index != ptr)
        {
            array[ptr] = array[ptr - 1];
            --ptr;
        }

        array[index] = data;
        ++count;
    }

    // O(n)
    void Remove(uint32 index)
    {
        uint32 ptr = index;
        while (ptr != count)
        {
            array[ptr] = array[ptr + 1];
            ++ptr;
        }

        --count;
    }

    uint32 Count() const
    {
        return count;
    }

    void Clear()
    {
        count = 0;
    }

    void Reset()
    {
        if (array != stack_array)
        {
            free(array);
        }
        array = stack_array;
        count = 0;
    }

    uint32 Capacity() const
    {
        return capacity;
    }

    T& At(uint32 index) const
    {
        return array[index];
    }

    T& operator[](uint32 index) const
    {
        return array[index];
    }

private:
    T* array;
    T stack_array[N];
    uint32 count;
    uint32 capacity;
};

} // namespace muli
