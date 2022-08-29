#pragma once

#include "stdlib.h"

namespace spe
{

// Inspired by b2GrowableStack in box2d code
// Use only element types that don't need a destructor
template <typename T, uint32_t N>
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
        assert(this != &other);
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
        assert(this != &other);
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
    void Insert(uint32_t index, const T& data)
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

        uint32_t ptr = count;
        while (index != ptr)
        {
            array[ptr] = array[ptr - 1];
            --ptr;
        }

        array[index] = data;
        ++count;
    }

    // O(n)
    void Remove(uint32_t index)
    {
        uint32_t ptr = index;
        while (ptr != count)
        {
            array[ptr] = array[ptr + 1];
            ++ptr;
        }

        --count;
    }

    uint32_t Count() const
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

    uint32_t Capacity() const
    {
        return capacity;
    }

    T& At(uint32_t index) const
    {
        return array[index];
    }

    T& operator[](uint32_t index) const
    {
        return array[index];
    }

private:
    T* array;
    T stack_array[N];
    uint32_t count;
    uint32_t capacity;
};

} // namespace spe
