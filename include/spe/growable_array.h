#pragma once

#include "stdlib.h"
#include "stdio.h"

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

    /// O(n)
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

    /// O(n)
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

    void Reset()
    {
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