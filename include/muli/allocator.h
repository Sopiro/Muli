#pragma once

#include "common.h"
#include "util.h"

namespace muli
{

class Allocator
{
public:
    Allocator() = default;
    virtual ~Allocator() = default;

    virtual void* Allocate(int32 size) = 0;
    virtual void Free(void* p, int32 size) = 0;
    virtual void Clear() = 0;
};

} // namespace muli
