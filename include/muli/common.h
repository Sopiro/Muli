#pragma once

// stds
#include <algorithm>
#include <array>
#include <cassert>
#include <cstring>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <span>
#include <string>
#include <unordered_set>
#include <vector>

#include "math.h"
#include "types.h"

#define muliAssert(A) assert(A)
#define muliNotUsed(x) ((void)(x))

namespace muli
{

// default alloc/dealloc funcitons

inline void* Alloc(int32 size)
{
    return std::malloc(size);
}

inline void Free(void* mem)
{
    std::free(mem);
}

} // namespace muli