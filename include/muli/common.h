#pragma once

// stds
#include <algorithm>
#include <array>
#include <cassert>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <format>
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

#define MuliAssert(A) assert(A)
#define MuliNotUsed(x) ((void)(x))

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