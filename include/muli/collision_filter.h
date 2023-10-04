#pragma once

#include "common.h"

namespace muli
{

struct CollisionFilter
{
    uint32 group = 1;
    uint32 bit = 1;
    uint32 mask = 0xffffffff;
};

constexpr CollisionFilter default_collision_filter{};

inline bool EvaluateFilter(const CollisionFilter& filterA, const CollisionFilter& filterB)
{
    if (filterA.group != filterB.group)
    {
        return true;
    }

    return (filterA.mask & filterB.bit) != 0 && (filterB.mask & filterA.bit) != 0;
}

} // namespace muli
