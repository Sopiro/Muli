#pragma once

#include "common.h"

namespace muli
{

struct CollisionFilter
{
    int32 group = 0;          // Collision group: same non-zero value means special behavior
                              // Positive: always collide
                              // Negative: never collide
                              // Zero: No group

    uint32 bit = 1;           // Collision bit representing this object's collision type
    uint32 mask = 0xffffffff; // Collision mask: which collision bits this object can collide with (default: all)
};

constexpr CollisionFilter default_collision_filter{};

inline bool EvaluateFilter(const CollisionFilter& filterA, const CollisionFilter& filterB)
{
    if (filterA.group == filterB.group && filterA.group != 0)
    {
        return filterA.group > 0;
    }

    return (filterA.mask & filterB.bit) != 0 && (filterB.mask & filterA.bit) != 0;
}

} // namespace muli
