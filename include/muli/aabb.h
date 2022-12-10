#pragma once

#include "common.h"

namespace muli
{

struct AABB
{
    Vec2 min;
    Vec2 max;

    Vec2 GetCenter() const
    {
        return (min + max) * 0.5f;
    }
};

inline float Area(const AABB& aabb)
{
    return (aabb.max.x - aabb.min.x) * (aabb.max.y - aabb.min.y);
}

inline float Perimeter(const AABB& aabb)
{
    Vec2 w = aabb.max - aabb.min;
    return 2.0f * (w.x + w.y);
}

inline void Fix(AABB& aabb)
{
    auto a = Max(aabb.min, aabb.max);

    Vec2 newMin = Min(aabb.min, aabb.max);
    Vec2 newMax = Max(aabb.min, aabb.max);

    aabb.min = newMin;
    aabb.max = newMax;
}

inline AABB Union(const AABB& b1, const AABB& b2)
{
    Vec2 min = Min(b1.min, b2.min);
    Vec2 max = Max(b1.max, b2.max);

    return AABB{ min, max };
}

inline bool TestPointInsideAABB(const AABB& aabb, const Vec2& point)
{
    if (aabb.min.x > point.x || aabb.max.x < point.x) return false;
    if (aabb.min.y > point.y || aabb.max.y < point.y) return false;

    return true;
}

inline bool TestOverlapAABB(const AABB& a, const AABB& b)
{
    if (a.min.x > b.max.x || a.max.x < b.min.x) return false;
    if (a.min.y > b.max.y || a.max.y < b.min.y) return false;

    return true;
}

inline bool ContainsAABB(const AABB& container, const AABB& testee)
{
    // clang-format off
    return container.min.x <= testee.min.x
        && container.min.y <= testee.min.y
        && container.max.x >= testee.max.x
        && container.max.y >= testee.max.y;
    // clang-format on
}

} // namespace muli
