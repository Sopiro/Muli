#pragma once

#include "common.h"

namespace muli
{

struct AABB
{
    Vec2 GetCenter() const
    {
        return (min + max) * 0.5f;
    }

    float GetArea() const
    {
        return (max.x - min.x) * (max.y - min.y);
    }

    float GetPerimeter(const AABB& aabb)
    {
        Vec2 w = max - min;
        return 2.0f * (w.x + w.y);
    }

    bool Contains(const AABB& aabb) const
    {
        return min.x <= aabb.min.x && min.y <= aabb.min.y && max.x >= aabb.max.x && max.y >= aabb.max.y;
    }

    Vec2 min;
    Vec2 max;
};

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

} // namespace muli
