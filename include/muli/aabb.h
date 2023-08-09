#pragma once

#include "common.h"

namespace muli
{

struct AABB
{
    AABB() = default;

    AABB(const Vec2& _min, const Vec2& _max)
        : min{ _min }
        , max{ _max }
    {
    }

    Vec2 GetCenter() const
    {
        return (min + max) * 0.5f;
    }

    Vec2 GetExtents() const
    {
        return (max - min) * 0.5f;
    }

    float GetArea() const
    {
        return (max.x - min.x) * (max.y - min.y);
    }

    float GetPerimeter()
    {
        Vec2 w = max - min;
        return 2.0f * (w.x + w.y);
    }

    bool Contains(const AABB& aabb) const
    {
        return min.x <= aabb.min.x && min.y <= aabb.min.y && max.x >= aabb.max.x && max.y >= aabb.max.y;
    }

    bool TestPoint(const Vec2& point) const
    {
        if (min.x > point.x || max.x < point.x) return false;
        if (min.y > point.y || max.y < point.y) return false;

        return true;
    }

    bool TestOverlap(const AABB& other) const
    {
        if (min.x > other.max.x || max.x < other.min.x) return false;
        if (min.y > other.max.y || max.y < other.min.y) return false;

        return true;
    }

    bool TestRay(const Vec2& from, const Vec2& to, float tMin, float tMax) const
    {
        Vec2 dir = to - from;

        for (int32 axis = 0; axis < 2; ++axis)
        {
            float invD = 1.0f / dir[axis];

            float t0 = (min[axis] - from[axis]) * invD;
            float t1 = (max[axis] - from[axis]) * invD;

            if (invD < 0.0)
            {
                float tmp = t0;
                t0 = t1;
                t1 = tmp;
            }

            tMin = t0 > tMin ? t0 : tMin;
            tMax = t1 < tMax ? t1 : tMax;

            if (tMax <= tMin)
            {
                return false;
            }
        }

        return true;
    }

    Vec2 min;
    Vec2 max;
};

inline void Fix(AABB& aabb)
{
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

} // namespace muli
