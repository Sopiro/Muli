#pragma once

#include "common.h"

namespace muli
{

struct AABB
{
    AABB() = default;
    AABB(const Vec2& min, const Vec2& max);

    Vec2 GetCenter() const;
    Vec2 GetHalfExtents() const;

    float GetArea() const;
    float GetPerimeter() const;

    bool Contains(const AABB& aabb) const;
    bool TestPoint(const Vec2& point) const;
    bool TestOverlap(const AABB& other) const;
    bool TestRay(const Vec2& from, const Vec2& to, float tMin, float tMax, Vec2 margin = Vec2::zero) const;
    float RayCast(const Vec2& from, const Vec2& to, float tMin, float tMax, Vec2 margin = Vec2::zero) const;

    Vec2 min;
    Vec2 max;

    static AABB Union(const AABB& b1, const AABB& b2);
};

inline AABB::AABB(const Vec2& _min, const Vec2& _max)
    : min{ _min }
    , max{ _max }
{
}

inline Vec2 AABB::GetCenter() const
{
    return (min + max) * 0.5f;
}

inline Vec2 AABB::GetHalfExtents() const
{
    return (max - min) * 0.5f;
}

inline float AABB::GetArea() const
{
    return (max.x - min.x) * (max.y - min.y);
}

inline float AABB::GetPerimeter() const
{
    Vec2 d = max - min;
    return 2.0f * (d.x + d.y);
}

inline bool AABB::Contains(const AABB& aabb) const
{
    return min.x <= aabb.min.x && min.y <= aabb.min.y && max.x >= aabb.max.x && max.y >= aabb.max.y;
}

inline bool AABB::TestPoint(const Vec2& point) const
{
    if (min.x > point.x || max.x < point.x) return false;
    if (min.y > point.y || max.y < point.y) return false;

    return true;
}

inline bool AABB::TestOverlap(const AABB& other) const
{
    if (min.x > other.max.x || max.x < other.min.x) return false;
    if (min.y > other.max.y || max.y < other.min.y) return false;

    return true;
}

inline AABB AABB::Union(const AABB& b1, const AABB& b2)
{
    Vec2 min = Min(b1.min, b2.min);
    Vec2 max = Max(b1.max, b2.max);

    return AABB{ min, max };
}

} // namespace muli
