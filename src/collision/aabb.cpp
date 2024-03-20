#include "muli/aabb.h"

namespace muli
{

// Slab method
bool AABB::TestRay(const Vec2& from, const Vec2& to, float tMin, float tMax) const
{
    Vec2 dir = to - from;

    for (int32 axis = 0; axis < 2; ++axis)
    {
        float invD = 1.0f / dir[axis];
        float origin = from[axis];

        float t0 = (min[axis] - origin) * invD;
        float t1 = (max[axis] - origin) * invD;

        if (invD < 0.0)
        {
            std::swap(t0, t1);
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

float AABB::RayCast(const Vec2& from, const Vec2& to, float tMin, float tMax) const
{
    Vec2 dir = to - from;

    for (int32 axis = 0; axis < 2; ++axis)
    {
        float invD = 1.0f / dir[axis];
        float origin = from[axis];

        float t0 = (min[axis] - origin) * invD;
        float t1 = (max[axis] - origin) * invD;

        if (invD < 0.0)
        {
            std::swap(t0, t1);
        }

        tMin = t0 > tMin ? t0 : tMin;
        tMax = t1 < tMax ? t1 : tMax;

        if (tMax <= tMin)
        {
            return max_value;
        }
    }

    return tMin;
}

} // namespace muli
