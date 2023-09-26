#include "muli/aabb.h"

namespace muli
{

bool AABB::TestRay(const Vec2& from, const Vec2& to, float tMin, float tMax) const
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

} // namespace muli
