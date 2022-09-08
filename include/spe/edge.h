#pragma once

#include "common.h"
#include "contact_point.h"

namespace spe
{

struct Edge
{
    ContactPoint p1;
    ContactPoint p2;

    Vec2 dir;
    Vec2 normal;

    Edge() = default;

    Edge(const Vec2 _p1, const Vec2& _p2, int32_t _id1, int32_t _id2)
        : p1{ ContactPoint{ _p1, _id1 } }
        , p2{ ContactPoint{ _p2, _id2 } }
    {
        dir = (p1.position == p2.position) ? Vec2{ 0.0f } : (p2.position - p1.position).Normalized();
        normal = cross(1.0f, dir);
    }

    Edge(const Vec2& _p1, const Vec2& _p2)
        : Edge(_p1, _p2, -1, -1)
    {
    }

    float Length() const
    {
        return spe::length(p2.position - p1.position);
    }
};

} // namespace spe