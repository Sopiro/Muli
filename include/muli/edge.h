#pragma once

#include "common.h"
#include "contact_point.h"

namespace muli
{

struct Edge
{
    ContactPoint p1;
    ContactPoint p2;

    Vec2 dir;
    Vec2 normal;

    Edge() = default;

    Edge(const ContactPoint& cp1, const ContactPoint& cp2)
        : p1{ cp1 }
        , p2{ cp2 }
    {
        ComputeProperty();
    }

    Edge(const Vec2& _p1, const Vec2& _p2, int32 _id1, int32 _id2)
    {
        p1.position = _p1;
        p2.position = _p2;
        p1.id = _id1;
        p2.id = _id2;

        ComputeProperty();
    }

    Edge(const Vec2& _p1, const Vec2& _p2)
    {
        p1.position = _p1;
        p2.position = _p2;
        p1.id = -1;
        p2.id = -1;

        ComputeProperty();
    }

    void ComputeProperty()
    {
        dir = (p1.position == p2.position) ? Vec2{ 0.0f } : (p2.position - p1.position).Normalized();
        normal = dir.Skew();
    }

    float GetLength() const
    {
        return (p2.position - p1.position).Length();
    }

    void Translate(Vec2&& d)
    {
        p1.position += d;
        p2.position += d;
    }
};

} // namespace muli