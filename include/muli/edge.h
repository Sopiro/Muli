#pragma once

#include "common.h"
#include "contact_point.h"

namespace muli
{

struct Edge
{
    Edge() = default;

    Edge(const ContactPoint& _p1, const ContactPoint& _p2)
        : p1{ _p1 }
        , p2{ _p2 }
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
        if (p1.position == p2.position)
        {
            dir.SetZero();
        }
        else
        {
            dir = (p2.position - p1.position).Normalized();
        }

        normal = Cross(1.0f, dir);
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

    ContactPoint p1;
    ContactPoint p2;

    Vec2 dir;
    Vec2 normal;
};

} // namespace muli