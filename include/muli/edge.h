#pragma once

#include "common.h"
#include "contact_point.h"

namespace muli
{

struct Edge
{
    Edge() = default;
    Edge(const ContactPoint& p1, const ContactPoint& p2);
    Edge(const Vec2& p1, const Vec2& p2, int32 id1 = -1, int32 id2 = -1);

    void ComputeProperty();
    void Translate(const Vec2& d);
    float GetLength() const;
    float GetLength2() const;

    ContactPoint p1, p2;

    Vec2 dir;
    Vec2 normal;
};

inline Edge::Edge(const ContactPoint& _p1, const ContactPoint& _p2)
    : p1{ _p1 }
    , p2{ _p2 }
{
    ComputeProperty();
}

inline Edge::Edge(const Vec2& _p1, const Vec2& _p2, int32 _id1, int32 _id2)
    : p1{ _p1, _id1 }
    , p2{ _p2, _id2 }
{
    ComputeProperty();
}

inline void Edge::ComputeProperty()
{
    if (p1.position == p2.position)
    {
        dir.SetZero();
    }
    else
    {
        dir = Normalize(p2.position - p1.position);
    }

    normal = Cross(1.0f, dir);
}

inline void Edge::Translate(const Vec2& d)
{
    p1.position += d;
    p2.position += d;
}

inline float Edge::GetLength() const
{
    return Dist(p1.position, p2.position);
}

inline float Edge::GetLength2() const
{
    return Dist2(p1.position, p2.position);
}

} // namespace muli