#pragma once

#include "contact_point.h"

namespace muli
{

struct Edge
{
    Edge(const ContactPoint& p1, const ContactPoint& p2);
    Edge(const Vec2& p1, const Vec2& p2, int32 id1 = -1, int32 id2 = -1);

    void ComputeProperty();
    void Translate(const Vec2& d);
    float GetLength2() const;

    ContactPoint p1, p2;

    Vec2 tangent;
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
    if (p1.p == p2.p)
    {
        tangent.SetZero();
        normal.SetZero();
    }
    else
    {
        tangent = Normalize(p2.p - p1.p);
        normal = Cross(1.0f, tangent);
    }
}

inline void Edge::Translate(const Vec2& d)
{
    p1.p += d;
    p2.p += d;
}

inline float Edge::GetLength2() const
{
    return Dist2(p1.p, p2.p);
}

} // namespace muli