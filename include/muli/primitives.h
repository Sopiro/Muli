#pragma once

#include "common.h"

namespace muli
{

// Geometric primitives

struct Point
{
    Vec2 p;   // vertex position
    int32 id; // vertex index(id)
};

struct Edge
{
    Edge(const Point& point1, const Point& point2);
    Edge(const Vec2& point1, const Vec2& point2, int32 id1 = -1, int32 id2 = -1);

    void ComputeProperty();
    void Translate(const Vec2& d);
    float GetLength2() const;

    Point p1, p2;
    Vec2 tangent, normal;
};

inline Edge::Edge(const Point& point1, const Point& point2)
    : p1{ point1 }
    , p2{ point2 }
{
    ComputeProperty();
}

inline Edge::Edge(const Vec2& point1, const Vec2& point2, int32 id1, int32 id2)
    : p1{ point1, id1 }
    , p2{ point2, id2 }
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
