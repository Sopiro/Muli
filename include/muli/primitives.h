#pragma once

#include "math.h"

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
    Edge() = default;

    Point p1, p2;
    Vec2 normal;
};

} // namespace muli
