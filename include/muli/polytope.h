#pragma once

#include "common.h"
#include "growable_array.h"
#include "simplex.h"

namespace muli
{

struct ClosestEdgeInfo
{
    uint32 index;
    float distance;
    Vec2 normal;
};

class Polytope
{
public:
    GrowableArray<Vec2, 8> vertices;

    Polytope(const Simplex& simplex);

    // Returns the edge closest to the Origin (0, 0)
    ClosestEdgeInfo GetClosestEdge() const;
};

inline Polytope::Polytope(const Simplex& simplex)
{
    vertices.Push(simplex.vertices[0]);
    vertices.Push(simplex.vertices[1]);
    vertices.Push(simplex.vertices[2]);
}

} // namespace muli