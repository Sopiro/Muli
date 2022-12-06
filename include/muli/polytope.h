#pragma once

#include "common.h"
#include "growable_array.h"
#include "simplex.h"

namespace muli
{

struct PolytopeEdge
{
    int32 index;
    float distance;
    Vec2 normal;
};

class Polytope
{
public:
    Polytope(const Simplex& simplex);

    // Returns the edge closest to the Origin (0, 0)
    PolytopeEdge GetClosestEdge() const;

    GrowableArray<Vec2, 8> vertices;
};

inline Polytope::Polytope(const Simplex& simplex)
{
    vertices.Emplace(simplex.vertices[0].point);
    vertices.Emplace(simplex.vertices[1].point);
    vertices.Emplace(simplex.vertices[2].point);
}

} // namespace muli