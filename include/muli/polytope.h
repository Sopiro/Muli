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

struct Polytope
{
    GrowableArray<Vec2, 8> vertices;

    Polytope(const Simplex& simplex);
    PolytopeEdge GetClosestEdge() const;
};

inline Polytope::Polytope(const Simplex& simplex)
{
    vertices.Emplace(simplex.vertices[0].point);
    vertices.Emplace(simplex.vertices[1].point);
    vertices.Emplace(simplex.vertices[2].point);
}

} // namespace muli