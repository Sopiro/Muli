#pragma once

#include "common.h"
#include "growable_array.h"
#include "simplex.h"

namespace spe
{

struct ClosestEdgeInfo
{
    uint32_t index;
    float distance;
    Vec2 normal;
};

class Polytope
{
public:
    GrowableArray<Vec2, 8> vertices;

    Polytope(const Simplex& simplex);

    ClosestEdgeInfo GetClosestEdge() const;
    uint32_t Count() const;
};

inline Polytope::Polytope(const Simplex& simplex)
{
    vertices.Push(simplex.vertices[0]);
    vertices.Push(simplex.vertices[1]);
    vertices.Push(simplex.vertices[2]);
}

inline uint32_t Polytope::Count() const
{
    return vertices.Count();
}

} // namespace spe