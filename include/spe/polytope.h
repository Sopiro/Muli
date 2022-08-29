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
    glm::vec2 normal;
};

class Polytope
{
public:
    GrowableArray<glm::vec2, 8> vertices;

    Polytope(const Simplex& simplex);

    ClosestEdgeInfo GetClosestEdge() const;
    uint32_t Count() const;
};

inline uint32_t Polytope::Count() const
{
    return vertices.Count();
}

} // namespace spe