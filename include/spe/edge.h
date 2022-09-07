#pragma once

#include "common.h"
#include "contact_point.h"

namespace spe
{

struct Edge
{
    ContactPoint p1;
    ContactPoint p2;

    glm::vec2 dir;
    glm::vec2 normal;

    Edge() = default;

    Edge(const glm::vec2 _p1, const glm::vec2& _p2, int32_t _id1, int32_t _id2)
        : p1{ ContactPoint{ _p1, _id1 } }
        , p2{ ContactPoint{ _p2, _id2 } }
    {
        dir = (p1.position == p2.position) ? glm::vec2{ 0.0f } : glm::normalize(p2.position - p1.position);
        normal = glm::cross(1.0f, dir);
    }

    Edge(const glm::vec2& _p1, const glm::vec2& _p2)
        : Edge(_p1, _p2, -1, -1)
    {
    }

    float Length() const
    {
        return glm::length(p2.position - p1.position);
    }
};

} // namespace spe