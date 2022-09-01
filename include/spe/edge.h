#pragma once

#include "common.h"

namespace spe
{

struct Edge
{
    glm::vec2 p1;
    glm::vec2 p2;
    int32_t id1;
    int32_t id2;
    glm::vec2 dir;

    Edge(const glm::vec2& _p1, const glm::vec2& _p2, int32_t _id1 = -1, int32_t _id2 = -1)
        : p1{ _p1 }
        , p2{ _p2 }
        , id1{ _id1 }
        , id2{ _id2 }
    {
        dir = (p1 == p2) ? glm::vec2{ 0.0f } : glm::normalize(p2 - p1);
    }

    float Length() const
    {
        return glm::length(p2 - p1);
    }

    glm::vec2 Normal() const
    {
        return glm::cross(1.0f, dir);
    }
};

} // namespace spe