#pragma once

#include "common.h"

namespace spe
{

struct Edge
{
    glm::vec2 p1;
    glm::vec2 p2;
    int32_t id1 = -1;
    int32_t id2 = -1;

    glm::vec2 dir;
    glm::vec2 normal;

    void ComputeDir()
    {
        dir = (p1 == p2) ? glm::vec2{ 0.0f } : glm::normalize(p2 - p1);
        normal = glm::cross(1.0f, dir);
    }

    float Length() const
    {
        return glm::length(p2 - p1);
    }
};

} // namespace spe