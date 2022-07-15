#pragma once

#include "../common.h"

namespace spe
{
    struct Edge
    {
        glm::vec2 p1;
        glm::vec2 p2;
        glm::vec2 dir;

        int32_t id1;
        int32_t id2;

        Edge(glm::vec2 _p1, glm::vec2 _p2, int32_t _id1 = -1, int32_t _id2 = -1);

        inline float Length() const
        {
            return glm::length(p2 - p1);
        }

        inline glm::vec2 Normal() const
        {
            return glm::cross(1.0f, dir);
        }
    };
}