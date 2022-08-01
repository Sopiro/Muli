#pragma once

#include "common.h"

namespace spe
{
struct ClosestResult
{
    glm::vec2 result;
    std::vector<uint32_t> contributors; // Vertex indices that contributed to calculating the closest point
};

class Simplex
{
public:
    std::vector<glm::vec2> vertices{};

    Simplex()
    {
        vertices.reserve(3);
    }

    inline size_t Count() const
    {
        return vertices.size();
    }

    inline void Clear()
    {
        vertices.clear();
    }

    inline void AddVertex(const glm::vec2& vertex)
    {
        if (vertices.size() >= 3)
            throw std::exception("2-simplex can have verticies less than 4");

        vertices.push_back(vertex);
    }

    inline bool ContainsVertex(const glm::vec2& vertex) const
    {
        for (size_t i = 0; i < vertices.size(); i++)
        {
            if (vertex == vertices[i])
                return true;
        }

        return false;
    }

    inline void Shrink(const std::vector<uint32_t>& indices)
    {
        std::vector<glm::vec2> res{};
        res.reserve(indices.size());

        for (size_t i = 0; i < indices.size(); i++)
        {
            res.push_back(vertices[indices[i]]);
        }

        vertices = std::move(res);
    }

    // Returns the closest point to the input q
    ClosestResult GetClosest(const glm::vec2& q) const;
};
}