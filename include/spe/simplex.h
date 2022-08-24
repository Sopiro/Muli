#pragma once

#include "common.h"

#define MAX_SIMPLEX_VERTEX_COUNT 3

namespace spe
{
struct ClosestResult
{
    glm::vec2 result;
    std::array<uint32_t, MAX_SIMPLEX_VERTEX_COUNT> contributors; // Vertex indices that contributed to calculating the closest point
    uint32_t count;
};

class Simplex
{
public:
    Simplex() = default;

    std::array<glm::vec2, MAX_SIMPLEX_VERTEX_COUNT> vertices{};

    size_t Count() const;
    void Clear();
    void AddVertex(const glm::vec2& vertex);
    bool ContainsVertex(const glm::vec2& vertex) const;
    void Shrink(const std::array<uint32_t, MAX_SIMPLEX_VERTEX_COUNT>& _indices, uint32_t _count);

    // Returns the closest point to the input q
    ClosestResult GetClosest(const glm::vec2& q) const;

private:
    uint32_t count = 0;
};

inline size_t Simplex::Count() const
{
    return count;
}

inline void Simplex::Clear()
{
    count = 0;
}

inline void Simplex::AddVertex(const glm::vec2& vertex)
{
    if (count == MAX_SIMPLEX_VERTEX_COUNT)
        throw std::exception("2-simplex can have verticies less than 4");

    vertices[count] = vertex;
    count++;
}

inline bool Simplex::ContainsVertex(const glm::vec2& vertex) const
{
    for (size_t i = 0; i < vertices.size(); i++)
    {
        if (vertex == vertices[i])
            return true;
    }

    return false;
}

inline void Simplex::Shrink(const std::array<uint32_t, MAX_SIMPLEX_VERTEX_COUNT>& _indices, uint32_t _count)
{
    std::array<glm::vec2, MAX_SIMPLEX_VERTEX_COUNT> tmp;

    for (uint32_t i = 0; i < _count; i++)
    {
        tmp[i] = vertices[_indices[i]];
    }

    vertices = std::move(tmp);
    count = _count;
}

}