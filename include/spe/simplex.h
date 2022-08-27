#pragma once

#include "common.h"

#define MAX_SIMPLEX_VERTEX_COUNT 3

namespace spe
{

struct ClosestResult
{
    glm::vec2 result;
    uint32_t contributors[MAX_SIMPLEX_VERTEX_COUNT]; // Vertex indices that contributed to calculating the closest point
    uint32_t count;
};

class Simplex
{
public:
    Simplex() = default;

    glm::vec2 vertices[MAX_SIMPLEX_VERTEX_COUNT];

    uint32_t Count() const;
    void Clear();
    void AddVertex(const glm::vec2& vertex);
    bool ContainsVertex(const glm::vec2& vertex) const;
    void Shrink(const uint32_t* _indices, uint32_t _count);

    // Returns the closest point to the input q
    ClosestResult GetClosest(const glm::vec2& q) const;

private:
    uint32_t count = 0;
};

inline uint32_t Simplex::Count() const
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
    for (uint32_t i = 0; i < count; i++)
    {
        if (vertex == vertices[i])
            return true;
    }

    return false;
}

inline void Simplex::Shrink(const uint32_t* _indices, uint32_t _count)
{
    glm::vec2 tmp[MAX_SIMPLEX_VERTEX_COUNT];

    for (uint32_t i = 0; i < _count; i++)
    {
        tmp[i] = vertices[_indices[i]];
    }

    memcpy(vertices, tmp, _count * sizeof(glm::vec2));
    count = _count;
}

}