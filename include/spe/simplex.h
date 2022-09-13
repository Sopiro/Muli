#pragma once

#include "common.h"

#define MAX_SIMPLEX_VERTEX_COUNT 3

namespace spe
{

struct ClosestResult
{
    Vec2 point;
    uint32 contributors[MAX_SIMPLEX_VERTEX_COUNT]; // Vertex indices that contributed to calculating the closest point
    uint32 count;
};

class Simplex
{
public:
    Simplex() = default;

    Vec2 vertices[MAX_SIMPLEX_VERTEX_COUNT];

    uint32 Count() const;
    void Clear();
    void AddVertex(const Vec2& vertex);
    bool ContainsVertex(const Vec2& vertex) const;
    void Shrink(const uint32* _indices, uint32 _count);

    // Returns the closest point to the input q
    ClosestResult GetClosest(const Vec2& q) const;

private:
    uint32 count = 0;
};

inline uint32 Simplex::Count() const
{
    return count;
}

inline void Simplex::Clear()
{
    count = 0;
}

inline void Simplex::AddVertex(const Vec2& vertex)
{
    if (count == MAX_SIMPLEX_VERTEX_COUNT) throw std::exception("2-simplex can have verticies less than 4");

    vertices[count] = vertex;
    count++;
}

inline bool Simplex::ContainsVertex(const Vec2& vertex) const
{
    for (uint32 i = 0; i < count; i++)
    {
        if (vertex == vertices[i]) return true;
    }

    return false;
}

inline void Simplex::Shrink(const uint32* _indices, uint32 _count)
{
    Vec2 tmp[MAX_SIMPLEX_VERTEX_COUNT];

    for (uint32 i = 0; i < _count; i++)
    {
        tmp[i] = vertices[_indices[i]];
    }

    memcpy(vertices, tmp, _count * sizeof(Vec2));
    count = _count;
}

} // namespace spe