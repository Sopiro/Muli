#pragma once

#include "common.h"
#include "contact_point.h"

#define MAX_SIMPLEX_VERTEX_COUNT 3

namespace muli
{

struct ClosestResult
{
    Vec2 point;
    int32 contributors[MAX_SIMPLEX_VERTEX_COUNT]; // Vertex indices that contributed to calculating the closest point
    int32 count;
};

struct SupportPoint
{
    ContactPoint pointA;
    ContactPoint pointB;
    Vec2 point; // pointA - pointB
};

class Simplex
{
public:
    Simplex() = default;

    void AddVertex(const SupportPoint& vertex);
    bool ContainsVertex(const Vec2& vertex) const;
    int32 VertexCount() const;
    void Clear();

    ClosestResult GetClosestPoint(const Vec2& q) const;
    void Shrink(const int32* indices, int32 count);

    SupportPoint vertices[MAX_SIMPLEX_VERTEX_COUNT];

private:
    int32 count = 0;
};

inline int32 Simplex::VertexCount() const
{
    return count;
}

inline void Simplex::Clear()
{
    count = 0;
}

inline void Simplex::AddVertex(const SupportPoint& vertex)
{
    muliAssert(count != MAX_SIMPLEX_VERTEX_COUNT);

    vertices[count++] = vertex;
}

inline bool Simplex::ContainsVertex(const Vec2& vertex) const
{
    for (int32 i = 0; i < count; ++i)
    {
        if (vertex == vertices[i].point)
        {
            return true;
        }
    }

    return false;
}

inline void Simplex::Shrink(const int32* _indices, int32 _count)
{
    SupportPoint tmp[MAX_SIMPLEX_VERTEX_COUNT];

    for (int32 i = 0; i < _count; ++i)
    {
        tmp[i] = vertices[_indices[i]];
    }

    memcpy(vertices, tmp, _count * sizeof(SupportPoint));
    count = _count;
}

} // namespace muli