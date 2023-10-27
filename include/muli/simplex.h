#pragma once

#include "common.h"
#include "contact_point.h"

namespace muli
{

constexpr int32 max_simplex_vertex_count = 3;

struct SupportPoint
{
    ContactPoint pointA;
    ContactPoint pointB;
    Vec2 point; // pointA - pointB
    float weight;
};

struct Simplex
{
    Simplex() = default;

    void AddVertex(const SupportPoint& vertex);
    void Save(Vec2* saveVertices, int32* saveCount);
    void Advance(const Vec2& q);
    Vec2 GetSearchDirection() const;
    Vec2 GetClosestPoint() const;
    void GetWitnessPoint(Vec2* pointA, Vec2* pointB);

    int32 count = 0;
    SupportPoint vertices[max_simplex_vertex_count];

    float divisor;
};

inline void Simplex::AddVertex(const SupportPoint& vertex)
{
    muliAssert(count != max_simplex_vertex_count);

    vertices[count++] = vertex;
}

inline void Simplex::Save(Vec2* saveVertices, int32* saveCount)
{
    *saveCount = count;
    for (int32 i = 0; i < count; ++i)
    {
        saveVertices[i] = vertices[i].point;
    }
}

} // namespace muli