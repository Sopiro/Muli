#pragma once

#include "common.h"
#include "contact_point.h"

#define MAX_SIMPLEX_VERTEX_COUNT 3

namespace muli
{

struct ClosestResult
{
    Vec2 point;
    int16 contributors[MAX_SIMPLEX_VERTEX_COUNT]; // Vertex indices contributed to calculating the closest point
    int16 count;
};

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
    SupportPoint vertices[MAX_SIMPLEX_VERTEX_COUNT];

    float divisor;
};

inline void Simplex::AddVertex(const SupportPoint& vertex)
{
    muliAssert(count != MAX_SIMPLEX_VERTEX_COUNT);

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

inline Vec2 Simplex::GetSearchDirection() const
{
    switch (count)
    {
    case 1:
        return -vertices[0].point;

    case 2:
    {
        // Triple product: a⨯b⨯c = b(a⋅c) - c(a⋅b)
        // In this case, ab⨯ao⨯ab = ao(ab⋅ab) - ab(ab⋅ao)
        Vec2 ab = vertices[1].point - vertices[0].point;
        Vec2 ao = -vertices[0].point;

        float d1 = Dot(ab, ab);
        float d2 = Dot(ab, ao);

        return ao * d1 - ab * d2;
    }
    default:
        muliAssert(false);
        return zero_vec2;
    }
}

inline Vec2 Simplex::GetClosestPoint() const
{
    switch (count)
    {
    case 1:
        return vertices[0].point;

    case 2:
    {
        float d = 1.0f / divisor;
        return (d * vertices[0].weight) * vertices[0].point + (d * vertices[1].weight) * vertices[1].point;
    }
    case 3:
        return zero_vec2;

    default:
        muliAssert(false);
        return zero_vec2;
    }
}

inline void Simplex::GetWitnessPoint(Vec2* pointA, Vec2* pointB)
{
    switch (count)
    {
    case 1:
    {
        *pointA = vertices[0].pointA.position;
        *pointB = vertices[0].pointB.position;
        return;
    }

    case 2:
    {
        float d = 1.0f / divisor;
        *pointA = (d * vertices[0].weight) * vertices[0].pointA.position + (d * vertices[1].weight) * vertices[1].pointA.position;
        *pointB = (d * vertices[0].weight) * vertices[0].pointB.position + (d * vertices[1].weight) * vertices[1].pointB.position;
        return;
    }

    case 3:
    {
        // clang-format off
        float d = 1.0f / divisor;
        *pointA = (d * vertices[0].weight) * vertices[0].pointA.position +
                  (d * vertices[1].weight) * vertices[1].pointA.position +
                  (d * vertices[2].weight) * vertices[2].pointA.position;
        *pointB = *pointA;
        return;
        // clang-format on
    }

    default:
        muliAssert(false);
    }
}

} // namespace muli