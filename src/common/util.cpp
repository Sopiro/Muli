#include "muli/util.h"
#include "muli/aabb.h"
#include "muli/circle.h"
#include "muli/polygon.h"

namespace muli
{

void ComputeConvexHull(const Vec2* vertices, int32 vertexCount, Vec2* out)
{
    if (vertexCount < 3)
    {
        memcpy(out, vertices, vertexCount);
        return;
    }

    // Find the lowest vertex
    int32 index = 0;
    for (int32 i = 1; i < vertexCount; i++)
    {
        if (vertices[i].y < vertices[index].y)
        {
            index = i;
        }
    }
    Vec2 bottom = vertices[index];

    // Sort the vertices based on the angle related to the lowest vertex.
    Vec2* sorted = (Vec2*)malloc(vertexCount * sizeof(Vec2));
    std::partial_sort_copy(vertices, vertices + vertexCount, sorted, sorted + vertexCount,
                           [&](const Vec2& a, const Vec2& b) -> bool {
                               Vec2 ra = a - bottom;
                               Vec2 rb = b - bottom;

                               return Atan2(ra.y, ra.x) < Atan2(rb.y, rb.x);
                           });
    muliAssert(sorted[0] == bottom);

    // Discard overlapped bottom vertices
    int32 i = 0;
    while (i < vertexCount)
    {
        Vec2& v0 = sorted[i];
        Vec2& v1 = sorted[(i + 1) % vertexCount];

        if (v0 == v1)
        {
            i++;
        }
        else
        {
            break;
        }
    }

    int32 sp = 0; // stack pointer
    out[sp++] = sorted[i++];
    out[sp++] = sorted[i++];

    while (i < vertexCount)
    {
        Vec2& v = sorted[i];
        int32 l = sp;

        if (v == out[l - 1])
        {
            ++i;
            continue;
        }

        Vec2 d1 = out[l - 1] - out[l - 2];
        Vec2 d2 = v - out[l - 1];

        if (Cross(d1, d2) <= 0)
        {
            --sp;

            if (l < 3)
            {
                out[sp++] = v;
                break;
            }
        }
        else
        {
            out[sp++] = v;
            ++i;
        }
    }

    free(sorted);
}

std::vector<Vec2> ComputeConvexHull(const std::vector<Vec2>& vertices)
{
    if (vertices.size() < 3)
    {
        return vertices;
    }

    uint32 index = 0;
    for (uint32 i = 1; i < vertices.size(); i++)
    {
        if (vertices[i].y < vertices[index].y)
        {
            index = i;
        }
    }

    Vec2 bottom = vertices[index];
    std::vector<Vec2> sorted(vertices.size());

    std::partial_sort_copy(vertices.begin(), vertices.end(), sorted.begin(), sorted.end(),
                           [&](const Vec2& a, const Vec2& b) -> bool {
                               Vec2 ra = a - bottom;
                               Vec2 rb = b - bottom;

                               return Atan2(ra.y, ra.x) < Atan2(rb.y, rb.x);
                           });
    muliAssert(sorted[0] == bottom);

    // Discard overlapped bottom vertices
    uint32 i = 0;
    while (i < sorted.size())
    {
        Vec2& v0 = sorted[i];
        Vec2& v1 = sorted[(i + 1) % sorted.size()];

        if (v0 == v1)
        {
            i++;
        }
        else
        {
            break;
        }
    }

    std::vector<Vec2> s;
    s.push_back(sorted[i++]);
    s.push_back(sorted[i++]);

    while (i < sorted.size())
    {
        Vec2& v = sorted[i];
        size_t l = s.size();

        if (v == s[l - 1])
        {
            ++i;
            continue;
        }

        Vec2 d1 = s[l - 1] - s[l - 2];
        Vec2 d2 = v - s[l - 1];

        if (Cross(d1, d2) <= 0)
        {
            s.pop_back();

            if (l < 3)
            {
                s.push_back(v);
                break;
            }
        }
        else
        {
            s.push_back(v);
            ++i;
        }
    }

    return s;
}

float ComputePolygonInertia(const Vec2* vertices, int32 vertexCount, float mass)
{
    float numerator = 0.0f;
    float denominator = 0.0f;

    for (int32 i = 0; i < vertexCount; i++)
    {
        int32 j = (i + 1) % vertexCount;

        const Vec2& vi = vertices[i];
        const Vec2& vj = vertices[j];

        float crs = Abs(Cross(vj, vi));

        numerator += crs * (Dot(vj, vj) + Dot(vj, vi) + Dot(vi, vi));
        denominator += crs;
    }

    return (numerator * mass) / (denominator * 6.0f);
}

float ComputeCapsuleInertia(float length, float radius, float mass)
{
    float width = length;
    float height = radius * 2.0f;

    float rectArea = width * height;
    float circleArea = MULI_PI * radius * radius;
    float totalArea = rectArea + circleArea;

    float rectInertia = (width * width + height * height) / 12.0f;
    float halfCircleInertia = ((MULI_PI / 4) - 8.0f / (9.0f * MULI_PI)) * radius * radius * radius * radius;

    float dist2 = length * 0.5f + (4.0f * radius) / (MULI_PI * 3.0f);
    dist2 *= dist2;

    // Parallel axis theorem applied
    return mass * (rectInertia * rectArea + (halfCircleInertia + (circleArea * 0.5f) * dist2) * 2.0f) / (rectArea + circleArea);
}

} // namespace muli