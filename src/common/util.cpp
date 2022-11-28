#include "muli/util.h"
#include "muli/aabb.h"

namespace muli
{

void ComputeConvexHull(const Vec2* vertices, int32 vertexCount, Vec2* outVertices, int32* outVertexCount)
{
    if (vertexCount < 3)
    {
        memcpy(outVertices, vertices, vertexCount);
        *outVertexCount = vertexCount;
        return;
    }

    // Find the lowest vertex
    int32 index = 0;
    for (int32 i = 1; i < vertexCount; ++i)
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

                               float angleA = Atan2(ra.y, ra.x);
                               float angleB = Atan2(rb.y, rb.x);

                               if (angleA == angleB)
                               {
                                   return Dist2(bottom, a) < Dist2(bottom, b);
                               }
                               else
                               {
                                   return angleA < angleB;
                               }
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
            ++i;
        }
        else
        {
            break;
        }
    }

    int32 sp = 0; // stack pointer
    *outVertexCount = vertexCount - i;
    outVertices[sp++] = sorted[i++];
    outVertices[sp++] = sorted[i++];

    while (i < vertexCount)
    {
        Vec2& v = sorted[i];
        int32 l = sp;

        if (v == outVertices[l - 1])
        {
            ++i;
            continue;
        }

        Vec2 d1 = outVertices[l - 1] - outVertices[l - 2];
        Vec2 d2 = v - outVertices[l - 1];

        if (Cross(d1, d2) <= 0)
        {
            --sp;

            if (l < 3)
            {
                outVertices[sp++] = v;
                break;
            }
        }
        else
        {
            outVertices[sp++] = v;
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
    for (uint32 i = 1; i < vertices.size(); ++i)
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

                               float angleA = Atan2(ra.y, ra.x);
                               float angleB = Atan2(rb.y, rb.x);

                               if (angleA == angleB)
                               {
                                   return Dist2(bottom, a) < Dist2(bottom, b);
                               }
                               else
                               {
                                   return angleA < angleB;
                               }
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

// https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
float RayCastCircle(const Vec2& position, float radius, const RayCastInput& input, RayCastOutput* output)
{
    Vec2 d = input.to - input.from;
    Vec2 f = input.from - position;
    float r2 = radius * radius;

    float a = Dot(d, d);
    float b = 2.0f * Dot(f, d);
    float c = Dot(f, f) - r2;

    // Quadratic equation discriminant
    float discriminant = b * b - 4.0f * a * c;

    if (discriminant < 0.0f)
    {
        return false;
    }

    discriminant = Sqrt(discriminant);

    float t = (-b - discriminant) / (2.0f * a);
    if (0.0f <= t && t <= input.maxFraction)
    {
        output->fraction = t;
        output->normal = (f + d * t).Normalized();

        return true;
    }

    return false;
}

bool RayCastLineSegment(const Vec2& v1, const Vec2& v2, const RayCastInput& input, RayCastOutput* output)
{
    Vec2 d = input.from - input.to;
    Vec2 e = v2 - v1;
    Vec2 normal = Cross(e, 1.0f);
    normal.Normalize();

    float denominator = Dot(normal, d);

    // Parallel or collinear case
    if (denominator == 0.0f)
    {
        return false;
    }

    float numeratorT = Dot(normal, v1 - input.from);

    float t = numeratorT / denominator;
    if (t < 0.0f || 1.0f < t)
    {
        return false;
    }

    // Point on the v1-v2 line
    Vec2 q = input.from + t * d;

    float u = Dot(q - v1, e) / Dot(e, e);
    if (u < 0.0f || 1.0f < u)
    {
        return false;
    }

    output->fraction = t;
    if (numeratorT > 0.0f)
    {
        output->normal = -normal;
    }
    else
    {
        output->normal = normal;
    }

    return true;
}

} // namespace muli