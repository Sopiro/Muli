#include "muli/util.h"
#include "muli/aabb.h"
#include "muli/capsule.h"
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

float ComputePolygonInertia(const Polygon* p)
{
    const Vec2* vertices = p->GetVertices();
    int32 vertexCount = p->GetVertexCount();
    float radius = p->GetRadius();

    float inertia;

    // Compute polygon inertia
    float numerator = 0.0f;
    float denominator = 0.0f;

    int32 i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; i1++)
    {
        const Vec2& v0 = vertices[i0];
        const Vec2& v1 = vertices[i1];

        float crs = Abs(Cross(v1, v0));

        numerator += crs * (Dot(v1, v1) + Dot(v1, v0) + Dot(v0, v0));
        denominator += crs;

        i0 = i1;
    }

    inertia = (numerator) / (denominator * 6.0f);

    // Consider polygon skin(radius) inertia
    float r2 = radius * radius;
    float invArea = 1.0f / p->GetArea();

    i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; i1++)
    {
        const Vec2& v0 = vertices[i0];
        const Vec2& v1 = vertices[i1];

        Vec2 normal = v1 - v0;
        float length = normal.Normalize();
        normal = Cross(normal, 1.0f);

        Vec2 mid = (v0 + v1) * 0.5f + normal * radius * 0.5f; // Mid point of edge rect

        float areaFraction = length * radius * invArea;
        float rectInertia = (length * length + r2) / 12.0f;
        float d2 = mid.Length2();

        inertia += (rectInertia + d2) * areaFraction;

        i0 = i1;
    }

    // Consider corner arc inertia
    i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; i1++)
    {
        int32 i2 = (i1 + 1) % vertexCount;

        Vec2 n0 = (Cross(vertices[i1] - vertices[i0], 1.0f)).Normalized();
        Vec2 n1 = (Cross(vertices[i2] - vertices[i1], 1.0f)).Normalized();

        float theta = AngleBetween(n0, n1);

        float areaFraction = r2 * theta * 0.5f * invArea;
        float arcInertia = r2 * 0.5f;
        float d2 = vertices[i1].Length2();

        inertia += (arcInertia + d2) * areaFraction;

        i0 = i1;
    }

    muliAssert(inertia > 0.0f);

    return inertia * p->GetMass();
}

float ComputeCapsuleInertia(const Capsule* c)
{
    float length = c->GetLength();
    float radius = c->GetRadius();
    float height = radius * 2.0f;
    float invArea = 1.0f / c->GetArea();

    float inertia;

    float rectArea = length * height;
    float rectInertia = (length * length + height * height) / 12.0f;

    inertia = rectInertia * rectArea * invArea;

    float circleArea = MULI_PI * radius * radius;
    float halfCircleInertia = ((MULI_PI / 4) - 8.0f / (9.0f * MULI_PI)) * radius * radius * radius * radius;
    float dist2 = length * 0.5f + (4.0f * radius) / (MULI_PI * 3.0f);
    dist2 *= dist2;

    inertia += (halfCircleInertia + (circleArea * 0.5f) * dist2) * 2.0f * invArea;

    // Parallel axis theorem applied
    return c->GetMass() * inertia;
}

// https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
float RayCastCircle(const Vec2& position, float radius, const Vec2& p1, const Vec2& p2, RayCastOutput* output)
{
    output->fraction = FLT_MAX;

    Vec2 d = p2 - p1;
    Vec2 f = p1 - position;
    float r2 = radius * radius;

    float a = Dot(d, d);
    float b = 2.0f * Dot(f, d);
    float c = Dot(f, f) - r2;

    // Quadratic equation discriminant
    float discriminant = b * b - 4 * a * c;
    if (discriminant < 0.0f)
    {
        return false;
    }

    discriminant = Sqrt(discriminant);

    float t = (-b - discriminant) / (2.0f * a);
    if (t >= 0.0f && t <= 1.0f)
    {
        output->fraction = t;
        output->normal = (f + d * t).Normalized();

        return true;
    }

    return false;
}

bool RayCastLineSegment(const Vec2& v1, const Vec2& v2, const Vec2& p1, const Vec2& p2, RayCastOutput* output)
{
    Vec2 d = p2 - p1;
    Vec2 e = v2 - v1;
    Vec2 normal = Cross(e, 1.0f);
    normal.Normalize();

    float denominator = Dot(normal, d);

    // Parallel or collinear case
    if (denominator == 0.0f)
    {
        return false;
    }

    float numeratorT = Dot(normal, v1 - p1);

    float t = numeratorT / denominator;
    if (t < 0.0f || 1.0f < t)
    {
        return false;
    }

    // Point on the v1-v2 line
    Vec2 q = p1 + t * d;

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