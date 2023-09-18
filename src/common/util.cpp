#include "muli/util.h"
#include "muli/aabb.h"

namespace muli
{

void ComputeConvexHull(const Vec2* vertices, int32 vertexCount, Vec2* outVertices, int32* outVertexCount)
{
    if (vertexCount < 3)
    {
        memcpy(outVertices, vertices, vertexCount * sizeof(Vec2));
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

    Vec2* sorted = new Vec2[vertexCount];
    memcpy(sorted, vertices, vertexCount * sizeof(Vec2));
    std::swap(sorted[index], sorted[0]);

    // Sort the vertices based on the angle related to the lowest vertex.
    std::sort(sorted + 1, sorted + vertexCount, [&](const Vec2& a, const Vec2& b) -> bool {
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

    // Initialize stack
    int32 sp = 0;
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

    delete[] sorted;
}

std::vector<Vec2> ComputeConvexHull(const std::vector<Vec2>& vertices)
{
    if (vertices.size() < 3)
    {
        return vertices;
    }

    size_t index = 0;
    for (size_t i = 1; i < vertices.size(); ++i)
    {
        if (vertices[i].y < vertices[index].y)
        {
            index = i;
        }
    }
    Vec2 bottom = vertices[index];

    std::vector<Vec2> sorted = vertices;
    std::swap(sorted[index], sorted[0]);

    std::sort(sorted.begin() + 1, sorted.end(), [&](const Vec2& a, const Vec2& b) -> bool {
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

    // Discard overlapped bottom vertices
    size_t i = 0;
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
bool RayCastCircle(const Vec2& p, float r, const RayCastInput& input, RayCastOutput* output)
{
    Vec2 d = input.to - input.from;
    Vec2 f = input.from - p;
    float r2 = r * r;

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
        output->normal = Normalize(f + d * t);

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

    float u = Dot(q - v1, e);
    if (u < 0.0f || Dot(e, e) < u)
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

bool RayCastCapsule(const Vec2& va, const Vec2& vb, float radius, const RayCastInput& input, RayCastOutput* output)
{
    Vec2 p1 = input.from;
    Vec2 p2 = input.to;

    Vec2 v1 = va;
    Vec2 v2 = vb;

    Vec2 d = p2 - p1;
    Vec2 e = v2 - v1;
    Vec2 n = Cross(1.0f, e);
    n.Normalize();

    Vec2 pv = p1 - v1;

    // Signed distance along normal
    float distance = Dot(pv, n);

    // Does the ray start within the capsule band?
    if (Abs(distance) <= radius)
    {
        float r = Dot(e, pv);

        // Raycast to va circle
        if (r < 0.0)
        {
            // Ray cast to va circle
            Vec2 f = p1 - va;

            float a = Dot(d, d);
            float b = 2.0f * Dot(f, d);
            float c = Dot(f, f) - radius * radius;

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
                output->normal = Normalize(f + d * t);
                return true;
            }
            else
            {
                return false;
            }
        }

        // Raycast to vb circle
        if (r > Dot(e, e))
        {
            // Raycast to vb circle
            Vec2 f = p1 - vb;

            float a = Dot(d, d);
            float b = 2.0f * Dot(f, d);
            float c = Dot(f, f) - radius * radius;

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
                output->normal = Normalize(f + d * t);
                return true;
            }
            else
            {
                return false;
            }
        }

        // Totally inside the capsule
        return false;
    }

    Vec2 rn = n * radius;

    // Translate edge along normal
    if (distance > 0.0f)
    {
        v1 += rn;
        v2 += rn;
    }
    else
    {
        v1 -= rn;
        v2 -= rn;
    }

    // Raycast to a line segment

    float denominator = Dot(n, d);

    // Parallel or collinear case
    if (denominator == 0.0f)
    {
        return false;
    }

    float numerator = Dot(n, v1 - p1);

    float t = numerator / denominator;
    if (t < 0.0f || input.maxFraction < t)
    {
        return false;
    }

    // Point on the v1-v2 line
    Vec2 q = p1 + t * d;

    float u = Dot(q - v1, e);
    if (u < 0.0f)
    {
        // Ray cast to va circle
        Vec2 f = p1 - va;

        float a = Dot(d, d);
        float b = 2.0f * Dot(f, d);
        float c = Dot(f, f) - radius * radius;

        // Quadratic equation discriminant
        float discriminant = b * b - 4.0f * a * c;

        if (discriminant < 0.0f)
        {
            return false;
        }

        discriminant = Sqrt(discriminant);

        t = (-b - discriminant) / (2.0f * a);
        if (0.0f <= t && t <= input.maxFraction)
        {
            output->fraction = t;
            output->normal = Normalize(f + d * t);
            return true;
        }
        else
        {
            return false;
        }
    }

    if (Dot(e, e) < u)
    {
        // Raycast to vb circle
        Vec2 f = p1 - vb;

        float a = Dot(d, d);
        float b = 2.0f * Dot(f, d);
        float c = Dot(f, f) - radius * radius;

        // Quadratic equation discriminant
        float discriminant = b * b - 4.0f * a * c;

        if (discriminant < 0.0f)
        {
            return false;
        }

        discriminant = Sqrt(discriminant);

        t = (-b - discriminant) / (2.0f * a);
        if (0.0f <= t && t <= input.maxFraction)
        {
            output->fraction = t;
            output->normal = Normalize(f + d * t);
            return true;
        }
        else
        {
            return false;
        }
    }

    // Inside the edge region
    output->fraction = t;
    if (numerator > 0.0f)
    {
        output->normal = -n;
    }
    else
    {
        output->normal = n;
    }

    return true;
}

} // namespace muli