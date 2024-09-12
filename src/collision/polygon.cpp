#include "muli/polygon.h"
#include "muli/geometry.h"

namespace muli
{

Polygon::Polygon(const Vec2* inVertices, int32 inVertexCount, bool resetPosition, float radius)
    : Shape(polygon, radius)
{
    if (inVertexCount > max_local_polygon_vertices)
    {
        vertices = (Vec2*)muli::Alloc(inVertexCount * sizeof(Vec2));
        normals = (Vec2*)muli::Alloc(inVertexCount * sizeof(Vec2));
    }
    else
    {
        vertices = localVertices;
        normals = localNormals;
    }

    ComputeConvexHull(inVertices, inVertexCount, vertices, &vertexCount);

    int32 i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; ++i1)
    {
        center += vertices[i0];
        normals[i0] = Normalize(Cross(vertices[i1] - vertices[i0], 1.0f));

        i0 = i1;
    }
    center *= 1.0f / vertexCount;

    // Compute area
    area = 0.0f;
    for (int32 i = 1; i < vertexCount; ++i)
    {
        Vec2 v0 = vertices[i - 1] - center;
        Vec2 v1 = vertices[i] - center;
        area += Cross(v0, v1) * 0.5f;        // internal triangle
        area += radius * (v0 - v1).Length(); // edge rect
    }
    Vec2 v0 = vertices[vertexCount - 1] - center;
    Vec2 v1 = vertices[0] - center;
    area += Cross(v0, v1) * 0.5f;
    area += radius * (v0 - v1).Length();

    area += pi * radius * radius; // corner arc

    if (resetPosition)
    {
        for (int32 i = 0; i < vertexCount; ++i)
        {
            vertices[i] -= center;
        }
        center.SetZero();
    }
}

Polygon::Polygon(std::initializer_list<Vec2> vertices, bool resetPosition, float radius)
    : Polygon(vertices.begin(), int32(vertices.size()), resetPosition, radius)
{
}

Polygon::Polygon(float width, float height, float radius, const Vec2& position, float angle)
    : Shape(polygon, radius)
{
    vertices = localVertices;
    normals = localNormals;
    vertexCount = 4;

    float hx = width * 0.5f;
    float hy = height * 0.5f;

    Transform t{ position, angle };

    vertices[0] = Mul(t, Vec2{ -hx, -hy });
    vertices[1] = Mul(t, Vec2{ hx, -hy });
    vertices[2] = Mul(t, Vec2{ hx, hy });
    vertices[3] = Mul(t, Vec2{ -hx, hy });

    normals[0] = Mul(t.rotation, Vec2{ 0.0f, -1.0f });
    normals[1] = Mul(t.rotation, Vec2{ 1.0f, 0.0f });
    normals[2] = Mul(t.rotation, Vec2{ 0.0f, 1.0f });
    normals[3] = Mul(t.rotation, Vec2{ -1.0f, 0.0f });

    center = position;
    area = width * height;
}

Polygon::Polygon(float size, float radius, const Vec2& position, float angle)
    : Polygon(size, size, radius, position, angle)
{
}

Polygon::~Polygon()
{
    if (vertices != localVertices)
    {
        muli::Free(vertices);
        muli::Free(normals);
    }
}

Polygon::Polygon(const Polygon& other)
    : Polygon(other.vertices, other.vertexCount, false, other.radius)
{
}

Shape* Polygon::Clone(Allocator* allocator) const
{
    void* mem = allocator->Allocate(sizeof(Polygon));
    Polygon* shape = new (mem) Polygon(vertices, vertexCount, false, radius);
    return shape;
}

Vec2 Polygon::GetClosestPoint(const Transform& transform, const Vec2& q) const
{
    Vec2 localQ = MulT(transform, q);

    Vec2 normal;

    int32 dir = 0;
    int32 i0 = 0;
    int32 insideCheck = 0;

    while (insideCheck < vertexCount)
    {
        int32 i1 = (i0 + 1) % vertexCount;

        const Vec2& v0 = vertices[i0];
        const Vec2& v1 = vertices[i1];

        float u = Dot(localQ - v1, v0 - v1);
        float v = Dot(localQ - v0, v1 - v0);

        if (v <= 0.0f) // Region v0
        {
            if (dir > 0)
            {
                normal = Normalize(localQ - v0);
                float distance = Dot(localQ - v0, normal);
                if (distance > radius)
                {
                    return Mul(transform, v0 + normal * radius);
                }
                else
                {
                    return q;
                }
            }

            dir = -1;
            // Go clock wise!
            i0 = (i0 - 1 + vertexCount) % vertexCount;
        }
        else if (u <= 0.0f) // Region v1
        {
            if (dir < 0)
            {
                normal = Normalize(localQ - v1);
                float distance = Dot(localQ - v1, normal);
                if (distance > radius)
                {
                    return Mul(transform, v1 + normal * radius);
                }
                else
                {
                    return q;
                }
            }

            dir = 1;
            // Go counter-clock wise!
            i0 = (i0 + 1) % vertexCount;
        }
        else // Inside the edge
        {
            normal = normals[i0];
            float distance = Dot(localQ - v0, normal);
            if (distance > radius)
            {
                Vec2 closest = localQ + normal * (radius - distance);
                return Mul(transform, closest);
            }

            if (dir != 0)
            {
                return q;
            }

            ++insideCheck;
            dir = 0;
            i0 = (i0 + 1) % vertexCount;
        }
    }

    return q;
}

// Returns the farthest edge along dir
Edge Polygon::GetFeaturedEdge(const Transform& transform, const Vec2& dir) const
{
    Vec2 localDir = MulT(transform.rotation, dir);

    int32 index = GetSupport(localDir);
    Vec2 curr = vertices[index];

    int32 prevIndex = (index - 1 + vertexCount) % vertexCount;
    int32 nextIndex = (index + 1) % vertexCount;

    Vec2 prev = vertices[prevIndex];
    Vec2 next = vertices[nextIndex];

    Vec2 e1 = curr - prev;
    Vec2 e2 = curr - next;

    if (Dot(e1, localDir) <= Dot(e2, localDir))
    {
        return Edge{ Mul(transform, prev), Mul(transform, curr), prevIndex, index };
    }
    else
    {
        return Edge{ Mul(transform, curr), Mul(transform, next), index, nextIndex };
    }
}

int32 Polygon::GetSupport(const Vec2& localDir) const
{
    int32 index = 0;
    float maxValue = Dot(localDir, vertices[0]);

    for (int32 i = 1; i < vertexCount; ++i)
    {
        float value = Dot(localDir, vertices[i]);
        if (value > maxValue)
        {
            index = i;
            maxValue = value;
        }
    }

    return index;
}

void Polygon::ComputeMass(float density, MassData* outMassData) const
{
    outMassData->mass = density * area;

    float inertia;

    // Compute polygon inertia
    float numerator = 0.0f;
    float denominator = 0.0f;

    int32 i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; ++i1)
    {
        Vec2 v0 = vertices[i0] - center;
        Vec2 v1 = vertices[i1] - center;

        float crs = Abs(Cross(v1, v0));

        numerator += crs * (Dot(v1, v1) + Dot(v1, v0) + Dot(v0, v0));
        denominator += crs;

        i0 = i1;
    }

    inertia = (numerator) / (denominator * 6.0f);

    // Consider polygon skin(radius) inertia
    float r2 = radius * radius;
    float invArea = 1.0f / area;

    i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; ++i1)
    {
        Vec2 v0 = vertices[i0] - center;
        Vec2 v1 = vertices[i1] - center;

        Vec2 edge = v1 - v0;
        float length = edge.Normalize();
        Vec2 normal = Cross(edge, 1.0f);

        Vec2 mid = (v0 + v1) * 0.5f + normal * radius * 0.5f; // Mid point of edge rect

        float areaFraction = length * radius * invArea;
        float rectInertia = (length * length + r2) / 12.0f;
        float d2 = mid.Length2();

        inertia += (rectInertia + d2) * areaFraction;

        i0 = i1;
    }

    // Consider corner arc inertia
    i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; ++i1)
    {
        Vec2 n0 = normals[i0];
        Vec2 n1 = normals[i1];
        Vec2 v1 = vertices[i1] - center;

        float theta = AngleBetween(n0, n1);

        float areaFraction = r2 * theta * 0.5f * invArea;
        float arcInertia = r2 * 0.5f;
        float d2 = v1.Length2();

        inertia += (arcInertia + d2) * areaFraction;

        i0 = i1;
    }

    MuliAssert(inertia > 0.0f);

    outMassData->inertia = outMassData->mass * (inertia + Length2(center));
    outMassData->centerOfMass = center;
}

void Polygon::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    Vec2 min = Mul(transform, vertices[0]);
    Vec2 max = min;

    for (int32 i = 1; i < vertexCount; ++i)
    {
        Vec2 v = Mul(transform, vertices[i]);

        min = Min(min, v);
        max = Max(max, v);
    }

    min -= radius;
    max += radius;
    outAABB->min = min;
    outAABB->max = max;
}

bool Polygon::TestPoint(const Transform& transform, const Vec2& q) const
{
    Vec2 localQ = MulT(transform, q);

    int32 index = 0;
    float maxSeparation = -max_value;

    int32 i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; ++i1)
    {
        Vec2 n0 = normals[i0];
        float separation = Dot(n0, localQ - vertices[i0]);
        if (separation > radius)
        {
            return false;
        }

        if (separation > maxSeparation)
        {
            maxSeparation = separation;
            index = i0;
        }

        i0 = i1;
    }

    // Totally inside
    if (maxSeparation < 0.0f)
    {
        return true;
    }

    // Test for polygon skin(radius)
    Vec2 v0 = vertices[index];
    Vec2 v1 = vertices[(index + 1) % vertexCount];

    Vec2 d = localQ - v0;
    Vec2 e = v1 - v0;

    float w = Dot(d, e);
    float l2 = Dot(e, e);

    if (w <= 0.0f)
    {
        return Dot(d, d) < radius * radius;
    }
    else if (w >= l2)
    {
        Vec2 d1 = localQ - v1;
        return Dot(d1, d1) < radius * radius;
    }
    else
    {
        return true;
    }
}

bool Polygon::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    Vec2 p1 = MulT(transform, input.from);
    Vec2 p2 = MulT(transform, input.to);
    Vec2 d = p2 - p1;

    // Offset for polygon skin
    float radii = radius + input.radius;
    float offset = (radii <= default_radius) ? 0.0f : radii;

    float near = 0.0f;
    float far = input.maxFraction;

    int32 index = -1;

    int32 i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; ++i1)
    {
        Vec2 normal = normals[i0];
        Vec2 v = vertices[i0] + normal * offset;

        float numerator = Dot(normal, v - p1);
        float denominator = Dot(normal, d);

        if (denominator == 0.0f)  // Parallel
        {
            if (numerator < 0.0f) // Non-collinear
            {
                return false;
            }
        }
        else
        {
            if (denominator < 0.0f && numerator < near * denominator)
            {
                // Increase near fraction
                near = numerator / denominator;
                index = i0;
            }
            else if (denominator > 0.0f && numerator < far * denominator)
            {
                // Decrease far fraction
                far = numerator / denominator;
            }
        }

        if (far < near)
        {
            return false;
        }

        i0 = i1;
    }

    MuliAssert(0.0f <= near && near <= input.maxFraction);

    if (index >= 0)
    {
        Vec2 n = normals[index];

        if (offset == 0.0f)
        {
            output->fraction = near;
            output->normal = Mul(transform.rotation, n);
            return true;
        }

        Vec2 v1 = vertices[index];
        Vec2 v2 = vertices[(index + 1) % vertexCount];
        Vec2 e = v2 - v1;
        Vec2 q = p1 + d * near;

        float u = Dot(q - (v1 + n * radii), e);

        if (u < 0.0f)
        {
            // Ray cast to a v1 circle
            Vec2 f = p1 - v1;

            float a = Dot(d, d);
            float b = 2.0f * Dot(f, d);
            float c = Dot(f, f) - radii * radii;

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
                output->normal = Mul(transform.rotation, Normalize(f + d * t));
                return true;
            }
            else
            {
                return false;
            }
        }
        else if (u > Dot(e, e))
        {
            // Ray cast to a v2 circle
            Vec2 f = p1 - v2;

            float a = Dot(d, d);
            float b = 2.0f * Dot(f, d);
            float c = Dot(f, f) - radii * radii;

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
                output->normal = Mul(transform.rotation, Normalize(f + d * t));
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            output->fraction = near;
            output->normal = Mul(transform.rotation, n);
            return true;
        }
    }

    return false;
}

} // namespace muli