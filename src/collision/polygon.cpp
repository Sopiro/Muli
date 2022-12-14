#include "muli/polygon.h"
#include "muli/util.h"

namespace muli
{

Polygon::Polygon(const Vec2* _vertices, int32 _vertexCount, bool _resetPosition, float _radius)
    : Shape(polygon, _radius)
{
    if (_vertexCount > max_local_polygon_vertices)
    {
        vertices = (Vec2*)malloc(_vertexCount * sizeof(Vec2));
        normals = (Vec2*)malloc(_vertexCount * sizeof(Vec2));
    }
    else
    {
        vertices = localVertices;
        normals = localNormals;
    }

    ComputeConvexHull(_vertices, _vertexCount, vertices, &vertexCount);

    int32 i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; ++i1)
    {
        center += vertices[i0];
        normals[i0] = Cross(vertices[i1] - vertices[i0], 1.0f).Normalized();

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

    area += MULI_PI * radius * radius; // corner arc

    if (_resetPosition)
    {
        for (int32 i = 0; i < vertexCount; ++i)
        {
            vertices[i] -= center;
        }
        center.SetZero();
    }
}

Polygon::Polygon(std::initializer_list<Vec2> vertices, bool resetPosition, float radius)
    : Polygon(vertices.begin(), static_cast<int32>(vertices.size()), resetPosition, radius)
{
}

Polygon::Polygon(float width, float height, float _radius, const Vec2& position, float angle)
    : Shape(polygon, _radius)
{
    vertices = localVertices;
    normals = localNormals;
    vertexCount = 4;

    float hx = width / 2.0f;
    float hy = height / 2.0f;

    Transform t{ position, angle };

    vertices[0] = t * Vec2{ -hx, -hy };
    vertices[1] = t * Vec2{ hx, -hy };
    vertices[2] = t * Vec2{ hx, hy };
    vertices[3] = t * Vec2{ -hx, hy };

    normals[0] = t.rotation * Vec2{ 0.0f, -1.0f };
    normals[1] = t.rotation * Vec2{ 1.0f, 0.0f };
    normals[2] = t.rotation * Vec2{ 0.0f, 1.0f };
    normals[3] = t.rotation * Vec2{ -1.0f, 0.0f };

    center = position;
    area = width * height;
}

Polygon::Polygon(float size, float _radius, const Vec2& position, float angle)
    : Polygon(size, size, _radius, position, angle)
{
}

Polygon::~Polygon()
{
    if (vertices != localVertices)
    {
        free(vertices);
        free(normals);
    }
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

    UV w;
    Vec2 normal;

    int32 dir = 0;
    int32 i0 = 0;
    int32 insideCheck = 0;

    while (insideCheck < vertexCount)
    {
        int32 i1 = (i0 + 1) % vertexCount;

        const Vec2& v0 = vertices[i0];
        const Vec2& v1 = vertices[i1];

        w = ComputeWeights(v0, v1, localQ);
        if (w.v <= 0) // Region v0
        {
            if (dir > 0)
            {
                normal = (localQ - v0).Normalized();
                float distance = Dot(localQ - v0, normal);
                if (distance > radius)
                {
                    return transform * (v0 + normal * radius);
                }
                else
                {
                    return q;
                }
            }

            dir = -1;
            i0 = (i0 - 1 + vertexCount) % vertexCount;
        }
        else if (w.v >= 1) // Region v1
        {
            if (dir < 0)
            {
                normal = (localQ - v1).Normalized();
                float distance = Dot(localQ - v1, normal);
                if (distance > radius)
                {
                    return transform * (v1 + normal * radius);
                }
                else
                {
                    return q;
                }
            }

            dir = 1;
            i0 = (i0 + 1) % vertexCount;
        }
        else // Inside the region
        {
            normal = normals[i0];
            float distance = Dot(localQ - v0, normal);
            if (distance > radius)
            {
                Vec2 closest = localQ + normal * (radius - distance);
                return transform * closest;
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

Edge Polygon::GetFeaturedEdge(const Transform& transform, const Vec2& dir) const
{
    Vec2 localDir = MulT(transform.rotation, dir);
    ContactPoint farthest = Support(localDir);

    Vec2 curr = farthest.position;
    int32 index = farthest.id;
    int32 prevIndex = (index - 1 + vertexCount) % vertexCount;
    int32 nextIndex = (index + 1) % vertexCount;

    Vec2 prev = vertices[prevIndex];
    Vec2 next = vertices[nextIndex];

    Vec2 e1 = (curr - prev).Normalized();
    Vec2 e2 = (curr - next).Normalized();

    bool w = Dot(e1, localDir) <= Dot(e2, localDir);
    if (w)
    {
        return Edge{ transform * prev, transform * curr, prevIndex, index };
    }
    else
    {
        return Edge{ transform * curr, transform * next, index, nextIndex };
    }
}

ContactPoint Polygon::Support(const Vec2& localDir) const
{
    int32 index = 0;
    float maxValue = Dot(localDir, vertices[index]);

    for (int32 i = 1; i < vertexCount; ++i)
    {
        float value = Dot(localDir, vertices[i]);
        if (value > maxValue)
        {
            index = i;
            maxValue = value;
        }
    }

    return ContactPoint{ vertices[index], index };
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
        int32 i2 = (i1 + 1) % vertexCount;

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

    muliAssert(inertia > 0.0f);

    outMassData->inertia = outMassData->mass * (inertia + Length2(center));
    outMassData->centerOfMass = center;
}

void Polygon::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    Vec2 min = transform * vertices[0];
    Vec2 max = min;

    for (int32 i = 1; i < vertexCount; ++i)
    {
        Vec2 v = transform * vertices[i];

        min = Min(min, v);
        max = Max(max, v);
    }

    outAABB->min = min - radius;
    outAABB->max = max + radius;
}

bool Polygon::TestPoint(const Transform& transform, const Vec2& q) const
{
    Vec2 localQ = MulT(transform, q);

    int32 index = 0;
    float maxSeparation = -FLT_MAX;

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
    float offset = (radius <= default_radius) ? 0.0f : radius;

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

        if (denominator == 0.0f) // Parallel
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

    muliAssert(0.0f <= near && near <= input.maxFraction);

    if (index >= 0)
    {
        Vec2 n = normals[index];

        if (offset == 0.0f)
        {
            output->fraction = near;
            output->normal = transform.rotation * n;
            return true;
        }

        Vec2 v1 = vertices[index];
        Vec2 v2 = vertices[(index + 1) % vertexCount];
        Vec2 e = v2 - v1;
        Vec2 q = p1 + d * near;

        float u = Dot(q - (v1 + n * radius), e) / Dot(e, e);

        if (u < 0.0f)
        {
            // Ray cast to a v1 circle
            Vec2 f = p1 - v1;

            float a = Dot(d, d);
            float b = 2.0f * Dot(f, d);
            float c = Dot(f, f) - radius * radius;

            // Quadratic equation discriminant
            float discriminant = b * b - 4 * a * c;

            if (discriminant < 0.0f)
            {
                return false;
            }

            discriminant = Sqrt(discriminant);

            float t = (-b - discriminant) / (2.0f * a);
            if (0.0f <= t && t <= input.maxFraction)
            {
                output->fraction = t;
                output->normal = transform.rotation * (f + d * t).Normalized();
                return true;
            }
            else
            {
                return false;
            }
        }
        else if (u > 1.0f)
        {
            // Ray cast to a v2 circle
            Vec2 f = p1 - v2;

            float a = Dot(d, d);
            float b = 2.0f * Dot(f, d);
            float c = Dot(f, f) - radius * radius;

            // Quadratic equation discriminant
            float discriminant = b * b - 4 * a * c;

            if (discriminant < 0.0f)
            {
                return false;
            }

            discriminant = Sqrt(discriminant);

            float t = (-b - discriminant) / (2.0f * a);
            if (0.0f <= t && t <= input.maxFraction)
            {
                output->fraction = t;
                output->normal = transform.rotation * (f + d * t).Normalized();
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
            output->normal = transform.rotation * n;
            return true;
        }
    }

    return false;
}

} // namespace muli