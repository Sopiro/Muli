#include "muli/polygon_shape.h"
#include "muli/util.h"

namespace muli
{

PolygonShape::PolygonShape(const Vec2* _vertices, int32 _vertexCount, float radius)
    : Shape(Shape::Type::polygon, radius)
{
    vertexCount = _vertexCount;

    if (vertexCount > MAX_LOCAL_POLYGON_VERTICES)
    {
        vertices = (Vec2*)malloc(vertexCount * sizeof(Vec2));
        normals = (Vec2*)malloc(vertexCount * sizeof(Vec2));
    }
    else
    {
        vertices = localVertices;
        normals = localNormals;
    }

    ComputeConvexHull(_vertices, vertexCount, vertices);

    // Traslate center to the origin

    Vec2 centeroid{ 0.0f };
    int32 i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; ++i1)
    {
        centeroid += vertices[i0];
        normals[i0] = Cross(vertices[i1] - vertices[i0], 1.0f).Normalized();

        i0 = i1;
    }
    centeroid *= 1.0f / vertexCount;

    // Compute area
    area = 0;
    vertices[0] -= centeroid;
    for (int32 i = 1; i < vertexCount; ++i)
    {
        vertices[i] -= centeroid;
        area += Cross(vertices[i - 1], vertices[i]) * 0.5f;        // inside triangle
        area += radius * (vertices[i - 1] - vertices[i]).Length(); // edge rect
    }
    area += Cross(vertices[vertexCount - 1], vertices[0]) * 0.5f;
    area += radius * (vertices[vertexCount - 1] - vertices[0]).Length();

    area += MULI_PI * radius * radius; // corner arc

    localPosition = centeroid;
}

PolygonShape::~PolygonShape()
{
    if (vertexCount > MAX_LOCAL_POLYGON_VERTICES)
    {
        free(vertices);
        free(normals);
    }
}

void PolygonShape::ComputeMass(float density, MassData* outMassData) const
{
    outMassData->mass = density * area;

    float inertia;

    // Compute polygon inertia
    float numerator = 0.0f;
    float denominator = 0.0f;

    int32 i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; ++i1)
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
    float invArea = 1.0f / area;

    i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; ++i1)
    {
        const Vec2& v0 = vertices[i0];
        const Vec2& v1 = vertices[i1];

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

        float theta = AngleBetween(n0, n1);

        float areaFraction = r2 * theta * 0.5f * invArea;
        float arcInertia = r2 * 0.5f;
        float d2 = vertices[i1].Length2();

        inertia += (arcInertia + d2) * areaFraction;

        i0 = i1;
    }

    muliAssert(inertia > 0.0f);

    outMassData->inertia = outMassData->mass * (inertia + Length2(localPosition));
    outMassData->centerOfMass = localPosition;
}

void PolygonShape::ComputeAABB(const Transform& transform, AABB* outAABB) const
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

bool PolygonShape::TestPoint(const Transform& transform, const Vec2& q) const
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

bool PolygonShape::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    Vec2 p1 = MulT(transform, input.from);
    Vec2 p2 = MulT(transform, input.to);
    Vec2 d = p2 - p1;

    // Offset for polygon skin
    float offset = (radius <= DEFAULT_RADIUS) ? 0.0f : radius;

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