#include "muli/polygon.h"

namespace muli
{

Polygon::Polygon(const Vec2* _vertices, int32 _vertexCount, Type _type, bool _resetCenter, float _radius, float _density)
    : RigidBody(_type, RigidBody::Shape::polygon)
{
    radius = _radius;
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

    // Traslate center of mass to the origin

    Vec2 center{ 0.0f };
    int32 i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; ++i1)
    {
        center += vertices[i0];
        normals[i0] = Cross(vertices[i1] - vertices[i0], 1.0f).Normalized();

        i0 = i1;
    }
    center *= 1.0f / vertexCount;

    // Compute area
    area = 0;
    vertices[0] -= center;
    for (int32 i = 1; i < vertexCount; ++i)
    {
        vertices[i] -= center;
        area += Cross(vertices[i - 1], vertices[i]) * 0.5f;        // inside triangle
        area += radius * (vertices[i - 1] - vertices[i]).Length(); // edge rect
    }
    area += Cross(vertices[vertexCount - 1], vertices[0]) * 0.5f;
    area += radius * (vertices[vertexCount - 1] - vertices[0]).Length();

    area += MULI_PI * radius * radius; // corner arc

    if (type == dynamic_body)
    {
        muliAssert(_density > 0);

        density = _density;
        mass = _density * area;
        invMass = 1.0f / mass;
        inertia = ComputePolygonInertia(this);
        invInertia = 1.0f / inertia;
    }

    if (!_resetCenter)
    {
        Translate(center);
    }
}

Polygon::~Polygon()
{
    if (vertexCount > MAX_LOCAL_POLYGON_VERTICES)
    {
        free(vertices);
        free(normals);
    }
}

void Polygon::SetMass(float _mass)
{
    muliAssert(_mass > 0);

    SetDensity(_mass / area);
}

void Polygon::SetDensity(float _density)
{
    muliAssert(_density > 0);

    density = _density;
    mass = _density * area;
    invMass = 1.0f / mass;

    if ((flag & flag_fixed_rotation) == 0)
    {
        inertia = ComputePolygonInertia(this);
        invInertia = 1.0f / inertia;
    }
    else
    {
        inertia = 0.0f;
        invInertia = 0.0f;
    }
}

AABB Polygon::GetAABB() const
{
    Vec2 min = transform * vertices[0];
    Vec2 max = min;

    for (int32 i = 1; i < vertexCount; ++i)
    {
        Vec2 v = transform * vertices[i];

        min = Min(min, v);
        max = Max(max, v);
    }

    min -= radius;
    max += radius;

    return AABB{ min, max };
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

Edge Polygon::GetFeaturedEdge(const Vec2& dir) const
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

bool Polygon::TestPoint(const Vec2& p) const
{
    Vec2 localP = MulT(transform, p);

    int32 index = 0;
    float maxSeparation = -FLT_MAX;

    int32 i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; ++i1)
    {
        Vec2 n0 = normals[i0];
        float separation = Dot(n0, localP - vertices[i0]);
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

    Vec2 d = localP - v0;
    Vec2 e = v1 - v0;

    float w = Dot(d, e);
    float l2 = Dot(e, e);

    if (w <= 0.0f)
    {
        return Dot(d, d) < radius * radius;
    }
    else if (w >= l2)
    {
        Vec2 d1 = localP - v1;
        return Dot(d1, d1) < radius * radius;
    }
    else
    {
        return true;
    }
}

Vec2 Polygon::GetClosestPoint(const Vec2& p) const
{
    Vec2 localP = MulT(transform, p);

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

        w = ComputeWeights(v0, v1, localP);
        if (w.v <= 0) // Region v0
        {
            if (dir > 0)
            {
                normal = (localP - v0).Normalized();
                float distance = Dot(localP - v0, normal);
                if (distance > radius)
                {
                    return transform * (v0 + normal * radius);
                }
                else
                {
                    return p;
                }
            }

            dir = -1;
            i0 = (i0 - 1 + vertexCount) % vertexCount;
        }
        else if (w.v >= 1) // Region v1
        {
            if (dir < 0)
            {
                normal = (localP - v1).Normalized();
                float distance = Dot(localP - v1, normal);
                if (distance > radius)
                {
                    return transform * (v1 + normal * radius);
                }
                else
                {
                    return p;
                }
            }

            dir = 1;
            i0 = (i0 + 1) % vertexCount;
        }
        else // Inside the region
        {
            normal = normals[i0];
            float distance = Dot(localP - v0, normal);
            if (distance > radius)
            {
                Vec2 closest = localP + normal * (radius - distance);
                return transform * closest;
            }

            if (dir != 0)
            {
                return p;
            }

            ++insideCheck;
            dir = 0;
            i0 = (i0 + 1) % vertexCount;
        }
    }

    return p;
}

Edge Polygon::GetIntersectingEdge(const Vec2& dir) const
{
    Vec2 localDir = MulT(transform.rotation, dir);

    int32 i0 = vertexCount - 1;
    for (int32 i1 = 0; i1 < vertexCount; ++i1)
    {
        const Vec2& v0 = vertices[i0];
        const Vec2& v1 = vertices[i1];

        if (Cross(v0, localDir) > 0 && Cross(v1, localDir) < 0)
        {
            return Edge{ transform * v0, transform * v1, i0, i1 };
        }

        i0 = i1;
    }

    throw std::runtime_error("Unreachable");
}

bool Polygon::RayCast(const RayCastInput& input, RayCastOutput* output) const
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