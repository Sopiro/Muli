#include "muli/box_shape.h"

namespace muli
{

Box::Box(float width, float height, float radius, const Transform& tf)
    : Shape(box, radius)
    , halfExtents{ width * 0.5f, height * 0.5f }
    , rotation{ tf.rotation }
{
    center = tf.position;
    area = width * height + 2.0f * radius * (width + height) + pi * radius * radius;
}

Box::Box(const Box& other, const Transform& tf)
    : Shape(box, other.radius)
    , halfExtents{ other.halfExtents }
{
    area = other.area;
    center = Mul(tf, other.center);

    Vec2 x = Mul(tf.rotation, Mul(other.rotation, Vec2{ 1.0f, 0.0f }));
    rotation = Atan2(x.y, x.x);
}

void Box::ComputeMass(float density, MassData* outMassData) const
{
    outMassData->mass = density * area;

    Vec2 vertices[4] = {
        Vec2{ -halfExtents.x, -halfExtents.y },
        Vec2{ halfExtents.x, -halfExtents.y },
        Vec2{ halfExtents.x, halfExtents.y },
        Vec2{ -halfExtents.x, halfExtents.y },
    };
    Vec2 normals[4] = {
        Vec2{ 0.0f, -1.0f },
        Vec2{ 1.0f, 0.0f },
        Vec2{ 0.0f, 1.0f },
        Vec2{ -1.0f, 0.0f },
    };

    float inertia;
    float numerator = 0.0f;
    float denominator = 0.0f;

    for (int32 i0 = 3, i1 = 0; i1 < 4; i0 = i1, ++i1)
    {
        Vec2 v0 = vertices[i0];
        Vec2 v1 = vertices[i1];

        float crs = Abs(Cross(v1, v0));

        numerator += crs * (Dot(v1, v1) + Dot(v1, v0) + Dot(v0, v0));
        denominator += crs;
    }

    inertia = numerator / (denominator * 6.0f);

    float r2 = radius * radius;
    float invArea = 1.0f / area;

    for (int32 i0 = 3, i1 = 0; i1 < 4; i0 = i1, ++i1)
    {
        Vec2 v0 = vertices[i0];
        Vec2 v1 = vertices[i1];

        Vec2 edge = v1 - v0;
        float length = edge.Normalize();
        Vec2 normal = normals[i0];

        Vec2 mid = (v0 + v1) * 0.5f + normal * radius * 0.5f;

        float areaFraction = length * radius * invArea;
        float rectInertia = (length * length + r2) / 12.0f;
        float d2 = mid.Length2();

        inertia += (rectInertia + d2) * areaFraction;
    }

    for (int32 i0 = 3, i1 = 0; i1 < 4; i0 = i1, ++i1)
    {
        Vec2 n0 = normals[i0];
        Vec2 n1 = normals[i1];
        Vec2 v1 = vertices[i1];

        float theta = AngleBetween(n0, n1);

        float areaFraction = r2 * theta * 0.5f * invArea;
        float arcInertia = r2 * 0.5f;
        float d2 = v1.Length2();

        inertia += (arcInertia + d2) * areaFraction;
    }

    outMassData->inertia = outMassData->mass * (inertia + Length2(center));
    outMassData->centerOfMass = center;
}

void Box::ComputeAABB(const Transform& transform, AABB* outAABB) const
{
    Vec2 min = Mul(transform, GetVertex(0));
    Vec2 max = min;

    for (int32 i = 1; i < 4; ++i)
    {
        Vec2 v = Mul(transform, GetVertex(i));
        min = Min(min, v);
        max = Max(max, v);
    }

    min -= radius;
    max += radius;
    outAABB->min = min;
    outAABB->max = max;
}

Edge Box::GetFeaturedEdge(const Transform& transform, const Vec2& dir) const
{
    int32 index = 0;
    float maxProjection = Dot(Mul(transform.rotation, GetNormal(0)), dir);

    for (int32 i = 1; i < 4; ++i)
    {
        float projection = Dot(Mul(transform.rotation, GetNormal(i)), dir);
        if (projection > maxProjection)
        {
            maxProjection = projection;
            index = i;
        }
    }

    Edge edge;
    edge.p1 = { Mul(transform, GetVertex(index)), index };
    edge.p2 = { Mul(transform, GetVertex((index + 1) % 4)), (index + 1) % 4 };
    edge.normal = Mul(transform.rotation, GetNormal(index));

    return edge;
}

bool Box::TestPoint(const Transform& transform, const Vec2& q) const
{
    Vec2 localQ = MulT(rotation, MulT(transform, q) - center);
    Vec2 closest = Clamp(localQ, -halfExtents, halfExtents);
    Vec2 delta = localQ - closest;

    return delta.Length2() <= radius * radius;
}

Vec2 Box::GetClosestPoint(const Transform& transform, const Vec2& q) const
{
    Vec2 localQ = MulT(rotation, MulT(transform, q) - center);
    Vec2 closest = Clamp(localQ, -halfExtents, halfExtents);
    Vec2 normal = localQ - closest;
    float distance = normal.NormalizeSafe();

    if (distance == 0.0f || distance <= radius)
    {
        return q;
    }

    closest += normal * radius;
    return Mul(transform, center + Mul(rotation, closest));
}

bool Box::RayCast(const Transform& transform, const RayCastInput& input, RayCastOutput* output) const
{
    Vec2 p1 = MulT(rotation, MulT(transform, input.from) - center);
    Vec2 p2 = MulT(rotation, MulT(transform, input.to) - center);
    Vec2 d = p2 - p1;

    float radii = radius + input.radius;
    float offset = (radii <= default_radius) ? 0.0f : radii;

    Vec2 vertices[4] = {
        Vec2{ -halfExtents.x, -halfExtents.y },
        Vec2{ halfExtents.x, -halfExtents.y },
        Vec2{ halfExtents.x, halfExtents.y },
        Vec2{ -halfExtents.x, halfExtents.y },
    };
    Vec2 normals[4] = {
        Vec2{ 0.0f, -1.0f },
        Vec2{ 1.0f, 0.0f },
        Vec2{ 0.0f, 1.0f },
        Vec2{ -1.0f, 0.0f },
    };

    float near = 0.0f;
    float far = input.maxFraction;
    int32 index = -1;

    // Ray cast against the expanded face planes first.
    for (int32 i0 = 3, i1 = 0; i1 < 4; i0 = i1, ++i1)
    {
        Vec2 normal = normals[i0];
        Vec2 v = vertices[i0] + normal * offset;

        float numerator = Dot(normal, v - p1);
        float denominator = Dot(normal, d);

        if (denominator == 0.0f)
        {
            if (numerator < 0.0f)
            {
                return false;
            }
        }
        else
        {
            if (denominator < 0.0f && numerator < near * denominator)
            {
                near = numerator / denominator;
                index = i0;
            }
            else if (denominator > 0.0f && numerator < far * denominator)
            {
                far = numerator / denominator;
            }

            if (far < near)
            {
                return false;
            }
        }
    }

    if (index < 0)
    {
        return false;
    }

    Vec2 n = normals[index];

    if (offset == 0.0f)
    {
        output->fraction = near;
        output->normal = Mul(transform.rotation, Mul(rotation, n));
        return true;
    }

    Vec2 v1 = vertices[index];
    Vec2 v2 = vertices[(index + 1) % 4];
    Vec2 e = v2 - v1;
    Vec2 q = p1 + d * near;

    // A face hit outside the edge span belongs to a rounded corner arc.
    float u = Dot(q - (v1 + n * radii), e);

    if (u < 0.0f)
    {
        Vec2 f = p1 - v1;

        float a = Dot(d, d);
        float b = 2.0f * Dot(f, d);
        float c = Dot(f, f) - radii * radii;

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
            output->normal = Mul(transform.rotation, Mul(rotation, Normalize(f + d * t)));
            return true;
        }

        return false;
    }
    else if (u > Dot(e, e))
    {
        Vec2 f = p1 - v2;

        float a = Dot(d, d);
        float b = 2.0f * Dot(f, d);
        float c = Dot(f, f) - radii * radii;

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
            output->normal = Mul(transform.rotation, Mul(rotation, Normalize(f + d * t)));
            return true;
        }

        return false;
    }

    output->fraction = near;
    output->normal = Mul(transform.rotation, Mul(rotation, n));
    return true;
}

} // namespace muli
