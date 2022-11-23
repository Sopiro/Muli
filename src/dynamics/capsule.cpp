#include "muli/capsule.h"

namespace muli
{

Capsule::Capsule(const Vec2& p1, const Vec2& p2, float _radius, Type _type, bool _resetPosition, float _density)
    : RigidBody(_type, Shape::capsule)
{
    radius = _radius;
    Vec2 a2b = p2 - p1;
    length = a2b.Length();
    area = length * radius * 2.0f + MULI_PI * radius * radius;

    if (type == RigidBody::Type::dynamic_body)
    {
        muliAssert(_density > 0);

        density = _density;
        mass = _density * area;
        invMass = 1.0f / mass;
        inertia = ComputeCapsuleInertia(this);
        invInertia = 1.0f / inertia;
    }

    Vec2 center = (p1 + p2) * 0.5f;

    va = p1 - center;
    vb = p2 - center;

    if (_resetPosition == false)
    {
        transform.position = center;
    }
}

Capsule::Capsule(float _length, float _radius, bool _horizontal, Type _type, float _density)
    : RigidBody(_type, Shape::capsule)
    , length{ _length }
{
    radius = _radius;
    area = length * radius * 2.0f + MULI_PI * radius * radius;

    if (type == RigidBody::Type::dynamic_body)
    {
        muliAssert(_density > 0);

        density = _density;
        mass = _density * area;
        invMass = 1.0f / mass;
        inertia = ComputeCapsuleInertia(this);
        invInertia = 1.0f / inertia;
    }

    va = Vec2{ -length / 2.0f, 0.0f };
    vb = Vec2{ length / 2.0f, 0.0f };

    if (_horizontal == false)
    {
        transform.rotation = MULI_PI / 2.0f;
    }
}

void Capsule::SetMass(float _mass)
{
    muliAssert(_mass > 0);

    SetDensity(_mass / area);
}

void Capsule::SetDensity(float _density)
{
    muliAssert(density > 0);

    density = _density;
    mass = density * area;
    invMass = 1.0f / mass;

    if ((flag & flag_fixed_rotation) == 0)
    {
        inertia = ComputeCapsuleInertia(this);
        invInertia = 1.0f / inertia;
    }
    else
    {
        inertia = 0.0f;
        invInertia = 0.0f;
    }
}

inline bool Capsule::TestPoint(const Vec2& p) const
{
    Vec2 localP = MulT(transform, p);

    return SignedDistanceToLineSegment(localP, va, vb, radius) < 0.0f;
}

inline Vec2 Capsule::GetClosestPoint(const Vec2& p) const
{
    Vec2 localP = MulT(transform, p);
    UV w = ComputeWeights(va, vb, localP);

    Vec2 closest;
    Vec2 normal;
    float distance;
    if (w.v <= 0) // Region A
    {
        closest = va;
        normal = localP - va;
        distance = normal.Normalize();
    }
    else if (w.v >= 1) // Region B
    {
        closest = vb;
        normal = localP - vb;
        distance = normal.Normalize();
    }
    else // Region AB
    {
        normal.Set(0.0f, 1.0f);
        distance = Dot(localP - va, normal);

        if (Dot(normal, localP - va) < 0.0f)
        {
            normal *= -1;
            distance *= -1;
        }

        closest = localP + normal * -distance;
    }

    if (distance <= radius)
    {
        return p;
    }
    else
    {
        closest += normal * radius;
        return transform * closest;
    }
}

bool Capsule::RayCast(const RayCastInput& input, RayCastOutput* output) const
{
    Vec2 p1 = MulT(transform, input.from);
    Vec2 p2 = MulT(transform, input.to);

    Vec2 v1 = va;
    Vec2 v2 = vb;

    Vec2 normal = Cross(1.0f, (v2 - v1)).Normalized();

    if (SignedDistanceToLineSegment(p1, v1, v2, radius) <= 0.0f)
    {
        return false;
    }

    // Signed distance along noraml
    float distance = Dot(p1, normal);
    if (Abs(distance) <= radius)
    {
        Vec2 v = p1.x < 0.0f ? va : vb;

        Vec2 d = p2 - p1;
        Vec2 f = p1 - v;

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
            output->normal = transform.rotation * (f + d * t).Normalized();
            return true;
        }
        else
        {
            return false;
        }
    }

    // Translate edge along the normal
    if (distance > 0.0f)
    {
        v1 += normal * radius;
        v2 += normal * radius;
    }
    else
    {
        v1 -= normal * radius;
        v2 -= normal * radius;
    }

    // Ray casting to a line segment
    Vec2 d = p2 - p1;
    Vec2 e = v2 - v1;

    float denominator = Dot(normal, d);

    // Parallel or collinear case
    if (denominator == 0.0f)
    {
        return false;
    }

    float numerator = Dot(normal, v1 - p1);

    float t = numerator / denominator;
    if (t < 0.0f || input.maxFraction < t)
    {
        return false;
    }

    // Point on the v1-v2 line
    Vec2 q = p1 + t * d;

    float l2 = Dot(e, e);
    muliAssert(l2 > 0.0f);

    float u = Dot(q - v1, e) / l2;
    if (u < 0.0f)
    {
        // Ray cast to va circle
        Vec2 f = p1 - va;

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
        // Ray cast to vb circle
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
            output->normal = transform.rotation * (f + d * t).Normalized();
            return true;
        }
        else
        {
            return false;
        }
    }
    else // Inside the edge region
    {
        output->fraction = t;
        if (numerator > 0.0f)
        {
            output->normal = transform.rotation * -normal;
        }
        else
        {
            output->normal = transform.rotation * normal;
        }

        return true;
    }
}

} // namespace muli