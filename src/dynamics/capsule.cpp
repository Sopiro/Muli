#include "muli/capsule.h"

namespace muli
{

Capsule::Capsule(const Vec2& p1, const Vec2& p2, float _radius, Type _type, bool _resetPosition, float _density)
    : RigidBody(_type, Shape::ShapeCapsule)
{
    radius = _radius;
    Vec2 a2b = p2 - p1;
    length = a2b.Length();
    area = length * radius * 2.0f + MULI_PI * radius * radius;

    if (type == RigidBody::Type::Dynamic)
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

    if (_resetPosition == false)
    {
        Vec2 mid = (p2 + p1) * 0.5f;

        transform.position = mid;
        transform.rotation = Atan2(a2b.y, a2b.x);
    }
}

Capsule::Capsule(float _length, float _radius, bool _horizontal, Type _type, float _density)
    : RigidBody(_type, Shape::ShapeCapsule)
    , length{ _length }
{
    radius = _radius;
    area = length * radius * 2 + MULI_PI * radius * radius;

    if (type == RigidBody::Type::Dynamic)
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

    density = _mass / area;
    mass = _mass;
    invMass = 1.0f / mass;
    inertia = ComputeCapsuleInertia(this);
    invInertia = 1.0f / inertia;
}

void Capsule::SetDensity(float _density)
{
    muliAssert(density > 0);

    density = _density;
    mass = density * area;
    invMass = 1.0f / mass;
    inertia = ComputeCapsuleInertia(this);
    invInertia = 1.0f / inertia;
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
    Vec2 p2 = MulT(transform, input.from + (input.to - input.from) * input.maxFraction);

    Vec2 v1 = va;
    Vec2 v2 = vb;

    if (SignedDistanceToLineSegment(p1, v1, v2, radius) <= 0.0f)
    {
        return false;
    }

    if (Abs(p1.y) <= radius)
    {
        Vec2 c = p1.x < 0.0f ? va : vb;
        if (RayCastCircle(c, radius, p1, p2, output))
        {
            output->normal = transform.rotation * output->normal;
            return true;
        }
        else
        {
            return false;
        }
    }

    // Translate edge along the normal
    if (p1.y > 0.0f)
    {
        v1.y += radius;
        v2.y += radius;
    }
    else
    {
        v1.y -= radius;
        v2.y -= radius;
    }

    // Ray casting to a line segment
    Vec2 d = p2 - p1;
    Vec2 e = v2 - v1;
    Vec2 normal{ 0.0f, 1.0f };

    float denominator = Dot(normal, d);

    // Parallel or collinear case
    if (denominator == 0.0f)
    {
        return false;
    }

    float numerator = Dot(normal, v1 - p1);

    float t = numerator / denominator;
    if (t < 0.0f || 1.0f < t)
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
        if (RayCastCircle(va, radius, p1, p2, output))
        {
            output->normal = transform.rotation * output->normal;
            return true;
        }
        else
        {
            return false;
        }
    }
    else if (u > 1.0f)
    {
        if (RayCastCircle(vb, radius, p1, p2, output))
        {
            output->normal = transform.rotation * output->normal;
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