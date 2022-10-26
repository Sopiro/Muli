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
    Vec2 p2 = MulT(transform, input.to);

    Vec2 v1 = va;
    Vec2 v2 = vb;

    if (SignedDistanceToLineSegment(p1, v1, v2, radius) < 0.0f)
    {
        return false;
    }

    if (Abs(p1.y) <= radius)
    {
        goto circle_test;
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

    bool hit = RayCastLineSegment(v1, v2, p1, p2, output);
    if (hit)
    {
        output->normal = transform.rotation * output->normal;
        return true;
    }

    if (radius <= 0.0f)
    {
        return false;
    }

circle_test:
    RayCastOutput co1;
    RayCastOutput co2;
    bool r1 = RayCastCircle(va, radius, p1, p2, &co1);
    bool r2 = RayCastCircle(vb, radius, p1, p2, &co2);

    if (!r1 && !r2)
    {
        return false;
    }

    if (co1.fraction < co2.fraction)
    {
        memcpy(output, &co1, sizeof(RayCastOutput));
    }
    else
    {
        memcpy(output, &co2, sizeof(RayCastOutput));
    }

    output->normal = transform.rotation * output->normal;

    return true;
}

} // namespace muli