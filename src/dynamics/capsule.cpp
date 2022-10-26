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

bool Capsule::RayCast(const RayCastInput& input, RayCastOutput* output) const
{
    // https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect

    float r2 = radius * radius;

    Vec2 p1 = MulT(transform, input.from);
    Vec2 p2 = MulT(transform, input.to);

    Vec2 v1 = va;
    Vec2 v2 = vb;

    Vec2 normal{ 0.0f, 1.0f };

    if (p1.y > 0.0f)
    {
        v1 += normal * radius;
        v2 += normal * radius;
    }
    else
    {
        v1 -= normal * radius;
        v2 -= normal * radius;
    }

    Vec2 r = p2 - p1;
    Vec2 s = v2 - v1;

    float denominator = Cross(r, s);

    // Parallel or collinear case
    if (denominator == 0.0f)
    {
        goto test_for_circle_1;
    }

    float numeratorT = Cross(v1 - p1, s);

    float t = numeratorT / denominator;
    if (t < 0.0f || t > 1.0f)
    {
        goto test_for_circle_1;
    }

    float numeratorU = Cross(v1 - p1, r);

    float u = numeratorU / denominator;
    if (u < 0.0f || u > 1.0f)
    {
        goto test_for_circle_1;
    }

    output->fraction = t;
    if (numeratorT > 0.0f)
    {
        output->normal = transform.rotation * normal;
    }
    else
    {
        output->normal = transform.rotation * -normal;
    }

    return true;

test_for_circle_1:

    Vec2 d = p2 - p1;
    Vec2 f = p1 - va;

    float a = Dot(d, d);
    float b = 2.0f * Dot(f, d);
    float c = Dot(f, f) - r2;

    // Quadratic equation discriminant
    float discriminant = b * b - 4 * a * c;

    if (discriminant < 0.0f)
    {
        goto test_for_circle_2;
    }

    discriminant = Sqrt(discriminant);

    t = (-b - discriminant) / (2.0f * a);
    if (t >= 0.0f && t <= 1.0f)
    {
        output->fraction = t;
        output->normal = transform.rotation * (f + d * t).Normalized();

        return true;
    }

    return false;

test_for_circle_2:

    d = p2 - p1;
    f = p1 - vb;

    a = Dot(d, d);
    b = 2.0f * Dot(f, d);
    c = Dot(f, f) - r2;

    // Quadratic equation discriminant
    discriminant = b * b - 4 * a * c;

    if (discriminant < 0.0f)
    {
        return false;
    }

    discriminant = Sqrt(discriminant);

    t = (-b - discriminant) / (2.0f * a);
    if (t >= 0.0f && t <= 1.0f)
    {
        output->fraction = t;
        output->normal = transform.rotation * (f + d * t).Normalized();

        return true;
    }

    return false;
}

} // namespace muli