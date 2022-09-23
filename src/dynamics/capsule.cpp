#include "muli/capsule.h"

namespace muli
{

Capsule::Capsule(const Vec2& p1, const Vec2& p2, float _radius, bool _resetPosition, Type _type, float _density)
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
        inertia = ComputeCapsuleInertia(length, radius, mass);
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
        inertia = ComputeCapsuleInertia(length, radius, mass);
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
    inertia = ComputeCapsuleInertia(length, radius, mass);
    invInertia = 1.0f / inertia;
}

void Capsule::SetDensity(float _density)
{
    muliAssert(density > 0);

    density = _density;
    mass = density * area;
    invMass = 1.0f / mass;
    inertia = ComputeCapsuleInertia(length, radius, mass);
    invInertia = 1.0f / inertia;
}

} // namespace muli