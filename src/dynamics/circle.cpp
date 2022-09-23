#include "muli/circle.h"

namespace muli
{

Circle::Circle(float _radius, Type _type, float _density)
    : RigidBody(_type, Shape::ShapeCircle)
{
    radius = _radius;
    area = MULI_PI * radius * radius;

    if (type == RigidBody::Type::Dynamic)
    {
        muliAssert(_density > 0);

        density = _density;
        mass = _density * area;
        invMass = 1.0f / mass;
        inertia = ComputeCircleInertia(radius, mass);
        invInertia = 1.0f / inertia;
    }
}

void Circle::SetMass(float _mass)
{
    muliAssert(_mass > 0);

    density = _mass / area;
    mass = _mass;
    invMass = 1.0f / mass;
    inertia = ComputeCircleInertia(radius, mass);
    invInertia = 1.0f / inertia;
}

void Circle::SetDensity(float _density)
{
    muliAssert(density > 0);

    density = _density;
    mass = density * area;
    invMass = 1.0f / mass;
    inertia = ComputeCircleInertia(radius, mass);
    invInertia = 1.0f / inertia;
}

} // namespace muli