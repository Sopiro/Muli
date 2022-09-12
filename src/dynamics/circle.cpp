#include "spe/circle.h"

namespace spe
{

Circle::Circle(float _radius, Type _type, float _density)
    : RigidBody(_type, Shape::ShapeCircle)
    , radius{ _radius }
{
    area = SPE_PI * radius * radius;

    if (type == RigidBody::Type::Dynamic)
    {
        speAssert(_density > 0);

        density = _density;
        mass = _density * area;
        invMass = 1.0f / mass;
        inertia = ComputeCircleInertia(radius, mass);
        invInertia = 1.0f / inertia;
    }
}

void Circle::SetMass(float _mass)
{
    speAssert(_mass > 0);

    density = _mass / area;
    mass = _mass;
    invMass = 1.0f / mass;
    inertia = ComputeCircleInertia(radius, mass);
    invInertia = 1.0f / inertia;
}

void Circle::SetDensity(float _density)
{
    speAssert(density > 0);

    density = _density;
    mass = density * area;
    invMass = 1.0f / mass;
    inertia = ComputeCircleInertia(radius, mass);
    invInertia = 1.0f / inertia;
}

} // namespace spe