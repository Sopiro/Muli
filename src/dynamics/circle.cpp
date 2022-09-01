#include "spe/circle.h"

namespace spe
{

Circle::Circle(float _radius, Type _type, float _density)
    : RigidBody(std::move(_type), Shape::ShapeCircle)
    , radius{ std::move(_radius) }
{
    area = glm::pi<float>() * radius * radius;

    if (type == RigidBody::Type::Dynamic)
    {
        assert(_density > 0);

        density = _density;
        mass = _density * area;
        invMass = 1.0f / mass;
        inertia = calculate_circle_inertia(radius, mass);
        invInertia = 1.0f / inertia;
    }
}

void Circle::SetMass(float _mass)
{
    assert(_mass > 0);

    density = _mass / area;
    mass = _mass;
    invMass = 1.0f / mass;
    inertia = calculate_circle_inertia(radius, mass);
    invInertia = 1.0f / inertia;
}

void Circle::SetDensity(float _density)
{
    assert(density > 0);

    density = _density;
    mass = density * area;
    invMass = 1.0f / mass;
    inertia = calculate_circle_inertia(radius, mass);
    invInertia = 1.0f / inertia;
}

} // namespace spe