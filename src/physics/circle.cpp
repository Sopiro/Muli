#include "spe/physics/circle.h"

namespace spe
{

Circle::Circle(float _radius, BodyType _type, float _density) :
    RigidBody(std::move(_type)),
    radius{ std::move(_radius) }
{
    area = glm::pi<float>() * radius * radius;

    if (type == Dynamic)
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

float Circle::GetRadius() const
{
    return radius;
}

float Circle::GetArea() const
{
    return area;
}

}