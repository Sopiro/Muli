#include "circle.h"

using namespace spe;

Circle::Circle(float _radius, Type _type, float _density) :
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
        inertia = CalculateCircleInertia(radius, mass);
        invInertia = 1.0f / inertia;
    }
}

Circle::~Circle()
{

}

void Circle::SetMass(float _mass)
{
    assert(_mass > 0);

    density = _mass / area;
    mass = _mass;
    invMass = 1.0f / mass;
    inertia = CalculateCircleInertia(radius, mass);
    invInertia = 1.0f / inertia;
}

void Circle::SetDensity(float _density)
{
    assert(density > 0);

    density = _density;
    mass = density * area;
    invMass = 1.0f / mass;
    inertia = CalculateCircleInertia(radius, mass);
    invInertia = 1.0f / inertia;
}