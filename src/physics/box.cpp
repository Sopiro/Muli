#include "box.h"

using namespace spe;

Box::Box(float _width, float _height, Type _type, float _density) :
    Polygon{ {{0, 0}, {0, _height}, {_width, _height}, {_width, 0}}, std::move(_type), true, std::move(_density) },
    width{ std::move(_width) },
    height{ std::move(_height) }
{

}

float Box::GetWidth()
{
    return width;
}

float Box::GetHeight()
{
    return height;
}

// This will automatically set the inertia
void Box::SetMass(float _mass)
{
    assert(_mass > 0);

    density = _mass / area;
    mass = _mass;
    invMass = 1.0f / mass;
    inertia = calculate_box_inertia(width, height, mass);
    invInertia = 1.0f / inertia;
}

// This will automatically set the mass and inertia
void Box::SetDensity(float _density)
{
    assert(_density > 0);

    density = _density;
    mass = _density * area;
    invMass = 1.0f / mass;
    inertia = calculate_box_inertia(width, height, mass);
    invInertia = 1.0f / inertia;
}