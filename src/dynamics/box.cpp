#include "spe/box.h"

namespace spe
{

Box::Box(float _width, float _height, Type _type, float _radius, float _density)
    : Polygon{ { Vec2{ 0, 0 }, Vec2{ _width, 0 }, Vec2{ _width, _height }, Vec2{ 0, _height } }, _type, true, _radius, _density }
    , width{ _width }
    , height{ _height }
{
}

Box::Box(float _size, Type _type, float _radius, float _density)
    : Box{ _size, _size, _type, _radius, _density }
{
}

// This will automatically set the inertia
void Box::SetMass(float _mass)
{
    speAssert(_mass > 0);

    density = _mass / area;
    mass = _mass;
    invMass = 1.0f / mass;
    inertia = ComputeBoxInertia(width, height, mass);
    invInertia = 1.0f / inertia;
}

// This will automatically set the mass and inertia
void Box::SetDensity(float _density)
{
    speAssert(_density > 0);

    density = _density;
    mass = _density * area;
    invMass = 1.0f / mass;
    inertia = ComputeBoxInertia(width, height, mass);
    invInertia = 1.0f / inertia;
}

} // namespace spe