#include "spe/polygon.h"

namespace spe
{

Polygon::Polygon(const std::vector<Vec2>& _vertices, Type _type, bool _resetCenter, float _radius, float _density)
    : RigidBody(_type, RigidBody::Shape::ShapePolygon)
    , vertices{ _vertices }
{
    radius = _radius;

    Vec2 centerOfMass{ 0.0f };
    size_t count = vertices.size();

    for (size_t i = 0; i < count; i++)
    {
        centerOfMass += vertices[i];
    }

    centerOfMass *= 1.0f / count;

    float _area = 0;

    vertices[0] -= centerOfMass;

    for (uint32 i = 1; i < count; i++)
    {
        vertices[i] -= centerOfMass;
        _area += Cross(vertices[i - 1], vertices[i]);
    }
    _area += Cross(vertices[count - 1], vertices[0]);

    area = Abs(_area) / 2.0f;

    if (type == Dynamic)
    {
        speAssert(_density > 0);

        density = _density;
        mass = _density * area;
        invMass = 1.0f / mass;
        inertia = ComputePolygonInertia(vertices, mass);
        invInertia = 1.0f / inertia;
    }

    if (!_resetCenter)
    {
        Translate(centerOfMass);
    }
}

void Polygon::SetMass(float _mass)
{
    speAssert(_mass > 0);

    density = _mass / area;
    mass = _mass;
    invMass = 1.0f / mass;
    inertia = ComputePolygonInertia(vertices, mass);
    invInertia = 1.0f / inertia;
}

void Polygon::SetDensity(float _density)
{
    speAssert(_density > 0);

    density = _density;
    mass = _density * area;
    invMass = 1.0f / mass;
    inertia = ComputePolygonInertia(vertices, mass);
    invInertia = 1.0f / inertia;
}

} // namespace spe