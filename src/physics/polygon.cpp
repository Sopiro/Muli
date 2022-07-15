#include "spe/physics/polygon.h"

using namespace spe;

Polygon::Polygon(std::vector<glm::vec2> _vertices, BodyType _type, bool _resetPosition, float _density) :
    RigidBody(std::move(_type)),
    vertices{ std::move(_vertices) }
{
    glm::vec2 centerOfMass{ 0.0f };
    size_t count = vertices.size();

    for (int i = 0; i < count; i++)
    {
        centerOfMass += vertices[i];
    }

    centerOfMass /= count;

    float _area = 0;

    vertices[0] -= centerOfMass;
    radius = glm::length(vertices[0]);

    for (int i = 1; i < count; i++)
    {
        vertices[i] -= centerOfMass;
        radius = glm::max(radius, glm::length(vertices[i]));
        _area += glm::cross(vertices[i - 1], vertices[i]);
    }
    _area += glm::cross(vertices[count - 1], vertices[0]);

    area = glm::abs(_area) / 2.0f;

    if (type == Dynamic)
    {
        assert(_density > 0);

        density = _density;
        mass = _density * area;
        invMass = 1.0f / mass;
        inertia = calculate_convex_polygon_inertia(vertices, mass, area);
        invInertia = 1.0f / inertia;
    }

    if (!_resetPosition)
        Translate(centerOfMass);
}

void Polygon::SetMass(float _mass)
{
    assert(_mass > 0);

    density = _mass / area;
    mass = _mass;
    invMass = 1.0f / mass;
    inertia = calculate_convex_polygon_inertia(vertices, mass, area);
    invInertia = 1.0f / inertia;
}

void Polygon::SetDensity(float _density)
{
    assert(_density > 0);

    density = _density;
    mass = _density * area;
    invMass = 1.0f / mass;
    inertia = calculate_convex_polygon_inertia(vertices, mass, area);
    invInertia = 1.0f / inertia;
}

float Polygon::GetRadius() const
{
    return radius;
}

float Polygon::GetArea() const
{
    return area;
}

const std::vector<glm::vec2>& Polygon::GetVertices() const
{
    return vertices;
}

size_t Polygon::VertexCount() const
{
    return vertices.size();
}
