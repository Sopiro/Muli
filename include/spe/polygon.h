#pragma once

#include "util.h"
#include "rigidbody.h"

namespace spe
{
// Children: Box
class Polygon : public RigidBody
{
public:
    Polygon(std::vector<glm::vec2> _vertices, BodyType _type = Dynamic, bool _resetPosition = true, float _density = DEFAULT_DENSITY);

    inline virtual void SetDensity(float d) override;
    inline virtual void SetMass(float m) override;

    inline float GetRadius() const;
    inline virtual float GetArea() const override final;

    inline const std::vector<glm::vec2>& GetVertices() const;
    inline size_t VertexCount() const;

protected:
    std::vector<glm::vec2> vertices;
    float area;
    float radius;
};

inline void Polygon::SetMass(float _mass)
{
    assert(_mass > 0);

    density = _mass / area;
    mass = _mass;
    invMass = 1.0f / mass;
    inertia = calculate_convex_polygon_inertia(vertices, mass, area);
    invInertia = 1.0f / inertia;
}

inline void Polygon::SetDensity(float _density)
{
    assert(_density > 0);

    density = _density;
    mass = _density * area;
    invMass = 1.0f / mass;
    inertia = calculate_convex_polygon_inertia(vertices, mass, area);
    invInertia = 1.0f / inertia;
}

inline float Polygon::GetRadius() const
{
    return radius;
}

inline float Polygon::GetArea() const
{
    return area;
}

inline const std::vector<glm::vec2>& Polygon::GetVertices() const
{
    return vertices;
}

inline size_t Polygon::VertexCount() const
{
    return vertices.size();
}

}