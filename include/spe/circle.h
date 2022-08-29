#pragma once

#include "rigidbody.h"
#include "util.h"

namespace spe
{

class Circle : public RigidBody
{
public:
    Circle(float radius, BodyType _type = Dynamic, float _density = DEFAULT_DENSITY);

    virtual void SetMass(float m) override;
    virtual void SetDensity(float d) override;

    float GetRadius() const;
    virtual float GetArea() const override final;
    virtual AABB GetAABB() const override;

protected:
    float radius;
    float area;
};

inline void Circle::SetMass(float _mass)
{
    assert(_mass > 0);

    density = _mass / area;
    mass = _mass;
    invMass = 1.0f / mass;
    inertia = calculate_circle_inertia(radius, mass);
    invInertia = 1.0f / inertia;
}

inline void Circle::SetDensity(float _density)
{
    assert(density > 0);

    density = _density;
    mass = density * area;
    invMass = 1.0f / mass;
    inertia = calculate_circle_inertia(radius, mass);
    invInertia = 1.0f / inertia;
}

inline float Circle::GetRadius() const
{
    return radius;
}

inline float Circle::GetArea() const
{
    return area;
}

inline AABB Circle::GetAABB() const
{
    // clang-format off
    return AABB
    {
        glm::vec2(position.x - radius, position.y - radius),
        glm::vec2(position.x + radius, position.y + radius)
    };
    // clang-format on
}

} // namespace spe