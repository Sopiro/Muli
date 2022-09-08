#pragma once

#include "rigidbody.h"
#include "util.h"

namespace spe
{

class Circle : public RigidBody
{
public:
    Circle(float radius, Type _type = Dynamic, float _density = DEFAULT_DENSITY);

    virtual void SetMass(float m) override;
    virtual void SetDensity(float d) override;

    float GetRadius() const;
    virtual float GetArea() const override final;
    virtual AABB GetAABB() const override;

protected:
    float radius;
    float area;
};

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
        glm::vec2(transform.position.x - radius, transform.position.y - radius),
        glm::vec2(transform.position.x + radius, transform.position.y + radius)
    };
    // clang-format on
}

} // namespace spe