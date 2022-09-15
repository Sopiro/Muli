#pragma once

#include "rigidbody.h"
#include "util.h"

namespace spe
{

class Circle : public RigidBody
{
public:
    Circle(float _radius, Type _type = Dynamic, float _density = DEFAULT_DENSITY);

    virtual void SetMass(float m) override;
    virtual void SetDensity(float d) override;

    virtual float GetArea() const override final;
    virtual AABB GetAABB() const override;

protected:
    float area;
};

inline float Circle::GetArea() const
{
    return area;
}

inline AABB Circle::GetAABB() const
{
    // clang-format off
    return AABB
    {
        Vec2{transform.position.x - radius, transform.position.y - radius},
        Vec2{transform.position.x + radius, transform.position.y + radius}
    };
    // clang-format on
}

} // namespace spe