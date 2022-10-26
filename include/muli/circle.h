#pragma once

#include "rigidbody.h"
#include "util.h"

namespace muli
{

class Circle : public RigidBody
{
public:
    Circle(float _radius, Type _type = Dynamic, float _density = DEFAULT_DENSITY);

    virtual void SetMass(float m) override;
    virtual void SetDensity(float d) override;

    virtual float GetArea() const override final;
    virtual AABB GetAABB() const override;
    virtual ContactPoint Support(const Vec2& localDir) const override;
    virtual Edge GetFeaturedEdge(const Vec2& dir) const override;
    virtual bool TestPoint(const Vec2& p) const override;
    virtual bool RayCast(const RayCastInput& input, RayCastOutput* output) const override;

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

inline ContactPoint Circle::Support(const Vec2& localDir) const
{
    return ContactPoint{ Vec2{ 0.0f, 0.0f }, -1 };
}

inline Edge Circle::GetFeaturedEdge(const Vec2& dir) const
{
    return Edge{ transform.position, transform.position };
}

inline bool Circle::TestPoint(const Vec2& p) const
{
    return Dist2(transform.position, p) < radius * radius;
}

} // namespace muli