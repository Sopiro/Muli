#pragma once

#include "rigidbody.h"
#include "util.h"

namespace spe
{

class Capsule : public RigidBody
{
public:
    Capsule(float _length, float _radius, bool _horizontal = false, Type _type = Dynamic, float _density = DEFAULT_DENSITY);
    Capsule(const Vec2& p1,
            const Vec2& p2,
            float _radius,
            bool _resetPosition = true,
            Type _type = Dynamic,
            float _density = DEFAULT_DENSITY);

    virtual void SetMass(float m) override;
    virtual void SetDensity(float d) override;

    float GetLength() const;
    float GetRadius() const;
    const Vec2& GetV1() const;
    const Vec2& GetV2() const;
    virtual float GetArea() const override final;
    virtual AABB GetAABB() const override;

protected:
    Vec2 v1;
    Vec2 v2;

    float length;
    float radius;
    float area;
};

inline float Capsule::GetLength() const
{
    return length;
}

inline float Capsule::GetRadius() const
{
    return radius;
}

inline const Vec2& Capsule::GetV1() const
{
    return v1;
}

inline const Vec2& Capsule::GetV2() const
{
    return v2;
}

inline float Capsule::GetArea() const
{
    return area;
}

inline AABB Capsule::GetAABB() const
{
    Vec2 v1 = transform * Vec2{ -length / 2.0f, 0.0f };
    Vec2 v2 = transform * Vec2{ length / 2.0f, 0.0f };

    return AABB{ Vec2{ Min(v1, v2) - Vec2{ radius, radius } }, Vec2{ Max(v1, v2) + Vec2{ radius, radius } } };
}

} // namespace spe