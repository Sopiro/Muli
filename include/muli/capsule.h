#pragma once

#include "rigidbody.h"
#include "util.h"

namespace muli
{

class Capsule : public RigidBody
{
public:
    Capsule(float _length, float _radius, bool _horizontal = false, Type _type = Dynamic, float _density = DEFAULT_DENSITY);
    Capsule(const Vec2& p1,
            const Vec2& p2,
            float _radius,
            bool _resetPosition = false,
            Type _type = Dynamic,
            float _density = DEFAULT_DENSITY);

    virtual void SetMass(float m) override;
    virtual void SetDensity(float d) override;

    virtual float GetArea() const override;
    virtual AABB GetAABB() const override;
    virtual ContactPoint Support(const Vec2& localDir) const override;
    virtual Edge GetFeaturedEdge(const Vec2& dir) const override;

    float GetLength() const;
    const Vec2& GetVertexA() const;
    const Vec2& GetVertexB() const;

protected:
    Vec2 va;
    Vec2 vb;

    float length;
    float area;
};

inline float Capsule::GetLength() const
{
    return length;
}

inline const Vec2& Capsule::GetVertexA() const
{
    return va;
}

inline const Vec2& Capsule::GetVertexB() const
{
    return vb;
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

inline ContactPoint Capsule::Support(const Vec2& localDir) const
{
    return localDir.x > 0 ? ContactPoint{ vb, 1 } : ContactPoint{ va, 0 };
}

inline Edge Capsule::GetFeaturedEdge(const Vec2& dir) const
{
    return Edge{ transform * va, transform * vb, 0, 1 };
}

} // namespace muli