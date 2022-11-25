#pragma once

#include "collision_filter.h"
#include "common.h"
#include "rigidbody.h"
#include "shape.h"

namespace muli
{

class Collider
{
public:
    const RigidBody* GetBody() const;

    Shape::Type GetType() const;
    const Shape* GetShape() const;

    const CollisionFilter& GetFilter() const;
    void SetFilter(const CollisionFilter& _filter);

    float GetDensity() const;
    float GetFriction() const;
    void SetFriction(float _friction);
    float GetRestitution() const;
    void SetRestitution(float _restitution);
    float GetSurfaceSpeed() const;
    void SetSurfaceSpeed(float _surfaceSpeed);

    void ComputeMass(MassData* outMassData) const;
    bool TestPoint(const Vec2& q) const;
    bool RayCast(const RayCastInput& input, RayCastOutput* output) const;

private:
    Collider(RigidBody* body, Shape* shape, float density, float friction, float restitution, float surfaceSpeed);

    RigidBody* body;
    Shape* shape;

    Collider* next;

    float density;
    float friction;
    float restitution;
    float surfaceSpeed;

    CollisionFilter filter;
};

inline const RigidBody* Collider::GetBody() const
{
    return body;
}

inline Shape::Type Collider::GetType() const
{
    return shape->GetType();
}

inline const Shape* Collider::GetShape() const
{
    return shape;
}

inline const CollisionFilter& Collider::GetFilter() const
{
    return filter;
}

inline void Collider::SetFilter(const CollisionFilter& _filter)
{
    filter = _filter;
}

inline float Collider::GetDensity() const
{
    return density;
}

inline float Collider::GetFriction() const
{
    return friction;
}

inline void Collider::SetFriction(float _friction)
{
    friction = _friction;
}

inline float Collider::GetRestitution() const
{
    return restitution;
}

inline void Collider::SetRestitution(float _restitution)
{
    restitution = _restitution;
}

inline float Collider::GetSurfaceSpeed() const
{
    return surfaceSpeed;
}

inline void Collider::SetSurfaceSpeed(float _surfaceSpeed)
{
    surfaceSpeed = _surfaceSpeed;
}

inline void Collider::ComputeMass(MassData* outMassData) const
{
    shape->ComputeMass(density, outMassData);
}

inline bool Collider::TestPoint(const Vec2& q) const
{
    return shape->TestPoint(body->transform, q);
}

inline bool Collider::RayCast(const RayCastInput& input, RayCastOutput* output) const
{
    return shape->RayCast(body->transform, input, output);
}

} // namespace muli
