#pragma once

#include "collision_filter.h"
#include "common.h"
#include "predefined_block_allocator.h"
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
    void SetFilter(const CollisionFilter& filter);

    float GetDensity() const;
    float GetFriction() const;
    void SetFriction(float friction);
    float GetRestitution() const;
    void SetRestitution(float restitution);
    float GetSurfaceSpeed() const;
    void SetSurfaceSpeed(float surfaceSpeed);

    AABB GetAABB() const;
    MassData GetMassData() const;
    bool TestPoint(const Vec2& q) const;
    Vec2 GetClosestPoint(const Vec2& q) const;
    bool RayCast(const RayCastInput& input, RayCastOutput* output) const;

    Collider* GetNext() const;

private:
    friend class RigidBody;
    friend class AABBTree;
    friend class BroadPhase;
    friend class Contact;
    friend class ContactManager;
    friend class World;
    friend class ContactSolver;

    Collider();
    ~Collider() = default;
    void Create(PredefinedBlockAllocator* allocator,
                RigidBody* body,
                Shape* shape,
                float density,
                float friction,
                float restitution,
                float surfaceSpeed);
    void Destroy(PredefinedBlockAllocator* allocator);

    RigidBody* body;
    Shape* shape;

    float density;
    float friction;
    float restitution;
    float surfaceSpeed;

    CollisionFilter filter;

    Collider* next;
    int32 node;
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

inline AABB Collider::GetAABB() const
{
    AABB aabb;
    shape->ComputeAABB(body->transform, &aabb);
    return aabb;
}

inline MassData Collider::GetMassData() const
{
    MassData massData;
    shape->ComputeMass(density, &massData);
    return massData;
}

inline bool Collider::TestPoint(const Vec2& q) const
{
    return shape->TestPoint(body->transform, q);
}

inline Vec2 Collider::GetClosestPoint(const Vec2& q) const
{
    return shape->GetClosestPoint(body->transform, q);
}

inline bool Collider::RayCast(const RayCastInput& input, RayCastOutput* output) const
{
    return shape->RayCast(body->transform, input, output);
}

inline Collider* Collider::GetNext() const
{
    return next;
}

} // namespace muli
