#pragma once

#include "collision_filter.h"
#include "common.h"
#include "material.h"
#include "predefined_block_allocator.h"
#include "rigidbody.h"
#include "shape.h"

namespace muli
{

class ColliderDestoryCallback;
class ContactListener;
typedef int32 NodeProxy;

class Collider final
{
public:
    RigidBody* GetBody();
    const RigidBody* GetBody() const;

    Shape::Type GetType() const;

    Shape* GetShape();
    const Shape* GetShape() const;

    const CollisionFilter& GetFilter() const;
    void SetFilter(const CollisionFilter& filter);

    float GetMass() const;
    void SetMass(float mass);
    float GetDensity() const;
    void SetDensity(float density);

    const Material& GetMaterial() const;
    void SetMaterial(const Material& material);
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

    Collider* GetNext();
    const Collider* GetNext() const;

    ColliderDestoryCallback* OnDestroy;
    muli::ContactListener* ContactListener;

private:
    friend class RigidBody;
    friend class AABBTree;
    friend class BroadPhase;
    friend class Contact;
    friend class ContactManager;
    friend class World;
    friend class ContactSolver;

    Collider();
    ~Collider();

    void Create(Allocator* allocator, RigidBody* body, Shape* shape, float density, const Material& material);
    void Destroy(Allocator* allocator);

    float density;

    Material material;

    RigidBody* body;
    Shape* shape;

    Collider* next;
    NodeProxy node;
};

inline RigidBody* Collider::GetBody()
{
    return body;
}

inline const RigidBody* Collider::GetBody() const
{
    return body;
}

inline Shape::Type Collider::GetType() const
{
    return shape->GetType();
}

inline Shape* Collider::GetShape()
{
    return shape;
}

inline const Shape* Collider::GetShape() const
{
    return shape;
}

inline float Collider::GetMass() const
{
    return shape->area * density;
}

inline void Collider::SetMass(float _mass)
{
    density = _mass / shape->area;
    body->ResetMassData();
}

inline float Collider::GetDensity() const
{
    return density;
}

inline void Collider::SetDensity(float _density)
{
    density = _density;
    body->ResetMassData();
}

inline const Material& Collider::GetMaterial() const
{
    return material;
}

inline void Collider::SetMaterial(const Material& _material)
{
    material = _material;
}

inline float Collider::GetFriction() const
{
    return material.friction;
}

inline void Collider::SetFriction(float _friction)
{
    material.friction = _friction;
}

inline float Collider::GetRestitution() const
{
    return material.restitution;
}

inline void Collider::SetRestitution(float _restitution)
{
    material.restitution = _restitution;
}

inline float Collider::GetSurfaceSpeed() const
{
    return material.surfaceSpeed;
}

inline void Collider::SetSurfaceSpeed(float _surfaceSpeed)
{
    material.surfaceSpeed = _surfaceSpeed;
}

inline const CollisionFilter& Collider::GetFilter() const
{
    return material.filter;
}

inline void Collider::SetFilter(const CollisionFilter& _filter)
{
    material.filter = _filter;
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

inline Collider* Collider::GetNext()
{
    return next;
}

inline const Collider* Collider::GetNext() const
{
    return next;
}

} // namespace muli
