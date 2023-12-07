#pragma once

#include "allocator.h"
#include "collision_filter.h"
#include "material.h"
#include "rigidbody.h"
#include "shape.h"

namespace muli
{

class ColliderDestroyCallback;
class ContactListener;
typedef int32 NodeProxy;

class Collider
{
public:
    RigidBody* GetBody();
    const RigidBody* GetBody() const;

    Collider* GetNext();
    const Collider* GetNext() const;

    Shape::Type GetType() const;

    Shape* GetShape();
    const Shape* GetShape() const;

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
    float GetRestitutionTreshold() const;
    void SetRestitutionTreshold(float treshold);
    float GetSurfaceSpeed() const;
    void SetSurfaceSpeed(float surfaceSpeed);

    const CollisionFilter& GetFilter() const;
    void SetFilter(const CollisionFilter& filter);

    bool IsEnabled() const;
    void SetEnabled(bool enabled);

    AABB GetAABB() const;
    MassData GetMassData() const;

    bool TestPoint(const Vec2& q) const;
    Vec2 GetClosestPoint(const Vec2& q) const;
    bool RayCast(const RayCastInput& input, RayCastOutput* output) const;

    ColliderDestroyCallback* OnDestroy;
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

    RigidBody* body;
    Collider* next;

    Shape* shape;

    float density;

    Material material;
    CollisionFilter filter;

    NodeProxy node;

    bool enabled;
};

inline RigidBody* Collider::GetBody()
{
    return body;
}

inline const RigidBody* Collider::GetBody() const
{
    return body;
}

inline Collider* Collider::GetNext()
{
    return next;
}

inline const Collider* Collider::GetNext() const
{
    return next;
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

inline float Collider::GetRestitutionTreshold() const
{
    return material.restitutionTreshold;
}

inline void Collider::SetRestitutionTreshold(float _treshold)
{
    material.restitutionTreshold = _treshold;
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
    return filter;
}

inline void Collider::SetFilter(const CollisionFilter& _filter)
{
    filter = _filter;
}

inline bool Collider::IsEnabled() const
{
    return enabled;
}

inline void Collider::SetEnabled(bool _enabled)
{
    enabled = _enabled;
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

} // namespace muli
