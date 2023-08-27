#pragma once

#include "aabb.h"
#include "collision.h"
#include "collision_filter.h"
#include "common.h"
#include "edge.h"
#include "material.h"
#include "settings.h"

namespace muli
{

class World;
class Collider;
class Shape;
struct Node;
struct ContactEdge;
struct JointEdge;
class RayCastAnyCallback;
class RayCastClosestCallback;
class BodyDestroyCallback;

class RigidBody
{
public:
    enum Type
    {
        static_body = 0,
        kinematic_body,
        dynamic_body,
    };

    RigidBody(RigidBody::Type type);
    ~RigidBody() noexcept;

    RigidBody(const RigidBody&) = delete;
    RigidBody& operator=(const RigidBody&) = delete;

    RigidBody(RigidBody&&) noexcept = delete;
    RigidBody& operator=(RigidBody&&) = delete;

    const Transform& GetTransform() const;
    void SetTransform(const Transform& transform);
    void SetTransform(const Vec2& pos, float angle);

    const Sweep& GetSweep() const;
    const Vec2& GetLocalCenter() const;

    const Vec2& GetPosition() const;
    void SetPosition(const Vec2& position);
    void SetPosition(float x, float y);

    const Rotation& GetRotation() const;
    float GetAngle() const;
    void SetRotation(const Rotation& rotation);
    void SetRotation(float angle);

    float GetMass() const;
    float GetInertia() const;
    float GetInertiaLocalOrigin() const;

    float GetLinearDamping() const;
    void SetLinearDamping(float linearDamping);
    float GetAngularDamping() const;
    void SetAngularDamping(float angularDamping);

    const Vec2& GetForce() const;
    void SetForce(const Vec2& force);
    float GetTorque() const;
    void SetTorque(float torque);

    const Vec2& GetLinearVelocity() const;
    void SetLinearVelocity(const Vec2& linearVelocity);
    void SetLinearVelocity(float vx, float vy);
    float GetAngularVelocity() const;
    void SetAngularVelocity(float angularVelocity);

    void ApplyForce(const Vec2& worldPoint, const Vec2& force, bool awake);
    void ApplyForceLocal(const Vec2& localPoint, const Vec2& force, bool awake);
    void ApplyTorque(float torque, bool awake);

    void ApplyLinearImpulse(const Vec2& worldPoint, const Vec2& linearImpulse, bool awake);
    void ApplyLinearImpulseLocal(const Vec2& localPoint, const Vec2& linearImpulse, bool awake);
    void ApplyAngularImpulse(float angularImpulse, bool awake);

    void Translate(const Vec2& d);
    void Translate(float dx, float dy);
    void Rotate(float a);

    Type GetType() const;
    void SetType(Type type);

    void SetFixedRotation(bool fixed);
    bool IsRotationFixed() const;

    void SetContinuous(bool continuous);
    bool IsContinuous() const;

    void SetSleeping(bool sleeping);
    bool IsSleeping() const;
    void Awake();
    void Sleep();

    void SetEnabled(bool enabled);
    bool IsEnabled() const;

    int32 GetIslandID() const;
    int32 GetIslandIndex() const;

    RigidBody* GetPrev();
    const RigidBody* GetPrev() const;
    RigidBody* GetNext();
    const RigidBody* GetNext() const;
    World* GetWorld();
    const World* GetWorld() const;

    // These material functions affect all child colliders
    void SetCollisionFilter(const CollisionFilter& filter) const;
    void SetFriction(float friction) const;
    void SetRestitution(float restitution) const;
    void SetRestitutionThreshold(float threshold) const;
    void SetSurfaceSpeed(float surfaceSpeed) const;

    bool TestPoint(const Vec2& q) const;
    Vec2 GetClosestPoint(const Vec2& q) const;
    void RayCastAny(
        const Vec2& from,
        const Vec2& to,
        const std::function<float(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)>& callback) const;
    bool RayCastClosest(
        const Vec2& from,
        const Vec2& to,
        const std::function<void(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)>& callback) const;
    void RayCastAny(const Vec2& from, const Vec2& to, RayCastAnyCallback* callback) const;
    bool RayCastClosest(const Vec2& from, const Vec2& to, RayCastClosestCallback* callback) const;

    // Collider factory functions

    Collider* CreateCollider(Shape* shape, float density = default_density, const Material& material = default_material);
    void DestroyCollider(Collider* collider);

    int32 GetColliderCount() const;
    Collider* GetColliderList();
    const Collider* GetColliderList() const;

    Collider* CreateCircleCollider(float radius,
                                   const Vec2& position = zero_vec2,
                                   float density = default_density,
                                   const Material& material = default_material);
    Collider* CreateBoxCollider(float width,
                                float height,
                                float radius = default_radius,
                                const Vec2& position = zero_vec2,
                                float angle = 0.0f,
                                float density = default_density,
                                const Material& material = default_material);
    Collider* CreateCapsuleCollider(float length,
                                    float radius,
                                    bool horizontal = false,
                                    const Vec2& position = zero_vec2,
                                    float density = default_density,
                                    const Material& material = default_material);
    Collider* CreateCapsuleCollider(const Vec2& p1,
                                    const Vec2& p2,
                                    float radius,
                                    bool resetPosition = false,
                                    float density = default_density,
                                    const Material& material = default_material);

    // Callbacks
    BodyDestroyCallback* OnDestroy;
    void* UserData;

protected:
    friend class World;
    friend class Island;

    friend class AABBTree;
    friend class BroadPhase;
    friend class ContactManager;

    friend class Collider;

    friend class Contact;
    friend class ContactSolver;
    friend class BlockSolver;
    friend class PositionSolver;

    friend class Joint;
    friend class GrabJoint;
    friend class RevoluteJoint;
    friend class DistanceJoint;
    friend class AngleJoint;
    friend class WeldJoint;
    friend class LineJoint;
    friend class PrismaticJoint;
    friend class PulleyJoint;
    friend class MotorJoint;

    enum
    {
        flag_enabled = 1 << 0,
        flag_island = 1 << 1,
        flag_sleeping = 1 << 2,
        flag_continuous = 1 << 3,
        flag_fixed_rotation = 1 << 4,
    };

    Type type;

    Transform transform;   // transform relative to the body origin
    Sweep sweep;           // swept motion of rigid body used for CCD

    Vec2 force;            // N
    float torque;          // N⋅m

    Vec2 linearVelocity;   // m/s
    float angularVelocity; // rad/s

    float mass;            // kg
    float invMass;
    float inertia;         // kg⋅m²
    float invInertia;

    float linearDamping;
    float angularDamping;

    int32 islandIndex;
    int32 islandID;

    uint16 flag;

    void ResetMassData();
    void SynchronizeTransform();
    void SynchronizeColliders();

    void Advance(float alpha);

private:
    World* world;

    RigidBody* prev;
    RigidBody* next;

    Collider* colliderList;
    int32 colliderCount;

    ContactEdge* contactList;
    JointEdge* jointList;

    float resting;
};

inline const Transform& RigidBody::GetTransform() const
{
    return transform;
}

inline void RigidBody::SetTransform(const Transform& _transform)
{
    SetTransform(_transform.position, _transform.rotation.GetAngle());
}

inline void RigidBody::SetTransform(const Vec2& _pos, float _angle)
{
    transform.position = _pos;
    transform.rotation = _angle;

    sweep.c = Mul(transform, sweep.localCenter);
    sweep.a = _angle;

    sweep.c0 = sweep.c;
    sweep.a0 = sweep.a;

    SynchronizeColliders();
}

inline const Sweep& RigidBody::GetSweep() const
{
    return sweep;
}

inline const Vec2& RigidBody::GetLocalCenter() const
{
    return sweep.localCenter;
}

inline const Vec2& RigidBody::GetPosition() const
{
    return transform.position;
}

inline void RigidBody::SetPosition(const Vec2& pos)
{
    SetPosition(pos.x, pos.y);
}

inline void RigidBody::SetPosition(float x, float y)
{
    transform.position.Set(x, y);
    sweep.c = Mul(transform, sweep.localCenter);
    sweep.c0 = sweep.c;

    SynchronizeColliders();
}

inline const Rotation& RigidBody::GetRotation() const
{
    return transform.rotation;
}

inline void RigidBody::SetRotation(const Rotation& _rotation)
{
    transform.rotation = _rotation;
    sweep.a = transform.rotation.GetAngle();
    sweep.a0 = sweep.a;

    SynchronizeColliders();
}

inline void RigidBody::SetRotation(float _angle)
{
    transform.rotation = _angle;
    sweep.a = _angle;
    sweep.a0 = sweep.a;

    SynchronizeColliders();
}

inline float RigidBody::GetAngle() const
{
    return sweep.a;
}

inline float RigidBody::GetMass() const
{
    return mass;
}

inline float RigidBody::GetInertia() const
{
    return inertia;
}

inline float RigidBody::GetInertiaLocalOrigin() const
{
    return inertia + mass * Length2(sweep.localCenter);
}

inline float RigidBody::GetLinearDamping() const
{
    return linearDamping;
}

inline void RigidBody::SetLinearDamping(float _linearDamping)
{
    linearDamping = _linearDamping;
}

inline float RigidBody::GetAngularDamping() const
{
    return angularDamping;
}

inline void RigidBody::SetAngularDamping(float _angularDamping)
{
    angularDamping = _angularDamping;
}

inline const Vec2& RigidBody::GetForce() const
{
    return force;
}

inline void RigidBody::SetForce(const Vec2& _force)
{
    if (type != Type::dynamic_body)
    {
        return;
    }

    force = _force;
}

inline float RigidBody::GetTorque() const
{
    return torque;
}

inline void RigidBody::SetTorque(float _torque)
{
    if (type != Type::dynamic_body)
    {
        return;
    }

    torque = _torque;
}

inline const Vec2& RigidBody::GetLinearVelocity() const
{
    return linearVelocity;
}

inline void RigidBody::SetLinearVelocity(const Vec2& _linearVelocity)
{
    SetLinearVelocity(_linearVelocity.x, _linearVelocity.y);
}

inline void RigidBody::SetLinearVelocity(float vx, float vy)
{
    if (type == Type::static_body)
    {
        return;
    }

    linearVelocity.Set(vx, vy);
}

inline float RigidBody::GetAngularVelocity() const
{
    return angularVelocity;
}

inline void RigidBody::SetAngularVelocity(float _angularVelocity)
{
    if (type == Type::static_body)
    {
        return;
    }

    angularVelocity = _angularVelocity;
}

inline void RigidBody::ApplyForce(const Vec2& worldPoint, const Vec2& inForce, bool awake)
{
    if (type != Type::dynamic_body)
    {
        return;
    }

    if (awake && IsSleeping())
    {
        Awake();
    }

    if (IsSleeping() == false)
    {
        force += inForce;
        torque += Cross(worldPoint - sweep.c, inForce);
    }
}

inline void RigidBody::ApplyForceLocal(const Vec2& localPoint, const Vec2& inForce, bool awake)
{
    if (type != Type::dynamic_body)
    {
        return;
    }

    if (awake && IsSleeping())
    {
        Awake();
    }

    if (IsSleeping() == false)
    {
        force += inForce;
        torque += Cross(localPoint - sweep.localCenter, inForce);
    }
}

inline void RigidBody::ApplyTorque(float inTorque, bool awake)
{
    if (type != Type::dynamic_body)
    {
        return;
    }

    if (awake && IsSleeping())
    {
        Awake();
    }

    if (IsSleeping() == false)
    {
        torque += inTorque;
    }
}

inline void RigidBody::ApplyLinearImpulse(const Vec2& worldPoint, const Vec2& impulse, bool awake)
{
    if (type != Type::dynamic_body)
    {
        return;
    }

    if (awake && IsSleeping())
    {
        Awake();
    }

    if (IsSleeping() == false)
    {
        linearVelocity += invMass * impulse;
        angularVelocity += invInertia * Cross(worldPoint - sweep.c, impulse);
    }
}

inline void RigidBody::ApplyLinearImpulseLocal(const Vec2& localPoint, const Vec2& impulse, bool awake)
{
    if (type != Type::dynamic_body)
    {
        return;
    }

    if (awake && IsSleeping())
    {
        Awake();
    }

    if (IsSleeping() == false)
    {
        linearVelocity += invMass * impulse;
        angularVelocity += invInertia * Cross(localPoint - sweep.localCenter, impulse);
    }
}

inline void RigidBody::ApplyAngularImpulse(float impulse, bool awake)
{
    if (type != Type::dynamic_body)
    {
        return;
    }

    if (awake && IsSleeping())
    {
        Awake();
    }

    if (IsSleeping() == false)
    {
        angularVelocity += invInertia * impulse;
    }
}

inline void RigidBody::Translate(const Vec2& d)
{
    Translate(d.x, d.y);
}

inline void RigidBody::Translate(float dx, float dy)
{
    transform.position.x += dx;
    transform.position.y += dy;
    sweep.c = Mul(transform, sweep.localCenter);
    sweep.c0 = sweep.c;

    SynchronizeColliders();
}

inline void RigidBody::Rotate(float a)
{
    sweep.a += a;
    sweep.a0 = sweep.a;
    transform.rotation = sweep.a;

    SynchronizeColliders();
}

inline RigidBody::Type RigidBody::GetType() const
{
    return type;
}

inline void RigidBody::SetFixedRotation(bool fixed)
{
    if (fixed)
    {
        flag |= flag_fixed_rotation;
    }
    else
    {
        flag &= ~flag_fixed_rotation;
    }

    ResetMassData();
}

inline bool RigidBody::IsRotationFixed() const
{
    return (flag & flag_fixed_rotation) == flag_fixed_rotation;
}

inline void RigidBody::SetContinuous(bool continuous)
{
    if (continuous)
    {
        flag |= flag_continuous;
    }
    else
    {
        flag &= ~flag_continuous;
    }
}

inline bool RigidBody::IsContinuous() const
{
    return (flag & flag_continuous) == flag_continuous;
}

inline void RigidBody::SetSleeping(bool sleeping)
{
    if (sleeping)
    {
        Sleep();
    }
    else
    {
        Awake();
    }
}

inline bool RigidBody::IsSleeping() const
{
    return (flag & flag_sleeping) == flag_sleeping;
}

inline void RigidBody::Awake()
{
    if (type == Type::static_body)
    {
        return;
    }

    resting = 0.0f;
    flag &= ~flag_sleeping;
}

inline void RigidBody::Sleep()
{
    if (type == Type::static_body)
    {
        return;
    }

    resting = max_value;
    force.SetZero();
    torque = 0.0f;
    linearVelocity.SetZero();
    angularVelocity = 0.0f;
    flag &= ~flag_sleeping;
}

inline bool RigidBody::IsEnabled() const
{
    return (flag & flag_enabled) == flag_enabled;
}

inline int32 RigidBody::GetIslandID() const
{
    return islandID;
}

inline int32 RigidBody::GetIslandIndex() const
{
    return islandIndex;
}

inline RigidBody* RigidBody::GetPrev()
{
    return prev;
}

inline const RigidBody* RigidBody::GetPrev() const
{
    return prev;
}

inline RigidBody* RigidBody::GetNext()
{
    return next;
}

inline const RigidBody* RigidBody::GetNext() const
{
    return next;
}

inline const World* RigidBody::GetWorld() const
{
    return world;
}

inline World* RigidBody::GetWorld()
{
    return world;
}

inline Collider* RigidBody::GetColliderList()
{
    return colliderList;
}

inline const Collider* RigidBody::GetColliderList() const
{
    return colliderList;
}

inline int32 RigidBody::GetColliderCount() const
{
    return colliderCount;
}

inline void RigidBody::SynchronizeTransform()
{
    transform.rotation = sweep.a;
    // Shift to origin
    transform.position = sweep.c - Mul(transform.rotation, sweep.localCenter);
}

// Advance to the new safe time
// c0 = c = sweep(alpha)
// a0 = a = sweep(alpha)
inline void RigidBody::Advance(float alpha)
{
    sweep.Advance(alpha);
    sweep.c = sweep.c0;
    sweep.a = sweep.a0;

    transform.rotation = sweep.a;
    transform.position = sweep.c - Mul(transform.rotation, sweep.localCenter);
}

} // namespace muli