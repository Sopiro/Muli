#pragma once

#include "aabb.h"
#include "collision.h"
#include "collision_filter.h"
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

    RigidBody(const Transform& tf, RigidBody::Type type);
    ~RigidBody();

    RigidBody(const RigidBody&) = delete;
    RigidBody& operator=(const RigidBody&) = delete;

    const Transform& GetTransform() const;
    void SetTransform(const Transform& transform);
    void SetTransform(const Vec2& pos, float angle);

    const Motion& GetMotion() const;
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

    RigidBody::Type GetType() const;
    void SetType(RigidBody::Type type);

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
        std::function<float(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)> callback
    ) const;
    bool RayCastClosest(
        const Vec2& from,
        const Vec2& to,
        std::function<void(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)> callback
    ) const;
    void RayCastAny(const Vec2& from, const Vec2& to, RayCastAnyCallback* callback) const;
    bool RayCastClosest(const Vec2& from, const Vec2& to, RayCastClosestCallback* callback) const;

    // Collider factory functions

    Collider* CreateCollider(
        Shape* shape, const Transform& tf = identity, float density = default_density, const Material& material = default_material
    );
    void DestroyCollider(Collider* collider);

    int32 GetColliderCount() const;
    Collider* GetColliderList();
    const Collider* GetColliderList() const;

    Collider* CreateCircleCollider(
        float radius, const Transform& tf = identity, float density = default_density, const Material& material = default_material
    );
    Collider* CreateBoxCollider(
        float width,
        float height,
        float radius = default_radius,
        const Transform& tf = identity,
        float density = default_density,
        const Material& material = default_material
    );
    Collider* CreateCapsuleCollider(
        float length,
        float radius,
        bool horizontal = false,
        const Transform& tf = identity,
        float density = default_density,
        const Material& material = default_material
    );
    Collider* CreateCapsuleCollider(
        const Vec2& p1,
        const Vec2& p2,
        float radius,
        bool resetPosition = false,
        const Transform& tf = identity,
        float density = default_density,
        const Material& material = default_material
    );

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
    friend class ContactSolverNormal;
    friend class ContactSolverTangent;
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
    Motion motion;         // swept motion of rigid body used for CCD

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

inline void RigidBody::SetTransform(const Transform& newTransform)
{
    SetTransform(newTransform.position, newTransform.rotation.GetAngle());
}

inline void RigidBody::SetTransform(const Vec2& newPos, float newAngle)
{
    transform.position = newPos;
    transform.rotation = newAngle;

    motion.c = Mul(transform, motion.localCenter);
    motion.a = newAngle;

    motion.c0 = motion.c;
    motion.a0 = motion.a;

    SynchronizeColliders();
}

inline const Motion& RigidBody::GetMotion() const
{
    return motion;
}

inline const Vec2& RigidBody::GetLocalCenter() const
{
    return motion.localCenter;
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
    motion.c = Mul(transform, motion.localCenter);
    motion.c0 = motion.c;

    SynchronizeColliders();
}

inline const Rotation& RigidBody::GetRotation() const
{
    return transform.rotation;
}

inline void RigidBody::SetRotation(const Rotation& newRotation)
{
    transform.rotation = newRotation;
    motion.a = transform.rotation.GetAngle();
    motion.a0 = motion.a;

    SynchronizeColliders();
}

inline void RigidBody::SetRotation(float newAngle)
{
    transform.rotation = newAngle;
    motion.a = newAngle;
    motion.a0 = motion.a;

    SynchronizeColliders();
}

inline float RigidBody::GetAngle() const
{
    return motion.a;
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
    return inertia + mass * Length2(motion.localCenter);
}

inline float RigidBody::GetLinearDamping() const
{
    return linearDamping;
}

inline void RigidBody::SetLinearDamping(float newLinearDamping)
{
    linearDamping = newLinearDamping;
}

inline float RigidBody::GetAngularDamping() const
{
    return angularDamping;
}

inline void RigidBody::SetAngularDamping(float newAngularDamping)
{
    angularDamping = newAngularDamping;
}

inline const Vec2& RigidBody::GetForce() const
{
    return force;
}

inline void RigidBody::SetForce(const Vec2& newForce)
{
    if (type != Type::dynamic_body)
    {
        return;
    }

    force = newForce;
}

inline float RigidBody::GetTorque() const
{
    return torque;
}

inline void RigidBody::SetTorque(float newTorque)
{
    if (type != Type::dynamic_body)
    {
        return;
    }

    torque = newTorque;
}

inline const Vec2& RigidBody::GetLinearVelocity() const
{
    return linearVelocity;
}

inline void RigidBody::SetLinearVelocity(const Vec2& newLinearVelocity)
{
    SetLinearVelocity(newLinearVelocity.x, newLinearVelocity.y);
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

inline void RigidBody::SetAngularVelocity(float newAngularVelocity)
{
    if (type == Type::static_body)
    {
        return;
    }

    angularVelocity = newAngularVelocity;
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
        torque += Cross(worldPoint - motion.c, inForce);
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
        torque += Cross(localPoint - motion.localCenter, inForce);
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
        angularVelocity += invInertia * Cross(worldPoint - motion.c, impulse);
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
        angularVelocity += invInertia * Cross(localPoint - motion.localCenter, impulse);
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
    motion.c = Mul(transform, motion.localCenter);
    motion.c0 = motion.c;

    SynchronizeColliders();
}

inline void RigidBody::Rotate(float a)
{
    motion.a += a;
    motion.a0 = motion.a;
    transform.rotation = motion.a;

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
    transform.rotation = motion.a;
    // Shift to origin
    transform.position = motion.c - Mul(transform.rotation, motion.localCenter);
}

// Advance to the new safe time
// c0 = c = motion(alpha)
// a0 = a = motion(alpha)
inline void RigidBody::Advance(float alpha)
{
    motion.Advance(alpha);
    motion.c = motion.c0;
    motion.a = motion.a0;

    transform.rotation = motion.a;
    transform.position = motion.c - Mul(transform.rotation, motion.localCenter);
}

} // namespace muli