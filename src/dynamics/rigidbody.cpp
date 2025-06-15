#include "muli/rigidbody.h"
#include "muli/capsule.h"
#include "muli/circle.h"
#include "muli/collider.h"
#include "muli/polygon.h"
#include "muli/world.h"

namespace muli
{

RigidBody::RigidBody(const Transform& tf, RigidBody::Type type)
    : OnDestroy{ nullptr }
    , UserData{ nullptr }
    , type{ type }
    , transform{ tf }
    , motion{ tf }
    , force{ 0.0f }
    , torque{ 0.0f }
    , linearVelocity{ 0.0f }
    , angularVelocity{ 0.0f }
    , mass{ 0.0f }
    , invMass{ 0.0f }
    , inertia{ 0.0f }
    , invInertia{ 0.0f }
    , linearDamping{ 0.0f }
    , angularDamping{ 0.0f }
    , islandIndex{ 0 }
    , islandID{ 0 }
    , flag{ flag_enabled }
    , world{ nullptr }
    , prev{ nullptr }
    , next{ nullptr }
    , colliderList{ nullptr }
    , colliderCount{ 0 }
    , contactList{ nullptr }
    , jointList{ nullptr }
    , resting{ 0.0f }
{
}

RigidBody::~RigidBody()
{
    if (OnDestroy)
    {
        OnDestroy->OnBodyDestroy(this);
    }

    world = nullptr;
}

Collider* RigidBody::CreateCollider(Shape* shape, const Transform& tf, float density, const Material& material)
{
    MuliAssert(world != nullptr);
    if (world == nullptr)
    {
        return nullptr;
    }

    Allocator* allocator = &world->blockAllocator;
    void* mem = allocator->Allocate(sizeof(Collider));

#if 1
    // Shape radius(skin) must be greater than or equal to linear_slop * 2.0 for stable CCD
    MuliAssert(shape->radius >= linear_slop * 2.0f);
#endif

    Collider* collider = new (mem) Collider;
    collider->Create(allocator, this, shape, tf, density, material);

    collider->next = colliderList;
    colliderList = collider;
    ++colliderCount;

    world->contactGraph.AddCollider(collider);

    ResetMassData();

    return collider;
}

void RigidBody::DestroyCollider(Collider* collider)
{
    if (collider == nullptr)
    {
        return;
    }

    MuliAssert(collider->body == this);
    MuliAssert(colliderCount > 0);

    // Remove collider from collider list
    Collider** c = &colliderList;
    while (*c)
    {
        if (*c == collider)
        {
            *c = collider->next;
            break;
        }

        c = &((*c)->next);
    }

    // Remove collider from contact manager(broad phase)
    world->contactGraph.RemoveCollider(collider);

    Allocator* allocator = &world->blockAllocator;

    collider->~Collider();
    collider->Destroy(allocator);
    allocator->Free(collider, sizeof(Collider));

    --colliderCount;

    ResetMassData();
}

Collider* RigidBody::CreateCircleCollider(float radius, const Transform& tf, float density, const Material& material)
{
    Circle circle{ radius };
    return CreateCollider(&circle, tf, density, material);
}

Collider* RigidBody::CreateBoxCollider(
    float width, float height, float radius, const Transform& tf, float density, const Material& material
)
{
    Polygon box{ width, height, radius };
    return CreateCollider(&box, tf, density, material);
}

Collider* RigidBody::CreateCapsuleCollider(
    float length, float radius, bool horizontal, const Transform& tf, float density, const Material& material
)
{
    Capsule capsule{ length, radius, horizontal };
    return CreateCollider(&capsule, tf, density, material);
}

Collider* RigidBody::CreateCapsuleCollider(
    const Vec2& p1, const Vec2& p2, float radius, bool resetPosition, const Transform& tf, float density, const Material& material
)
{
    Capsule capsule{ p1, p2, radius, resetPosition };
    return CreateCollider(&capsule, tf, density, material);
}

bool RigidBody::TestPoint(const Vec2& p) const
{
    MuliAssert(colliderCount > 0);

    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        if (collider->TestPoint(p))
        {
            return true;
        }
    }

    return false;
}

Vec2 RigidBody::GetClosestPoint(const Vec2& p) const
{
    MuliAssert(colliderCount > 0);

    Vec2 cp0 = colliderList->GetClosestPoint(p);
    if (cp0 == p)
    {
        return cp0;
    }

    float d0 = Dist2(cp0, p);

    for (Collider* collider = colliderList->next; collider; collider = collider->next)
    {
        Vec2 cp1 = collider->GetClosestPoint(p);
        if (cp1 == p)
        {
            return cp1;
        }

        float d1 = Dist2(cp1, p);
        if (d1 < d0)
        {
            cp0 = cp1;
        }
    }

    return cp0;
}

void RigidBody::RayCastAny(const Vec2& from, const Vec2& to, float radius, RayCastAnyCallback* callback) const
{
    RayCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;
    input.radius = radius;

    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        RayCastOutput output;

        bool hit = collider->RayCast(input, &output);
        if (hit)
        {
            float fraction = output.fraction;
            Vec2 point = (1.0f - fraction) * input.from + fraction * input.to;

            input.maxFraction = callback->OnHitAny(collider, point, output.normal, fraction);
        }

        if (input.maxFraction <= 0)
        {
            return;
        }
    }
}

bool RigidBody::RayCastClosest(const Vec2& from, const Vec2& to, float radius, RayCastClosestCallback* callback) const
{
    struct TempCallback : public RayCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider = nullptr;
        Vec2 closestPoint;
        Vec2 closestNormal;
        float closestFraction = 1.0f;

        float OnHitAny(Collider* collider, Vec2 point, Vec2 normal, float fraction)
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestFraction = fraction;

            return fraction;
        }
    } tempCallback;

    RayCastAny(from, to, radius, &tempCallback);

    if (tempCallback.hit)
    {
        callback->OnHitClosest(
            tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal, tempCallback.closestFraction
        );
        return true;
    }

    return false;
}

void RigidBody::RayCastAny(
    const Vec2& from,
    const Vec2& to,
    float radius,
    std::function<float(Collider* collider, Vec2 point, Vec2 normal, float fraction)> callback
) const
{
    RayCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;
    input.radius = radius;

    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        RayCastOutput output;

        bool hit = collider->RayCast(input, &output);
        if (hit)
        {
            float fraction = output.fraction;
            Vec2 point = (1.0f - fraction) * input.from + fraction * input.to;

            input.maxFraction = callback(collider, point, output.normal, fraction);
        }

        if (input.maxFraction <= 0)
        {
            return;
        }
    }
}

bool RigidBody::RayCastClosest(
    const Vec2& from,
    const Vec2& to,
    float radius,
    std::function<void(Collider* collider, Vec2 point, Vec2 normal, float fraction)> callback
) const
{
    struct TempCallback : public RayCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider = nullptr;
        Vec2 closestPoint;
        Vec2 closestNormal;
        float closestFraction = 1.0f;

        float OnHitAny(Collider* collider, Vec2 point, Vec2 normal, float fraction)
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestFraction = fraction;

            return fraction;
        }
    } tempCallback;

    RayCastAny(from, to, radius, &tempCallback);

    if (tempCallback.hit)
    {
        callback(
            tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal, tempCallback.closestFraction
        );
        return true;
    }

    return false;
}

void RigidBody::SetType(RigidBody::Type newType)
{
    if (type == newType)
    {
        return;
    }

    type = newType;

    ResetMassData();

    force.SetZero();
    torque = 0.0f;
    if (type == Type::static_body)
    {
        linearVelocity.SetZero();
        angularVelocity = 0.0f;
        motion.c0 = motion.c;
        motion.a0 = motion.a;
        SynchronizeColliders();
    }

    Awake();

    // Refresh the broad phase contacts
    ContactEdge* ce = contactList;
    while (ce)
    {
        ContactEdge* ce0 = ce;
        ce = ce->next;
        world->contactGraph.Destroy(ce0->contact);
    }
    contactList = nullptr;

    for (Collider* c = colliderList; c; c = c->next)
    {
        world->contactGraph.broadPhase.Refresh(c);
    }

    islandID = 0;
    islandIndex = 0;
}

void RigidBody::SetEnabled(bool enabled)
{
    if (enabled == IsEnabled())
    {
        return;
    }

    if (enabled)
    {
        flag |= flag_enabled;

        for (Collider* c = colliderList; c; c = c->next)
        {
            world->contactGraph.broadPhase.Add(c, c->GetAABB());
        }
    }
    else
    {
        flag &= ~flag_enabled;

        ContactEdge* ce = contactList;
        while (ce)
        {
            ContactEdge* ce0 = ce;
            ce = ce->next;
            world->contactGraph.Destroy(ce0->contact);
        }
        contactList = nullptr;

        for (Collider* c = colliderList; c; c = c->next)
        {
            world->contactGraph.broadPhase.Remove(c);
        }

        islandID = 0;
        islandIndex = 0;
    }
}

void RigidBody::SetCollisionFilter(const CollisionFilter& filter) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetFilter(filter);
    }
}

void RigidBody::SetFriction(float friction) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetFriction(friction);
    }
}

void RigidBody::SetRestitution(float restitution) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetRestitution(restitution);
    }
}

void RigidBody::SetRestitutionThreshold(float threshold) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetRestitutionTreshold(threshold);
    }
}

void RigidBody::SetSurfaceSpeed(float surfaceSpeed) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        collider->SetSurfaceSpeed(surfaceSpeed);
    }
}

void RigidBody::ResetMassData()
{
    mass = 0.0f;
    invMass = 0.0f;
    inertia = 0.0f;
    invInertia = 0.0f;

    if (type == static_body || type == kinematic_body)
    {
        return;
    }

    if (colliderCount <= 0)
    {
        return;
    }

    Vec2 localCenter{ 0.0f };

    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        MassData massData = collider->GetMassData();
        mass += massData.mass;
        localCenter += massData.mass * massData.centerOfMass;
        inertia += massData.inertia;
    }

    if (mass > 0.0f)
    {
        invMass = 1.0f / mass;
        localCenter *= invMass;
    }

    if (inertia > 0.0f && (flag & flag_fixed_rotation) == 0)
    {
        // Center the inertia about the center of mass
        inertia -= mass * Length2(localCenter);
        invInertia = 1.0f / inertia;
    }
    else
    {
        inertia = 0.0f;
        invInertia = 0.0f;
    }

    Vec2 oldPosition = motion.c;
    motion.localCenter = localCenter;
    motion.c = Mul(transform, motion.localCenter);
    motion.c0 = motion.c;

    linearVelocity += Cross(angularVelocity, motion.c - oldPosition);
}

void RigidBody::SynchronizeColliders()
{
    if (IsSleeping())
    {
        for (Collider* collider = colliderList; collider; collider = collider->next)
        {
            world->contactGraph.UpdateCollider(collider, transform);
        }
    }
    else
    {
        // Transform at previus step
        Transform tf0;
        motion.GetTransform(0.0f, &tf0);

        for (Collider* collider = colliderList; collider; collider = collider->next)
        {
            world->contactGraph.UpdateCollider(collider, tf0, transform);
        }
    }
}

} // namespace muli