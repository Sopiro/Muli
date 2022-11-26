#include "muli/rigidbody.h"
#include "muli/aabbtree.h"
#include "muli/collider.h"
#include "muli/world.h"

namespace muli
{

RigidBody::RigidBody(RigidBody::Type _type)
    : type{ _type }
    , flag{ 0 }
    , userFlag{ 0 }
    , mass{ 0.0f }
    , invMass{ 0.0f }
    , inertia{ 0.0f }
    , invInertia{ 0.0f }
    , colliderList{ nullptr }
    , colliderCount{ 0 }
    , world{ nullptr }
    , id{ 0 }
    , islandID{ 0 }
    , contactList{ nullptr }
    , jointList{ nullptr }
    , resting{ 0.0f }
    , prev{ nullptr }
    , next{ nullptr }
    , transform{ identity }
    , force{ 0.0f }
    , torque{ 0.0f }
    , linearVelocity{ 0.0f }
    , angularVelocity{ 0.0f }
{
}

RigidBody::~RigidBody()
{
    world = nullptr;
    id = 0;

    if (OnDestroy != nullptr)
    {
        OnDestroy(this);
    }
}

void RigidBody::NotifyForceUpdate() const
{
    if (world)
    {
        world->integrateForce = true;
    }
}

Collider* RigidBody::AddCollider(Shape* _shape, float _density, const Material& _material)
{
    muliAssert(world != nullptr);

    PredefinedBlockAllocator* allocator = &world->blockAllocator;
    void* mem = allocator->Allocate(sizeof(Collider));

    Collider* collider = new (mem) Collider;
    collider->Create(allocator, this, _shape, _density, _material);

    collider->next = colliderList;
    colliderList = collider;
    ++colliderCount;

    collider->body->world->contactManager.Add(collider);

    ResetMassData();

    return collider;
}

bool RigidBody::TestPoint(const Vec2& p) const
{
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
    Vec2 closestPoint;

    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        closestPoint = collider->GetClosestPoint(p);
        if (closestPoint == p)
        {
            return closestPoint;
        }
    }

    return closestPoint;
}

bool RigidBody::RayCast(const RayCastInput& input, RayCastOutput* output) const
{
    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        if (collider->RayCast(input, output))
        {
            return true;
        }
    }

    return false;
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
        // Center the inertia about the center of mass.
        inertia -= mass * Length2(localCenter);
        invInertia = 1.0f / inertia;
    }
    else
    {
        inertia = 0.0f;
        invInertia = 0.0f;
    }
}

} // namespace muli