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
    , islandID{ 0 }
    , contactList{ nullptr }
    , jointList{ nullptr }
    , resting{ 0.0f }
    , prev{ nullptr }
    , next{ nullptr }
    , localCenter{ 0.0f }
    , transform{ identity }
    , position{ 0.0f }
    , angle{ 0.0f }
    , force{ 0.0f }
    , torque{ 0.0f }
    , linearVelocity{ 0.0f }
    , angularVelocity{ 0.0f }
{
}

RigidBody::~RigidBody()
{
    world = nullptr;

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

void RigidBody::RemoveCollider(Collider* collider)
{
    if (collider == nullptr)
    {
        return;
    }

    muliAssert(collider->body == this);

    Collider* c = colliderList;
    while (c)
    {
        if (c == collider)
        {
            c = collider->next;
            break;
        }

        c = c->next;
    }

    // Destroy any contacts associated with the collider
    ContactEdge* e = contactList;
    while (e)
    {
        Contact* contact = e->contact;
        e = e->next;

        Collider* colliderA = contact->GetColliderA();
        Collider* colliderB = contact->GetColliderB();

        if (collider == colliderA || collider == colliderB)
        {
            world->contactManager.Destroy(contact);
        }
    }

    PredefinedBlockAllocator* allocator = &world->blockAllocator;

    collider->Destroy(allocator);
    collider->body = nullptr;
    collider->next = nullptr;
    collider->~Collider();
    allocator->Free(collider, sizeof(Collider));

    --colliderCount;

    ResetMassData();
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

void RigidBody::RayCastAny(
    const Vec2& from,
    const Vec2& to,
    const std::function<float(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)>& callback) const
{
    RayCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;

    for (Collider* collider = colliderList; collider; collider = collider->next)
    {
        RayCastOutput output;

        bool hit = collider->RayCast(input, &output);
        if (hit)
        {
            float fraction = output.fraction;
            Vec2 point = (1.0f - fraction) * input.from + fraction * input.to;

            callback(collider, point, output.normal, fraction);
        }
    }
}

bool RigidBody::RayCastClosest(
    const Vec2& from,
    const Vec2& to,
    const std::function<float(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)>& callback) const
{
    bool hit = false;
    Collider* closestCollider;
    Vec2 closestPoint;
    Vec2 closestNormal;
    float closestFraction;

    RayCastAny(from, to, [&](Collider* collider, const Vec2& point, const Vec2& normal, float fraction) -> float {
        hit = true;
        closestCollider = collider;
        closestPoint = point;
        closestNormal = normal;
        closestFraction = fraction;

        return fraction;
    });

    if (hit)
    {
        callback(closestCollider, closestPoint, closestNormal, closestFraction);
        return true;
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

    localCenter.SetZero();
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

    Vec2 oldPosition = position;
    position = transform * localCenter;
    linearVelocity += Cross(angularVelocity, position - oldPosition);
}

} // namespace muli