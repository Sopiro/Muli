#include "muli/rigidbody.h"
#include "muli/aabbtree.h"
#include "muli/capsule.h"
#include "muli/circle.h"
#include "muli/collider.h"
#include "muli/polygon.h"
#include "muli/world.h"

namespace muli
{

RigidBody::RigidBody(RigidBody::Type _type)
    : type{ _type }
    , flag{ 0 }
    , world{ nullptr }
    , islandID{ 0 }
    , resting{ 0.0f }
    , prev{ nullptr }
    , next{ nullptr }
    , colliderList{ nullptr }
    , colliderCount{ 0 }
    , contactList{ nullptr }
    , jointList{ nullptr }
    , mass{ 0.0f }
    , invMass{ 0.0f }
    , inertia{ 0.0f }
    , invInertia{ 0.0f }
    , localCenter{ 0.0f }
    , transform{ identity }
    , position{ 0.0f }
    , angle{ 0.0f }
    , force{ 0.0f }
    , torque{ 0.0f }
    , linearVelocity{ 0.0f }
    , angularVelocity{ 0.0f }
    , UserFlag{ 0 }
    , OnDestroy{ nullptr }
{
}

RigidBody::~RigidBody() noexcept
{
    if (OnDestroy)
    {
        OnDestroy(this);
    }

    world = nullptr;
}

void RigidBody::NotifyForceUpdate() const
{
    if (world)
    {
        world->integrateForce = true;
    }
}

Collider* RigidBody::CreateCollider(Shape* _shape, float _density, const Material& _material)
{
    muliAssert(world != nullptr);

    Allocator* allocator = &world->blockAllocator;
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

void RigidBody::DestoryCollider(Collider* collider)
{
    if (collider == nullptr)
    {
        return;
    }

    muliAssert(collider->body == this);
    muliAssert(colliderCount > 0);

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
    world->contactManager.Remove(collider);

    Allocator* allocator = &world->blockAllocator;

    collider->~Collider();
    collider->Destroy(allocator);
    allocator->Free(collider, sizeof(Collider));

    --colliderCount;

    ResetMassData();
}

Collider* RigidBody::CreateCircleCollider(float radius, const Vec2& position, float density, const Material& material)
{
    Circle circle{ radius, position };
    return CreateCollider(&circle, density, material);
}

Collider* RigidBody::CreateBoxCollider(
    float width, float height, float radius, const Vec2& position, float angle, float density, const Material& material)
{
    Polygon box{ width, height, radius, position, angle };
    return CreateCollider(&box, density, material);
}

Collider* RigidBody::CreateCapsuleCollider(
    float length, float radius, bool horizontal, const Vec2& position, float density, const Material& material)
{
    Capsule capsule{ length, radius, horizontal, position };
    return CreateCollider(&capsule, density, material);
}

Collider* RigidBody::CreateCapsuleCollider(
    const Vec2& p1, const Vec2& p2, float radius, bool resetPosition, float density, const Material& material)
{
    Capsule capsule{ p1, p2, radius, resetPosition };
    return CreateCollider(&capsule, density, material);
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
    localCenter.SetZero();

    if (type == static_body || type == kinematic_body)
    {
        return;
    }

    if (colliderCount <= 0)
    {
        return;
    }

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