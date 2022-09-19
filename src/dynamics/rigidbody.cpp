#include "spe/rigidbody.h"
#include "spe/world.h"

namespace spe
{

RigidBody::RigidBody(RigidBody::Type _type, RigidBody::Shape _shape)
    : type{ _type }
    , shape{ _shape }
{
    if (type == Static)
    {
        density = 0.0f;
        mass = 0.0f;
        invMass = 0.0f;
        inertia = 0.0f;
        invInertia = 0.0f;
    }
    else
    {
        // This part is implemented by children.
    }
}

RigidBody::~RigidBody()
{
    if (moved) return;

    world = nullptr;
    id = 0;

    if (OnDestroy != nullptr)
    {
        OnDestroy(this);
    }
}

RigidBody::RigidBody(RigidBody&& _other) noexcept
{
    _other.moved = true;

    // private member
    world = _other.world;
    id = _other.id;
    islandID = _other.islandID;

    contactList = _other.contactList;
    _other.contactList = nullptr;
    jointList = _other.jointList;
    _other.jointList = nullptr;

    resting = _other.resting;
    sleeping = _other.sleeping;

    node = _other.node;
    _other.node = nullptr;
    prev = _other.prev;
    _other.prev = nullptr;
    next = _other.next;
    _other.next = nullptr;

    // protected member
    transform = _other.transform;

    force = _other.force;
    torque = _other.torque;

    linearVelocity = _other.linearVelocity;
    angularVelocity = _other.angularVelocity;

    density = _other.density;
    mass = _other.mass;
    invMass = _other.invMass;
    inertia = _other.inertia;
    invInertia = _other.invInertia;

    friction = _other.friction;
    restitution = _other.restitution;
    surfaceSpeed = _other.surfaceSpeed;

    shape = _other.shape;
    type = _other.type;

    // public member
    OnDestroy = std::move(_other.OnDestroy);
}

void RigidBody::NotifyForceUpdate() const
{
    if (world)
    {
        world->integrateForce = true;
    }
}

} // namespace spe