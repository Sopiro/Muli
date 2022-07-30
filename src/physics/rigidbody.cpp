#include "spe/physics/rigidbody.h"

namespace spe
{

RigidBody::RigidBody(BodyType _type) :
    Entity(),
    type{ std::move(_type) }
{
    if (type == Static)
    {
        density = FLT_MAX;
        mass = FLT_MAX;
        invMass = 0.0f;
        inertia = FLT_MAX;
        invInertia = 0.0f;
        sleeping = true;
    }
    else
    {
        // This part is implemented by children.
    }
}

RigidBody::~RigidBody()
{
    if (moved) return;
}

RigidBody::RigidBody(RigidBody&& _other) noexcept
{
    *this = std::move(_other);
}

RigidBody& RigidBody::operator=(RigidBody&& _other) noexcept
{
    if (this == &_other) return *this;

    _other.moved = true;

    //private member
    id = _other.id;
    islandID = _other.islandID;

    manifoldIDs = std::move(_other.manifoldIDs);
    jointIDs = std::move(_other.jointIDs);

    resting = _other.resting;
    sleeping = _other.sleeping;

    node = _other.node;
    _other.node = nullptr;

    //protected member
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

    type = _other.type;

    Entity::operator=(std::move(_other));

    return *this;
}

const Node* RigidBody::GetNode() const
{
    return node;
}

void RigidBody::Awake()
{
    if (type == Static) return;

    resting = 0.0f;
    sleeping = false;
}

float RigidBody::GetDensity() const
{
    return density;
}

float RigidBody::GetMass() const
{
    return mass;
}

float RigidBody::GetInverseMass() const
{
    return invMass;
}

float RigidBody::GetInertia() const
{
    return inertia;
}

float RigidBody::GetInverseInertia() const
{
    return invInertia;
}

float RigidBody::GetFriction() const
{
    return friction;
}

void RigidBody::SetFriction(float _friction)
{
    friction = std::move(_friction);
}

float RigidBody::GetRestitution() const
{
    return restitution;
}

void RigidBody::SetRestitution(float _restitution)
{
    restitution = std::move(_restitution);
}

float RigidBody::GetSurfaceSpeed() const
{
    return surfaceSpeed;
}

void RigidBody::SetSurfaceSpeed(float _surfaceSpeed)
{
    surfaceSpeed = std::move(_surfaceSpeed);
}

glm::vec2 RigidBody::GetLinearVelocity() const
{
    return linearVelocity;
}

void RigidBody::SetLinearVelocity(glm::vec2 _linearVelocity)
{
    linearVelocity = std::move(_linearVelocity);
}

float RigidBody::GetAngularVelocity() const
{
    return angularVelocity;
}

void RigidBody::SetAngularVelocity(float _angularVelocity)
{
    angularVelocity = std::move(_angularVelocity);
}

glm::vec2 RigidBody::GetForce() const
{
    return force;
}

void RigidBody::SetForce(glm::vec2 _force)
{
    force = std::move(_force);
}

float RigidBody::GetTorque() const
{
    return torque;
}

void RigidBody::SetTorque(float _torque)
{
    torque = std::move(_torque);
}

BodyType RigidBody::GetType() const
{
    return type;
}

bool RigidBody::IsSleeping() const
{
    return sleeping;
}

int32_t RigidBody::GetID() const
{
    return id;
}

int32_t RigidBody::GetIslandID() const
{
    return islandID;
}

}