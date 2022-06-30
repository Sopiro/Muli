#include "rigidbody.h"

using namespace spe;

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

const Node* RigidBody::GetNode()
{
    return node;
}

void RigidBody::Awake()
{
    if (type == Static) return;

    resting = 0;
    sleeping = false;
}

float RigidBody::GetDensity()
{
    return density;
}

float RigidBody::GetMass()
{
    return mass;
}

float RigidBody::GetInverseMass()
{
    return invMass;
}

float RigidBody::GetInertia()
{
    return inertia;
}

float RigidBody::GetInverseInertia()
{
    return invInertia;
}

float RigidBody::GetFriction()
{
    return friction;
}

void RigidBody::SetFriction(float _friction)
{
    friction = std::move(_friction);
}

float RigidBody::GetRestitution()
{
    return restitution;
}

void RigidBody::SetRestitution(float _restitution)
{
    restitution = std::move(_restitution);
}

float RigidBody::GetSurfaceSpeed()
{
    return surfaceSpeed;
}

void RigidBody::SetSurfaceSpeed(float _surfaceSpeed)
{
    surfaceSpeed = std::move(_surfaceSpeed);
}

glm::vec2 RigidBody::GetLinearVelocity()
{
    return linearVelocity;
}

void RigidBody::SetLinearVelocity(glm::vec2 _linearVelocity)
{
    linearVelocity = std::move(_linearVelocity);
}

float RigidBody::GetAngularVelocity()
{
    return angularVelocity;
}

void RigidBody::SetAngularVelocity(float _angularVelocity)
{
    angularVelocity = std::move(_angularVelocity);
}

glm::vec2 RigidBody::GetForce()
{
    return force;
}

void RigidBody::SetForce(glm::vec2 _force)
{
    force = std::move(_force);
}

float RigidBody::GetTorque()
{
    return torque;
}

void RigidBody::SetTorque(float _torque)
{
    torque = std::move(_torque);
}

BodyType RigidBody::GetType()
{
    return type;
}