#include "rigidbody.h"

using namespace spe;

RigidBody::RigidBody(Type _type) :
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

Type RigidBody::GetType()
{
    return type;
}