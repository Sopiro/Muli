#include "muli/joint.h"
#include "muli/callbacks.h"
#include "muli/world.h"

namespace muli
{

Joint::Joint(Joint::Type type, RigidBody* bodyA, RigidBody* bodyB, float jointFrequency, float jointDampingRatio, float jointMass)
    : Constraint(bodyA, bodyB)
    , OnDestroy{ nullptr }
    , UserData{ nullptr }
    , type{ type }
    , flagIsland{ false }
{
    SetParameters(jointFrequency, jointDampingRatio, jointMass);
}

Joint::~Joint() noexcept
{
    if (OnDestroy)
    {
        OnDestroy->OnJointDestroy(this);
    }
}

void Joint::SetParameters(float newJointFrequency, float newJointDampingRatio, float newJointMass)
{
    // 0 < Frequency
    // 0 <= Damping ratio <= 1
    // 0 < Mass

    if (newJointFrequency > 0.0f)
    {
        jointFrequency = newJointFrequency;
        jointDampingRatio = Clamp(newJointDampingRatio, 0.0f, 1.0f);
        jointMass = Clamp(newJointMass, epsilon, max_value);
    }
    else
    {
        jointFrequency = -1.0f;
        jointDampingRatio = 0.0f;
        jointMass = 0.0f;
    }
}

void Joint::ComputeBetaAndGamma(const Timestep& step)
{
    // If the frequency is less than or equal to zero, make this joint solid
    if (jointFrequency <= 0.0f)
    {
        beta = 1.0f;
        gamma = 0.0f;
    }
    else
    {
        float omega = 2.0f * pi * jointFrequency;
        float d = 2.0f * jointMass * jointDampingRatio * omega; // Damping coefficient
        float k = jointMass * omega * omega;                    // Spring constant
        float h = step.dt;

        beta = h * k / (d + h * k);
        gamma = 1.0f / ((d + h * k) * h);
    }
}

} // namespace muli
