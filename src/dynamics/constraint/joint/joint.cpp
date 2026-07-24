#include "muli/joint.h"
#include "muli/callbacks.h"

namespace muli
{

Joint::Joint(Joint::Type type, Body* bodyA, Body* bodyB, float jointFrequency, float jointDampingRatio)
    : Constraint(bodyA, bodyB)
    , OnDestroy{ nullptr }
    , UserData{ nullptr }
    , type{ type }
    , flagIsland{ false }
{
    SetParameters(jointFrequency, jointDampingRatio);
}

Joint::~Joint()
{
    if (OnDestroy)
    {
        OnDestroy->OnJointDestroy(this);
    }
}

void Joint::SetParameters(float newJointFrequency, float newJointDampingRatio)
{
    if (newJointFrequency > 0.0f)
    {
        jointFrequency = newJointFrequency;
        jointDampingRatio = Max(newJointDampingRatio, 0.0f);
    }
    else
    {
        jointFrequency = -1.0f;
        jointDampingRatio = 0.0f;
    }
}

void Joint::ComputeBetaAndGamma(float effectiveMass, float dt)
{
    // If the frequency is less than or equal to zero, make this joint solid
    if (jointFrequency <= 0.0f || effectiveMass <= 0.0f)
    {
        beta = 1.0f;
        gamma = 0.0f;
    }
    else
    {
        float omega = 2.0f * pi * jointFrequency;
        float d = 2.0f * effectiveMass * jointDampingRatio * omega; // Damping coefficient
        float k = effectiveMass * omega * omega;                    // Spring constant
        float h = dt;

        beta = h * k / (d + h * k);
        gamma = 1.0f / ((d + h * k) * h);
    }
}

} // namespace muli
