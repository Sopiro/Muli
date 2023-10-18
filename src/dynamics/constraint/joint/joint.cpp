#include "muli/joint.h"
#include "muli/callbacks.h"
#include "muli/world.h"

namespace muli
{

Joint::Joint(Joint::Type _type, RigidBody* _bodyA, RigidBody* _bodyB, float _frequency, float _dampingRatio, float _jointMass)
    : Constraint(_bodyA, _bodyB)
    , OnDestroy{ nullptr }
    , UserData{ nullptr }
    , type{ _type }
    , flagIsland{ false }
{
    SetParameters(_frequency, _dampingRatio, _jointMass);
}

Joint::~Joint() noexcept
{
    if (OnDestroy)
    {
        OnDestroy->OnJointDestroy(this);
    }
}

void Joint::SetParameters(float _frequency, float _dampingRatio, float _jointMass)
{
    if (_frequency > 0.0f)
    {
        frequency = _frequency;
        dampingRatio = Clamp(_dampingRatio, 0.0f, 1.0f);
        jointMass = Clamp(_jointMass, epsilon, max_value);
    }
    else
    {
        frequency = -1.0f;
        dampingRatio = 0.0f;
        jointMass = 0.0f;
    }
}

void Joint::ComputeBetaAndGamma()
{
    // If the frequency is less than or equal to zero, make this joint solid
    if (frequency < 0.0f)
    {
        beta = 1.0f;
        gamma = 0.0f;
    }
    else
    {
        const Timestep& step = bodyA->GetWorld()->GetWorldSettings().step;

        float omega = 2.0f * pi * frequency;
        float d = 2.0f * jointMass * dampingRatio * omega; // Damping coefficient
        float k = jointMass * omega * omega;               // Spring constant
        float h = step.dt;

        beta = h * k / (d + h * k);
        gamma = 1.0f / ((d + h * k) * h);
    }
}

} // namespace muli
