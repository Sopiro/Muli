#include "muli/joint.h"

namespace muli
{

Joint::Joint(Joint::Type _type,
             RigidBody* _bodyA,
             RigidBody* _bodyB,
             const WorldSettings& _settings,
             float _frequency,
             float _dampingRatio,
             float _jointMass)
    : Constraint(_bodyA, _bodyB, _settings)
    , OnDestroy{ nullptr }
{
    type = _type;
    SetProperties(_frequency, _dampingRatio, _jointMass);
}

Joint::~Joint() noexcept
{
    if (OnDestroy)
    {
        OnDestroy(this);
    }
}

void Joint::SetProperties(float _frequency, float _dampingRatio, float _jointMass)
{
    // If the frequency is less than or equal to zero, make this joint solid
    if (_frequency > 0.0f)
    {
        frequency = _frequency;
        dampingRatio = Clamp<float>(_dampingRatio, 0.0f, 1.0f);
        jointMass = Clamp<float>(_jointMass, 0.0f, FLT_MAX);

        ComputeBetaAndGamma();
    }
    else
    {
        frequency = -1.0f;
        dampingRatio = 0.0f;
        jointMass = 0.0f;

        beta = 1.0f;
        gamma = 0.0f;
    }
}

void Joint::ComputeBetaAndGamma()
{
    float omega = 2.0f * MULI_PI * frequency;
    float d = 2.0f * jointMass * dampingRatio * omega; // Damping coefficient
    float k = jointMass * omega * omega;               // Spring constant
    float h = settings.DT;

    beta = h * k / (d + h * k);
    gamma = 1.0f / ((d + h * k) * h);
}

} // namespace muli
