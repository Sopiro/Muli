#include "spe/joint.h"

namespace spe
{

Joint::Joint(Joint::Type _type,
             RigidBody* _bodyA,
             RigidBody* _bodyB,
             const WorldSettings& _settings,
             float _frequency,
             float _dampingRatio,
             float _jointMass)
    : Constraint(_bodyA, _bodyB, _settings)
{
    type = _type;
    SetProperties(_frequency, _dampingRatio, _jointMass);
}

Joint::~Joint()
{
    if (OnDestroy != nullptr) OnDestroy(this);
}

void Joint::SetProperties(float _frequency, float _dampingRatio, float _jointMass)
{
    // If the frequency is less than or equal to zero, make this joint solid
    if (_frequency > 0.0f)
    {
        frequency = _frequency;
        dampingRatio = spe::clamp<float>(_dampingRatio, 0.0f, 1.0f);
        jointMass = spe::clamp<float>(_jointMass, 0.0f, FLT_MAX);

        CalculateBetaAndGamma();
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

void Joint::CalculateBetaAndGamma()
{
    float omega = 2.0f * SPE_PI * frequency;
    float d = 2.0f * jointMass * dampingRatio * omega; // Damping coefficient
    float k = jointMass * omega * omega;               // Spring constant
    float h = 1.0f / 144.0f;

    beta = h * k / (d + h * k);
    gamma = 1.0f / ((d + h * k) * h);
}

} // namespace spe
