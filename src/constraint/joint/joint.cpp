#include "spe/joint.h"
#include "spe/world.h"

namespace spe
{
Joint::Joint(RigidBody* _bodyA, RigidBody* _bodyB, const Settings& _settings,
    float _frequency,
    float _dampingRatio,
    float _jointMass) :
    Constraint(_bodyA, _bodyB, _settings)
{
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
        dampingRatio = glm::clamp<float>(_dampingRatio, 0.0f, 1.0f);
        jointMass = glm::clamp<float>(_jointMass, 0.0f, FLT_MAX);

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
    float omega = 2.0f * glm::pi<float>() * frequency;
    float d = 2.0f * jointMass * dampingRatio * omega;  // Damping coefficient
    float k = jointMass * omega * omega;                // Spring constant
    float h = 1.0f / 144.0f;

    beta = h * k / (d + h * k);
    gamma = 1.0f / ((d + h * k) * h);
}

float Joint::GetFrequency() const
{
    return frequency;
}

void Joint::SetFrequency(float _frequency)
{
    SetProperties(_frequency, dampingRatio, jointMass);
}

float Joint::GetDampingRatio() const
{
    return dampingRatio;
}

void Joint::SetDampingRatio(float _dampingRatio)
{
    SetProperties(frequency, _dampingRatio, jointMass);
}

float Joint::GetJointMass() const
{
    return jointMass;
}

void Joint::SetJointMass(float _jointMass)
{
    SetProperties(frequency, dampingRatio, _jointMass);
}

bool Joint::IsSolid() const
{
    return frequency <= 0.0f;
}

}
