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

void Joint::SetProperties(float _frequency, float _dampingRatio, float _jointMass)
{
    if (_frequency > 0)
    {
        frequency = _frequency;
        dampingRatio = glm::clamp<float>(_dampingRatio, 0.0f, 1.0f);
        jointMass = _jointMass <= 0 ? bodyB->mass : _jointMass;

        CalculateBetaAndGamma();
    }
    else
    {
        // If the frequency is less than or equal to zero, make this joint solid
        frequency = -1;
        dampingRatio = 1.0;
        jointMass = -1;

        beta = 1.0f;
        gamma = 0.0f;
    }
}

void Joint::CalculateBetaAndGamma()
{
    float omega = 2.0f * glm::pi<float>() * frequency;
    float d = 2.0f * jointMass * dampingRatio * omega;  // Damping coefficient
    float k = jointMass * omega * omega;                // Spring constant
    float h = 1 / 144.0f;

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
