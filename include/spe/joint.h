#pragma once

#include "common.h"
#include "constraint.h"

namespace spe
{

struct Settings;

class Joint : public Constraint
{
    /*
     * Equation of motion for the damped harmonic oscillator
     * a = d²x/dt²
     * v = dx/dt
     *
     * ma + cv + kx = 0
     *
     * c = damping coefficient for springy motion
     * m = mass
     * k = spring constant
     *
     * a + 2ζωv + ω²x = 0
     *
     * ζ = damping ratio
     * ω = angular frequecy
     *
     * 2ζω = c / m
     * ω² = k / m
     *
     * Constraint equation
     * J·v + (β/h)·C(x) + (γ/h)·λ = 0
     *
     * h = dt
     * C(x) = Posiitonal error
     * λ = Corrective impulse
     *
     * β = hk / (c + hk)
     * γ = 1 / (c + hk)
     *
     * More reading:
     * https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf
     * https://pybullet.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
     */
    friend class World;

public:
    enum Type : uint8_t
    {
        JointGrab,
        JointRevolute,
        JointDistance,
    };

    Joint(Joint::Type _type,
          RigidBody* _bodyA,
          RigidBody* _bodyB,
          const Settings& _settings,
          float _frequency = DEFAULT_FREQUENCY,
          float _dampingRatio = DEFAULT_DAMPING_RATIO,
          float _jointMass = DEFAULT_JOINT_MASS);
    ~Joint() noexcept;

    Joint(const Joint&) = delete;
    Joint& operator=(const Joint&) = delete;

    Joint(Joint&&) noexcept = default;
    Joint& operator=(Joint&&) noexcept = default;

    float GetFrequency() const;
    void SetFrequency(float _frequency);
    float GetDampingRatio() const;
    void SetDampingRatio(float _dampingRatio);
    float GetJointMass() const;
    void SetJointMass(float _jointMass);

    bool IsSolid() const;
    Joint::Type GetType() const;

    std::function<void(Joint*)> OnDestroy = nullptr;

protected:
    Joint::Type type;

private:
    // Following parameters are used to soften the joint
    float frequency;
    float dampingRatio;
    float jointMass;

    // 0 < Frequency
    // 0 <= Damping ratio <= 1
    // 0 < Joint mass
    void SetProperties(float _frequency, float _dampingRatio, float _jointMass);
    void CalculateBetaAndGamma();
};

inline Joint::Joint(Joint::Type _type,
                    RigidBody* _bodyA,
                    RigidBody* _bodyB,
                    const Settings& _settings,
                    float _frequency,
                    float _dampingRatio,
                    float _jointMass)
    : Constraint(_bodyA, _bodyB, _settings)
{
    type = _type;
    SetProperties(_frequency, _dampingRatio, _jointMass);
}

inline Joint::~Joint()
{
    if (OnDestroy != nullptr) OnDestroy(this);
}

inline float Joint::GetFrequency() const
{
    return frequency;
}

inline void Joint::SetFrequency(float _frequency)
{
    SetProperties(_frequency, dampingRatio, jointMass);
}

inline float Joint::GetDampingRatio() const
{
    return dampingRatio;
}

inline void Joint::SetDampingRatio(float _dampingRatio)
{
    SetProperties(frequency, _dampingRatio, jointMass);
}

inline float Joint::GetJointMass() const
{
    return jointMass;
}

inline void Joint::SetJointMass(float _jointMass)
{
    SetProperties(frequency, dampingRatio, _jointMass);
}

inline bool Joint::IsSolid() const
{
    return frequency <= 0.0f;
}

inline Joint::Type Joint::GetType() const
{
    return type;
}

inline void Joint::SetProperties(float _frequency, float _dampingRatio, float _jointMass)
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

inline void Joint::CalculateBetaAndGamma()
{
    float omega = 2.0f * glm::pi<float>() * frequency;
    float d = 2.0f * jointMass * dampingRatio * omega; // Damping coefficient
    float k = jointMass * omega * omega;               // Spring constant
    float h = 1.0f / 144.0f;

    beta = h * k / (d + h * k);
    gamma = 1.0f / ((d + h * k) * h);
}

} // namespace spe