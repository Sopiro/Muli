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
    Joint(
        RigidBody* _bodyA, RigidBody* _bodyB, const Settings& _settings,
        float _frequency = DEFAULT_FREQUENCY,
        float _dampingRatio = DEFAULT_DAMPING_RATIO,
        float _jointMass = DEFAULT_JOINT_MASS
    );
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

    std::function<void(Joint*)> OnDestroy = nullptr;

private:
    uint32_t id{ 0 };

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
}