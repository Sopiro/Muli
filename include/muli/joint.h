#pragma once

#include "constraint.h"

namespace muli
{

class Joint;
class JointDestroyCallback;

struct JointEdge
{
    Body* other;
    Joint* joint;
    JointEdge* prev;
    JointEdge* next;
};

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
    enum Type : uint8
    {
        grab_joint,
        revolute_joint,
        distance_joint,
        angle_joint,
        weld_joint,
        line_joint,
        prismatic_joint,
        pulley_joint,
        motor_joint,
    };

    // clang-format off
    Joint(
        Joint::Type type,
        Body* bodyA,
        Body* bodyB,
        float jointFrequency,
        float jointDampingRatio
    );
    // clang-format on
    virtual ~Joint();

    virtual bool SolvePositionConstraints(const Timestep& step) override
    {
        MuliNotUsed(step);
        return true;
    }

    float GetJointFrequency() const;
    void SetJointFrequency(float jointFrequency);
    float GetJointDampingRatio() const;
    void SetJointDampingRatio(float jointDampingRatio);

    void SetParameters(float jointFrequency, float jointDampingRatio);

    bool IsSolid() const;
    Joint::Type GetType() const;

    Joint* GetPrev();
    const Joint* GetPrev() const;
    Joint* GetNext();
    const Joint* GetNext() const;

    bool IsEnabled() const;

    JointDestroyCallback* OnDestroy;
    void* UserData;

protected:
    Joint::Type type;

    void ComputeBetaAndGamma(float effectiveMass, float dt);

private:
    // Following parameters are used to soften the joint
    // Frequency values less than or equal to zero make joints rigid
    // 0 < Frequency
    // 0 <= Damping ratio
    float jointFrequency;
    float jointDampingRatio;

    Joint* prev;
    Joint* next;

    JointEdge nodeA;
    JointEdge nodeB;

    bool flagIsland;
};

inline float Joint::GetJointFrequency() const
{
    return jointFrequency;
}

inline void Joint::SetJointFrequency(float newJointFrequency)
{
    SetParameters(newJointFrequency, jointDampingRatio);
}

inline float Joint::GetJointDampingRatio() const
{
    return jointDampingRatio;
}

inline void Joint::SetJointDampingRatio(float newJointDampingRatio)
{
    SetParameters(jointFrequency, newJointDampingRatio);
}

inline bool Joint::IsSolid() const
{
    return jointFrequency <= 0.0f;
}

inline Joint::Type Joint::GetType() const
{
    return type;
}

inline Joint* Joint::GetPrev()
{
    return prev;
}

inline const Joint* Joint::GetPrev() const
{
    return prev;
}

inline Joint* Joint::GetNext()
{
    return next;
}

inline const Joint* Joint::GetNext() const
{
    return next;
}

inline bool Joint::IsEnabled() const
{
    return bodyA->IsEnabled() || bodyB->IsEnabled();
}

} // namespace muli
