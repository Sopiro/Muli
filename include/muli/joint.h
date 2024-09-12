#pragma once

#include "common.h"
#include "constraint.h"

namespace muli
{

class Joint;
class JointDestroyCallback;

struct JointEdge
{
    RigidBody* other;
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

    Joint(
        Joint::Type type,
        RigidBody* bodyA,
        RigidBody* bodyB,
        float frequency = default_joint_frequency,
        float dampingRatio = default_joint_damping_ratio,
        float jointMass = default_joint_mass
    );
    virtual ~Joint() noexcept;

    virtual bool SolvePositionConstraints(const Timestep& step) override
    {
        MuliNotUsed(step);
        return true;
    }

    float GetFrequency() const;
    void SetFrequency(float frequency);
    float GetDampingRatio() const;
    void SetDampingRatio(float dampingRatio);
    float GetJointMass() const;
    void SetJointMass(float jointMass);

    // 0.0 < Frequency
    // 0.0 <= Damping ratio <= 1.0
    // 0.0 < Joint mass
    void SetParameters(float frequency, float dampingRatio, float jointMass);

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

    void ComputeBetaAndGamma(const Timestep& step);

private:
    // Following parameters are used to soften the joint
    float frequency;
    float dampingRatio;
    float jointMass;

    Joint* prev;
    Joint* next;

    JointEdge nodeA;
    JointEdge nodeB;

    bool flagIsland;
};

inline float Joint::GetFrequency() const
{
    return frequency;
}

inline void Joint::SetFrequency(float newFrequency)
{
    SetParameters(newFrequency, dampingRatio, jointMass);
}

inline float Joint::GetDampingRatio() const
{
    return dampingRatio;
}

inline void Joint::SetDampingRatio(float newDampingRatio)
{
    SetParameters(frequency, newDampingRatio, jointMass);
}

inline float Joint::GetJointMass() const
{
    return jointMass;
}

inline void Joint::SetJointMass(float newJointMass)
{
    SetParameters(frequency, dampingRatio, newJointMass);
}

inline bool Joint::IsSolid() const
{
    return frequency <= 0.0f;
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