#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

class MotorJoint : public Joint
{
public:
    MotorJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec2& anchor,
        float maxJointForce,
        float maxJointTorque,
        float frequency,
        float dampingRatio,
        float jointMass
    );

    virtual void Prepare(const Timestep& step) override;
    virtual void SolveVelocityConstraints(const Timestep& step) override;

    const Vec2& GetLocalAnchorA() const;
    const Vec2& GetLocalAnchorB() const;
    float GetMaxForce() const;
    void SetMaxForce(float maxForce);
    float GetMaxTorque() const;
    void SetMaxTorque(float maxTorque);
    const Vec2& GetLinearOffset() const;
    void SetLinearOffset(const Vec2& linearOffset);
    float GetAngularOffset() const;
    void SetAngularOffset(float angularOffset);

private:
    Vec2 localAnchorA;
    Vec2 localAnchorB;
    float angleOffset; // Initial angle offset

    Vec2 linearOffset;
    float angularOffset;

    float maxForce;
    float maxTorque;

    Vec2 ra;
    Vec2 rb;
    Mat2 m0;
    float m1;

    Vec2 bias0;
    float bias1;

    Vec2 linearImpulseSum;
    float angularImpulseSum;

    void ApplyImpulse(const Vec2& lambda0, float lambda1);
};

inline const Vec2& MotorJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const Vec2& MotorJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

inline float MotorJoint::GetMaxForce() const
{
    return maxForce;
}

inline void MotorJoint::SetMaxForce(float newMaxForce)
{
    maxForce = newMaxForce;
}

inline float MotorJoint::GetMaxTorque() const
{
    return maxTorque;
}

inline void MotorJoint::SetMaxTorque(float newMaxTorque)
{
    maxTorque = newMaxTorque;
}

inline const Vec2& MotorJoint::GetLinearOffset() const
{
    return linearOffset;
}

inline void MotorJoint::SetLinearOffset(const Vec2& newLinearOffset)
{
    linearOffset = newLinearOffset;
}

inline float MotorJoint::GetAngularOffset() const
{
    return angularOffset;
}

inline void MotorJoint::SetAngularOffset(float newAngularOffset)
{
    angularOffset = newAngularOffset;
}

} // namespace muli