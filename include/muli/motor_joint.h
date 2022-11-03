#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

struct WorldSettings;

class MotorJoint : public Joint
{
public:
    MotorJoint(RigidBody* _bodyA,
               RigidBody* _bodyB,
               const Vec2& _anchor,
               const WorldSettings& _settings,
               float _maxForce = 1000.0f,
               float _maxTorque = 1000.0f,
               float _frequency = -1.0f,
               float _dampingRatio = 1.0f,
               float _jointMass = 1.0f);

    virtual void Prepare() override;
    virtual void SolveVelocityConstraint() override;

    const Vec2& GetLocalAnchorA() const;
    const Vec2& GetLocalAnchorB() const;
    float GetMaxForce() const;
    void SetMaxForce(float _maxForce);
    float GetMaxTorque() const;
    void SetMaxTorque(float _maxTorque);
    const Vec2& GetLinearOffset() const;
    void SetLinearOffset(const Vec2& _linearOffset);
    const Vec2& GetAngularOffset() const;
    void SetAngularOffset(float _angularOffset);

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

    Vec2 linearImpulseSum{ 0.0f };
    float angularImpulseSum = 0.0f;

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

inline void MotorJoint::SetMaxForce(float _maxForce)
{
    maxForce = _maxForce;
}

inline float MotorJoint::GetMaxTorque() const
{
    return maxTorque;
}

inline void MotorJoint::SetMaxTorque(float _maxTorque)
{
    maxTorque = _maxTorque;
}

inline const Vec2& MotorJoint::GetLinearOffset() const
{
    return linearOffset;
}

inline void MotorJoint::SetLinearOffset(const Vec2& _linearOffset)
{
    linearOffset = _linearOffset;
}

inline const Vec2& MotorJoint::GetAngularOffset() const
{
    return angularOffset;
}

inline void MotorJoint::SetAngularOffset(float _angularOffset)
{
    angularOffset = _angularOffset;
}

} // namespace muli