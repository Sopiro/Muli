#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

class AngleJoint : public Joint
{
public:
    AngleJoint(RigidBody* bodyA, RigidBody* bodyB, float angleLimit, float frequency, float dampingRatio, float jointMass);

    virtual void Prepare(const Timestep& step) override;
    virtual void SolveVelocityConstraints(const Timestep& step) override;

    float GetJointAngleOffset() const;
    float GetJointAngleLimit() const;

private:
    float angleOffset, angleLimit;

    float m;

    Vec2 bias;
    Vec2 impulseSum;

    void ApplyImpulse(float lambda);
};

inline float AngleJoint::GetJointAngleOffset() const
{
    return angleOffset;
}

inline float AngleJoint::GetJointAngleLimit() const
{
    return angleLimit;
}

} // namespace muli