#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

class AngleJoint : public Joint
{
public:
    AngleJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        float angleOffset,
        float minAngle,
        float maxAngle,
        float frequency,
        float dampingRatio,
        float jointMass
    );

    virtual void Prepare(const Timestep& step) override;
    virtual void SolveVelocityConstraints(const Timestep& step) override;

    float GetJointAngleOffset() const;

    float GetJointAngle() const;
    void SetJointAngle(float newAngle);

    float GetJointMinAngle() const;
    void SetJointMinAngle(float newMinAngle);
    float GetJointMaxAngle() const;
    void SetJointMaxAngle(float newMaxAngle);

private:
    float angleOffset;
    float minAngle, maxAngle;

    float m;

    Vec2 bias;
    Vec2 impulseSum;

    void ApplyImpulse(float lambda);
};

inline float AngleJoint::GetJointAngleOffset() const
{
    return angleOffset;
}

inline float AngleJoint::GetJointAngle() const
{
    return minAngle;
}

inline void AngleJoint::SetJointAngle(float newAngle)
{
    minAngle = newAngle;
    maxAngle = newAngle;
}

inline float AngleJoint::GetJointMinAngle() const
{
    return minAngle;
}

inline void AngleJoint::SetJointMinAngle(float newMinAngle)
{
    minAngle = newMinAngle;
    maxAngle = Max(minAngle, maxAngle);
}

inline float AngleJoint::GetJointMaxAngle() const
{
    return maxAngle;
}

inline void AngleJoint::SetJointMaxAngle(float newMaxAngle)
{
    maxAngle = newMaxAngle;
    minAngle = Min(minAngle, maxAngle);
}
} // namespace muli