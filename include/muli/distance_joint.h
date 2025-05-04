#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

class DistanceJoint : public Joint
{
public:
    DistanceJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec2& anchorA,
        const Vec2& anchorB,
        float minLength,
        float maxLength,
        float frequency,
        float dampingRatio,
        float jointMass
    );

    virtual void Prepare(const Timestep& step) override;
    virtual void SolveVelocityConstraints(const Timestep& step) override;

    const Vec2& GetLocalAnchorA() const;
    const Vec2& GetLocalAnchorB() const;
    float GetJointLength() const;
    void SetJointLength(float newLength);
    float GetJointMinLength() const;
    void SetJointMinLength(float newMinLength);
    float GetJointMaxLength() const;
    void SetJointMaxLength(float newMaxLength);

private:
    Vec2 localAnchorA;
    Vec2 localAnchorB;
    float minLength, maxLength;

    Vec2 ra;
    Vec2 rb;
    Vec2 d;
    float m;

    Vec2 bias;
    Vec2 impulseSum;

    void ApplyImpulse(float lambda);
};

inline const Vec2& DistanceJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const Vec2& DistanceJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

inline float DistanceJoint::GetJointLength() const
{
    return minLength;
}

inline void DistanceJoint::SetJointLength(float newLength)
{
    minLength = Max(newLength, 0.0f);
    maxLength = minLength;
}

inline float DistanceJoint::GetJointMinLength() const
{
    return minLength;
}

inline void DistanceJoint::SetJointMinLength(float newMinLength)
{
    minLength = Max(newMinLength, 0.0f);
    maxLength = Max(minLength, maxLength);
}

inline float DistanceJoint::GetJointMaxLength() const
{
    return maxLength;
}

inline void DistanceJoint::SetJointMaxLength(float newMaxLength)
{
    maxLength = Max(newMaxLength, 0.0f);
    minLength = Min(minLength, maxLength);
}

} // namespace muli