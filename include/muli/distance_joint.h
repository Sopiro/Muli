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
        float length,
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

private:
    Vec2 localAnchorA;
    Vec2 localAnchorB;
    float length;

    Vec2 ra;
    Vec2 rb;
    Vec2 d;
    float m;

    float bias;
    float impulseSum;

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
    return length;
}

inline void DistanceJoint::SetJointLength(float newLength)
{
    length = Clamp<float>(newLength, 0, max_value);
}

} // namespace muli