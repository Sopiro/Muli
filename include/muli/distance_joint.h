#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

struct WorldSettings;

class DistanceJoint : public Joint
{
public:
    DistanceJoint(RigidBody* _bodyA,
                  RigidBody* _bodyB,
                  const Vec2& _anchorA,
                  const Vec2& _anchorB,
                  float _length,
                  const WorldSettings& _settings,
                  float _frequency = 10.0f,
                  float _dampingRatio = 1.0f,
                  float _jointMass = 1.0f);

    virtual void Prepare() override;
    virtual void SolveVelocityConstraint() override;

    const Vec2& GetLocalAnchorA() const;
    const Vec2& GetLocalAnchorB() const;
    float GetJointLength() const;
    void SetJointLength(float _length);

private:
    Vec2 localAnchorA;
    Vec2 localAnchorB;
    float length;

    Vec2 ra;
    Vec2 rb;
    Vec2 d;
    float m;

    float bias;
    float impulseSum = 0.0f;

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

inline void DistanceJoint::SetJointLength(float _length)
{
    length = Clamp<float>(length, 0, FLT_MAX);
}

} // namespace muli