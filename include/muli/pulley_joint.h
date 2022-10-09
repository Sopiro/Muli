#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

struct WorldSettings;

class PulleyJoint : public Joint
{
public:
    PulleyJoint(RigidBody* _bodyA,
                RigidBody* _bodyB,
                const Vec2& _anchorA,
                const Vec2& _anchorB,
                const Vec2& _groundAnchorA,
                const Vec2& _groundAnchorB,
                const WorldSettings& _settings,
                float _ratio = 1.0f,
                float _frequency = -1.0f,
                float _dampingRatio = 1.0f,
                float _jointMass = 1.0f);

    virtual void Prepare() override;
    virtual void SolveVelocityConstraint() override;

    const Vec2& GetGroundAnchorA() const;
    const Vec2& GetGroundAnchorB() const;
    const Vec2& GetLocalAnchorA() const;
    const Vec2& GetLocalAnchorB() const;
    float GetPulleyLength() const;
    void SetPulleyLength(float _length);

private:
    Vec2 groundAnchorA;
    Vec2 groundAnchorB;
    Vec2 localAnchorA;
    Vec2 localAnchorB;
    float length;
    float ratio;

    Vec2 ra;
    Vec2 rb;
    Vec2 ua;
    Vec2 ub;
    float m;

    float bias;
    float impulseSum = 0.0f;

    void ApplyImpulse(float lambda);
};

inline const Vec2& PulleyJoint::GetGroundAnchorA() const
{
    return groundAnchorA;
}

inline const Vec2& PulleyJoint::GetGroundAnchorB() const
{
    return groundAnchorB;
}

inline const Vec2& PulleyJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const Vec2& PulleyJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

inline float PulleyJoint::GetPulleyLength() const
{
    return length;
}

inline void PulleyJoint::SetPulleyLength(float _length)
{
    length = _length;
}

} // namespace muli