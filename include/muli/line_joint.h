#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

class LineJoint : public Joint
{
public:
    LineJoint(RigidBody* bodyA,
              RigidBody* bodyB,
              Vec2 anchor,
              Vec2 dir,
              float frequency = 10.0f,
              float dampingRatio = 1.0f,
              float jointMass = -1.0f);

    virtual void Prepare(const Timestep& step) override;
    virtual void SolveVelocityConstraints(const Timestep& step) override;

    const Vec2& GetLocalAnchorA() const;
    const Vec2& GetLocalAnchorB() const;

private:
    Vec2 localAnchorA;
    Vec2 localAnchorB;
    Vec2 localYAxis;

    Vec2 t;
    float sa;
    float sb;

    float m;

    float bias;
    float impulseSum;

    void ApplyImpulse(float lambda);
};

inline const Vec2& LineJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const Vec2& LineJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

} // namespace muli