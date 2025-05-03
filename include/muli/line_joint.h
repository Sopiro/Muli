#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

class LineJoint : public Joint
{
public:
    LineJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec2& anchor,
        const Vec2& dir,
        float frequency,
        float dampingRatio,
        float jointMass
    );

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