#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

class RevoluteJoint : public Joint
{
public:
    RevoluteJoint(
        RigidBody* bodyA,
        RigidBody* bodyB,
        const Vec2& anchor,
        float frequency = 10.0f,
        float dampingRatio = 1.0f,
        float jointMass = -1.0f
    );

    virtual void Prepare(const Timestep& step) override;
    virtual void SolveVelocityConstraints(const Timestep& step) override;

    const Vec2& GetLocalAnchorA() const;
    const Vec2& GetLocalAnchorB() const;

private:
    Vec2 localAnchorA;
    Vec2 localAnchorB;

    Vec2 ra;
    Vec2 rb;
    Mat2 m;

    Vec2 bias;
    Vec2 impulseSum;

    void ApplyImpulse(const Vec2& lambda);
};

inline const Vec2& RevoluteJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const Vec2& RevoluteJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

} // namespace muli