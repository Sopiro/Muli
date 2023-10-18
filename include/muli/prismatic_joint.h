#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

struct WorldSettings;

class PrismaticJoint : public Joint
{
public:
    PrismaticJoint(RigidBody* bodyA,
                   RigidBody* bodyB,
                   const Vec2& anchor,
                   const Vec2& dir,
                   float frequency = -1.0f,
                   float dampingRatio = 1.0f,
                   float jointMass = 1.0f);

    virtual void Prepare() override;
    virtual void SolveVelocityConstraints() override;

    const Vec2& GetLocalAnchorA() const;
    const Vec2& GetLocalAnchorB() const;
    float GetAngleOffset() const;

private:
    Vec2 localAnchorA;
    Vec2 localAnchorB;
    Vec2 localYAxis;
    float angleOffset;

    Vec2 t; // perpendicular vector
    float sa;
    float sb;
    Mat2 m;

    Vec2 bias;
    Vec2 impulseSum;

    void ApplyImpulse(const Vec2& lambda);
};

inline const Vec2& PrismaticJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const Vec2& PrismaticJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

inline float PrismaticJoint::GetAngleOffset() const
{
    return angleOffset;
}

} // namespace muli