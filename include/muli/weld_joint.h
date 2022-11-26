#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

struct WorldSettings;

class WeldJoint : public Joint
{
public:
    WeldJoint(RigidBody* bodyA,
              RigidBody* bodyB,
              const Vec2& anchor,
              const WorldSettings& settings,
              float frequency = -1.0f,
              float dampingRatio = 1.0f,
              float jointMass = 1.0f);

    virtual void Prepare() override;
    virtual void SolveVelocityConstraint() override;

    const Vec2& GetLocalAnchorA() const;
    const Vec2& GetLocalAnchorB() const;

private:
    Vec2 localAnchorA;
    Vec2 localAnchorB;

    float angleOffset;

    Vec2 ra;
    Vec2 rb;
    Mat3 m;

    Vec3 bias;
    Vec3 impulseSum{ 0.0f };

    void ApplyImpulse(const Vec3& lambda);
};

inline const Vec2& WeldJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

inline const Vec2& WeldJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

} // namespace muli