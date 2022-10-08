#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

struct WorldSettings;

class PrismaticJoint : public Joint
{
public:
    PrismaticJoint(RigidBody* _bodyA,
                   RigidBody* _bodyB,
                   Vec2 _anchor,
                   Vec2 _dir,
                   const WorldSettings& _settings,
                   float _frequency = -1.0f,
                   float _dampingRatio = 1.0f,
                   float _jointMass = 1.0f);

    virtual void Prepare() override;
    virtual void SolveVelocityConstraint() override;

    const Vec2& GetLocalAnchorA() const;
    const Vec2& GetLocalAnchorB() const;
    float GetAngleOffset() const;

private:
    Vec2 localAnchorA;
    Vec2 localAnchorB;
    Vec2 localYAxis;
    float angleOffset;

    float sa;
    float sb;
    Vec2 t;

    Mat2 m;

    Vec2 bias;
    Vec2 impulseSum{ 0.0f };

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