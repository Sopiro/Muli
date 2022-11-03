#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

struct WorldSettings;

class LineJoint : public Joint
{
public:
    LineJoint(RigidBody* _bodyA,
              RigidBody* _bodyB,
              Vec2 _anchor,
              Vec2 _dir,
              const WorldSettings& _settings,
              float _frequency = 10.0f,
              float _dampingRatio = 1.0f,
              float _jointMass = -1.0f);

    virtual void Prepare() override;
    virtual void SolveVelocityConstraint() override;

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
    float impulseSum = 0.0f;

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