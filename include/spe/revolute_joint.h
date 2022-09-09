#pragma once

#include "common.h"
#include "joint.h"

namespace spe
{

struct Settings;

class RevoluteJoint : public Joint
{
public:
    RevoluteJoint(RigidBody* _bodyA,
                  RigidBody* _bodyB,
                  Vec2 _anchor,
                  const Settings& _settings,
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

    Vec2 ra{ 0.0f };
    Vec2 rb{ 0.0f };

    Mat2 m{ 1.0f };
    Vec2 bias{ 0.0f };
    Vec2 impulseSum{ 0.0f };

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

} // namespace spe