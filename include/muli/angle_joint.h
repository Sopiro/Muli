#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

struct WorldSettings;

class AngleJoint : public Joint
{
public:
    AngleJoint(RigidBody* _bodyA,
               RigidBody* _bodyB,
               const WorldSettings& _settings,
               float _frequency = 10.0f,
               float _dampingRatio = 1.0f,
               float _jointMass = -1.0f);

    virtual void Prepare() override;
    virtual void SolveVelocityConstraint() override;

    const Vec2& GetLocalAnchorA() const;
    const Vec2& GetLocalAnchorB() const;

    float GetAngleOffset() const;

private:
    float angleOffset;

    float m;

    float bias;
    float impulseSum = 0.0f;

    void ApplyImpulse(float lambda);
};

inline float AngleJoint::GetAngleOffset() const
{
    return angleOffset;
}

} // namespace muli