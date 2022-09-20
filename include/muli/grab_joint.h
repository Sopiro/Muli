#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

struct WorldSettings;

class GrabJoint : public Joint
{
public:
    GrabJoint(RigidBody* _body,
              Vec2 _anchor,
              Vec2 _target,
              const WorldSettings& _settings,
              float _frequency = 1.0f,
              float _dampingRatio = 0.5f,
              float _jointMass = -1.0f);

    virtual void Prepare() override;
    virtual void SolveVelocityConstraint() override;

    const Vec2& GetLocalAnchor() const;
    const Vec2& GetTarget() const;
    void SetTarget(const Vec2& target);

private:
    Vec2 localAnchor;
    Vec2 target;

    Vec2 r{ 0.0f };
    Mat2 m{ 1.0f };
    Vec2 bias{ 0.0f };
    Vec2 impulseSum{ 0.0f };

    void ApplyImpulse(const Vec2& lambda);
};

inline const Vec2& GrabJoint::GetLocalAnchor() const
{
    return localAnchor;
}

inline const Vec2& GrabJoint::GetTarget() const
{
    return target;
}

inline void GrabJoint::SetTarget(const Vec2& _target)
{
    target = _target;
}

} // namespace muli