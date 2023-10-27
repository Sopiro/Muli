#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

class GrabJoint : public Joint
{
public:
    GrabJoint(RigidBody* body,
              const Vec2& anchor,
              const Vec2& target,
              float frequency = 1.0f,
              float dampingRatio = 0.5f,
              float jointMass = -1.0f);

    virtual void Prepare(const Timestep& step) override;
    virtual void SolveVelocityConstraints(const Timestep& step) override;

    const Vec2& GetLocalAnchor() const;

    const Vec2& GetTarget() const;
    void SetTarget(const Vec2& newTarget);

private:
    Vec2 localAnchor;
    Vec2 target;

    Vec2 r;
    Mat2 m;

    Vec2 bias;
    Vec2 impulseSum;

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

inline void GrabJoint::SetTarget(const Vec2& newTarget)
{
    target = newTarget;
}

} // namespace muli