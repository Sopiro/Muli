#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

class AngleJoint : public Joint
{
public:
    AngleJoint(RigidBody* bodyA, RigidBody* bodyB, float frequency = 10.0f, float dampingRatio = 1.0f, float jointMass = -1.0f);

    virtual void Prepare(const Timestep& step) override;
    virtual void SolveVelocityConstraints(const Timestep& step) override;

    float GetAngleOffset() const;

private:
    float angleOffset;

    float m;

    float bias;
    float impulseSum;

    void ApplyImpulse(float lambda);
};

inline float AngleJoint::GetAngleOffset() const
{
    return angleOffset;
}

} // namespace muli