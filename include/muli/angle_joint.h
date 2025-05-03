#pragma once

#include "common.h"
#include "joint.h"

namespace muli
{

class AngleJoint : public Joint
{
public:
    AngleJoint(RigidBody* bodyA, RigidBody* bodyB, float frequency, float dampingRatio, float jointMass);

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