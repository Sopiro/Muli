#include "muli/angle_joint.h"
#include "muli/world.h"

namespace muli
{

AngleJoint::AngleJoint(RigidBody* bodyA, RigidBody* bodyB, float jointFrequency, float jointDampingRatio, float jointMass)
    : Joint(angle_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
    , impulseSum{ 0.0f }
{
    angleOffset = bodyB->sweep.a - bodyA->sweep.a;
}

void AngleJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass M
    // J = [0 -1 0 1]
    // M = (J · M^-1 · J^t)^-1

    float k = bodyA->invInertia + bodyB->invInertia + gamma;

    if (k != 0.0f)
    {
        m = 1.0f / k;
    }

    float error = bodyB->sweep.a - bodyA->sweep.a - angleOffset;
    bias = error * beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void AngleJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Compute corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    float jv = bodyB->angularVelocity - bodyA->angularVelocity;

    float lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void AngleJoint::ApplyImpulse(float lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    bodyA->angularVelocity -= lambda * bodyA->invInertia;
    bodyB->angularVelocity += lambda * bodyB->invInertia;
}

} // namespace muli