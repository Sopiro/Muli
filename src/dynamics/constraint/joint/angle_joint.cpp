#include "muli/angle_joint.h"
#include "muli/world.h"

namespace muli
{

AngleJoint::AngleJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    float jointAngleOffset,
    float jointMinAngle,
    float jointMaxAngle,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
    : Joint(angle_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
    , angleOffset{ jointAngleOffset }
    , minAngle{ jointMinAngle }
    , maxAngle{ jointMaxAngle }
    , impulseSum{ 0.0f }
{
    maxAngle = Max(minAngle, maxAngle);
}

void AngleJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass W
    // J = [0 -1 0 1]
    // W = (J · M^-1 · J^t)^-1

    float k = bodyA->invInertia + bodyB->invInertia + gamma;

    if (k != 0.0f)
    {
        m = 1.0f / k;
    }

    Vec2 error(bodyB->motion.a - bodyA->motion.a - angleOffset);
    error[0] -= minAngle;
    error[1] -= maxAngle;

    bias = error * beta * step.inv_dt;

    if (step.warm_starting)
    {
        if (minAngle == maxAngle)
        {
            ApplyImpulse(impulseSum[0]);
        }
        else
        {
            if (bias[0] < 0)
            {
                ApplyImpulse(impulseSum[0]);
            }
            if (bias[1] > 0)
            {
                ApplyImpulse(impulseSum[1]);
            }
        }
    }
}

void AngleJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Compute corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    float jv = bodyB->angularVelocity - bodyA->angularVelocity;

    if (minAngle == maxAngle)
    {
        float lambda = m * -(jv + bias[0] + impulseSum[0] * gamma);
        ApplyImpulse(lambda);
        impulseSum[0] += lambda;
    }
    else
    {
        if (bias[0] < 0)
        {
            float lambda = m * -(jv + bias[0] + impulseSum[0] * gamma);
            ApplyImpulse(lambda);
            impulseSum[0] += lambda;
        }
        if (bias[1] > 0)
        {
            float lambda = m * -(jv + bias[1] + impulseSum[1] * gamma);
            ApplyImpulse(lambda);
            impulseSum[1] += lambda;
        }
    }
}

void AngleJoint::ApplyImpulse(float lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    bodyA->angularVelocity -= lambda * bodyA->invInertia;
    bodyB->angularVelocity += lambda * bodyB->invInertia;
}

} // namespace muli