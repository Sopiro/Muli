#include "muli/angle_joint.h"

namespace muli
{

enum
{
    angle_limit_inactive,
    angle_limit_at_lower,
    angle_limit_at_upper,
    angle_limit_equal,
};

static float ClampImpulse(float impulse, int32 limitState)
{
    switch (limitState)
    {
    case angle_limit_at_lower:
        return Max(impulse, 0.0f);
    case angle_limit_at_upper:
        return Min(impulse, 0.0f);
    case angle_limit_inactive:
        return 0.0f;
    default:
        return impulse;
    }
}

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
    , bias{ 0.0f }
    , impulseSum{ 0.0f }
    , limitState{ angle_limit_inactive }
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

    float currentAngle = bodyB->motion.a - bodyA->motion.a - angleOffset;

    if (minAngle == maxAngle)
    {
        limitState = angle_limit_equal;
        bias = (currentAngle - minAngle) * beta * step.inv_dt;
    }
    else if (currentAngle < minAngle - angular_slop)
    {
        limitState = angle_limit_at_lower;
        bias = (currentAngle - (minAngle - angular_slop)) * beta * step.inv_dt;
    }
    else if (currentAngle > maxAngle + angular_slop)
    {
        limitState = angle_limit_at_upper;
        bias = (currentAngle - (maxAngle + angular_slop)) * beta * step.inv_dt;
    }
    else
    {
        limitState = angle_limit_inactive;
        bias = 0.0f;
    }

    impulseSum = ClampImpulse(impulseSum, limitState);

    if (step.warm_starting && limitState != angle_limit_inactive)
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

    if (limitState == angle_limit_inactive)
    {
        return;
    }

    float lambda = m * -(jv + bias + impulseSum * gamma);

    float newImpulseSum;
    if (limitState == angle_limit_equal)
    {
        newImpulseSum = impulseSum + lambda;
    }
    else
    {
        newImpulseSum = ClampImpulse(impulseSum + lambda, limitState);
    }

    lambda = newImpulseSum - impulseSum;
    impulseSum = newImpulseSum;

    ApplyImpulse(lambda);
}

void AngleJoint::ApplyImpulse(float lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    bodyA->angularVelocity -= lambda * bodyA->invInertia;
    bodyB->angularVelocity += lambda * bodyB->invInertia;
}

} // namespace muli
