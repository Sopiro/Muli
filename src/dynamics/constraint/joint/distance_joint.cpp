#include "muli/distance_joint.h"

namespace muli
{

enum
{
    distance_limit_inactive,
    distance_limit_at_lower,
    distance_limit_at_upper,
    distance_limit_equal,
};

static float ClampImpulse(float impulse, int32 limitState)
{
    switch (limitState)
    {
    case distance_limit_at_lower:
        return Max(impulse, 0.0f);
    case distance_limit_at_upper:
        return Min(impulse, 0.0f);
    case distance_limit_inactive:
        return 0.0f;
    default:
        return impulse;
    }
}

DistanceJoint::DistanceJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchorA,
    const Vec2& anchorB,
    float jointMinLength,
    float jointMaxLength,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
    : Joint(distance_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
    , bias{ 0.0f }
    , impulseSum{ 0.0f }
    , limitState{ distance_limit_inactive }
{
    localAnchorA = MulT(bodyA->GetTransform(), anchorA);
    localAnchorB = MulT(bodyB->GetTransform(), anchorB);
    minLength = jointMinLength < 0 ? Length(anchorB - anchorA) : jointMinLength;
    maxLength = jointMaxLength < 0 ? Length(anchorB - anchorA) : jointMaxLength;
    maxLength = Max(minLength, maxLength);
}

void DistanceJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass W
    // J = [-d, -d×ra, d, d×rb] ( d = (anchorB-anchorA) / ||anchorB-anchorA|| )
    // W = (J · M^-1 · J^t)^-1

    ra = Mul(bodyA->GetRotation(), localAnchorA - bodyA->GetLocalCenter());
    rb = Mul(bodyB->GetRotation(), localAnchorB - bodyB->GetLocalCenter());

    Vec2 pa = bodyA->motion.c + ra;
    Vec2 pb = bodyB->motion.c + rb;

    d = pb - pa;
    float currentLength = d.Normalize();

    // clang-format off
    float k = bodyA->invMass + bodyB->invMass
            + bodyA->invInertia * Cross(d, ra) * Cross(d, ra)
            + bodyB->invInertia * Cross(d, rb) * Cross(d, rb)
            + gamma;
    // clang-format on

    if (k != 0.0f)
    {
        m = 1.0f / k;
    }

    if (minLength == maxLength)
    {
        limitState = distance_limit_equal;
        bias = (currentLength - minLength) * beta * step.inv_dt;
    }
    else if (currentLength < minLength)
    {
        limitState = distance_limit_at_lower;
        bias = (currentLength - minLength) * beta * step.inv_dt;
    }
    else if (currentLength > maxLength)
    {
        limitState = distance_limit_at_upper;
        bias = (currentLength - maxLength) * beta * step.inv_dt;
    }
    else
    {
        limitState = distance_limit_inactive;
        bias = 0.0f;
    }

    impulseSum = ClampImpulse(impulseSum, limitState);

    if (step.warm_starting && limitState != distance_limit_inactive)
    {
        ApplyImpulse(impulseSum);
    }
}

void DistanceJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Compute corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    float jv =
        Dot((bodyB->linearVelocity + Cross(bodyB->angularVelocity, rb)) -
                (bodyA->linearVelocity + Cross(bodyA->angularVelocity, ra)),
            d);

    if (limitState == distance_limit_inactive)
    {
        return;
    }

    float lambda = m * -(jv + bias + impulseSum * gamma);

    float newImpulseSum;
    if (limitState == distance_limit_equal)
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

void DistanceJoint::ApplyImpulse(float lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    Vec2 p = d * lambda;

    bodyA->linearVelocity -= p * bodyA->invMass;
    bodyA->angularVelocity -= Dot(d, Cross(lambda, ra)) * bodyA->invInertia;
    bodyB->linearVelocity += p * bodyB->invMass;
    bodyB->angularVelocity += Dot(d, Cross(lambda, rb)) * bodyB->invInertia;
}

} // namespace muli
