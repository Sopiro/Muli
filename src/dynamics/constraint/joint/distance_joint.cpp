#include "muli/distance_joint.h"
#include "muli/world.h"

namespace muli
{

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
    , impulseSum{ 0.0f }
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

    Vec2 error(currentLength - minLength, currentLength - maxLength);
    bias = error * beta * step.inv_dt;

    if (step.warm_starting)
    {
        if (minLength == maxLength)
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

    if (minLength == maxLength)
    {
        // You don't have to clamp the impulse because it's equality constraint!
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