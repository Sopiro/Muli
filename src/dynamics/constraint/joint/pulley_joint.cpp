#include "muli/pulley_joint.h"
#include "muli/world.h"

namespace muli
{

PulleyJoint::PulleyJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchorA,
    const Vec2& anchorB,
    const Vec2& inGroundAnchorA,
    const Vec2& inGroundAnchorB,
    float pulleyRatio,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
    : Joint(pulley_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
    , impulseSum{ 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), anchorA);
    localAnchorB = MulT(bodyB->GetTransform(), anchorB);
    groundAnchorA = inGroundAnchorA;
    groundAnchorB = inGroundAnchorB;

    ratio = pulleyRatio;
    length = Dist(anchorA, groundAnchorA) + Dist(anchorB, groundAnchorB);
}

void PulleyJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass W
    // J = -[ua, ra×ua, r*ub, r*rb×ub]
    // K = (J · M^-1 · J^t)
    //   = iMa + iIa * (ra×ua)^2 + ratio*(iMb + iIb * (rb×ub)^2)
    // W = K^-1

    ra = Mul(bodyA->GetRotation(), localAnchorA - bodyA->GetLocalCenter());
    rb = Mul(bodyB->GetRotation(), localAnchorB - bodyB->GetLocalCenter());

    ua = (bodyA->motion.c + ra) - groundAnchorA;
    ub = (bodyB->motion.c + rb) - groundAnchorB;

    float lengthA = ua.Length();
    float lengthB = ub.Length();

    if (lengthA > linear_slop)
    {
        ua *= 1.0f / lengthA;
    }
    else
    {
        ua.SetZero();
    }

    if (lengthB > linear_slop)
    {
        ub *= 1.0f / lengthB;
    }
    else
    {
        ub.SetZero();
    }

    float rua = Cross(ra, ua);
    float rub = Cross(rb, ub);

    // clang-format off
    float k = bodyA->invMass + bodyA->invInertia * rua * rua
            + (bodyB->invMass + bodyB->invInertia * rub * rub) * ratio * ratio
            + gamma;
    // clang-format on

    if (k != 0.0f)
    {
        m = 1.0f / k;
    }

    float error = length - (lengthA + lengthB);
    bias = error * beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void PulleyJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Compute corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    float jv =
        -(ratio * (Dot(ub, bodyB->linearVelocity + Cross(bodyB->angularVelocity, rb))) +
          Dot(ua, bodyA->linearVelocity + Cross(bodyA->angularVelocity, ra)));

    float lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void PulleyJoint::ApplyImpulse(float lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    Vec2 pa = -lambda * ua;
    Vec2 pb = -ratio * lambda * ub;

    bodyA->linearVelocity += pa * bodyA->invMass;
    bodyA->angularVelocity += Cross(ra, pa) * bodyA->invInertia;
    bodyB->linearVelocity += pb * bodyB->invMass;
    bodyB->angularVelocity += Cross(rb, pb) * bodyB->invInertia;
}

} // namespace muli