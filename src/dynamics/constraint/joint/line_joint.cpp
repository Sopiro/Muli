#include "muli/line_joint.h"
#include "muli/world.h"

namespace muli
{

LineJoint::LineJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchor,
    const Vec2& dir,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
    : Joint(line_joint, bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass)
    , impulseSum{ 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), anchor);
    localAnchorB = MulT(bodyB->GetTransform(), anchor);

    if (dir.Length2() < epsilon)
    {
        Vec2 d = MulT(bodyA->GetRotation(), Normalize(bodyB->GetPosition() - bodyA->GetPosition()));
        localYAxis = Cross(1.0f, d);
    }
    else
    {
        localYAxis = MulT(bodyA->GetRotation(), Cross(1.0f, Normalize(dir)));
    }
}

void LineJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass W
    // J = [-t^t, -(ra + d)×t, t^t, rb×t]
    // W = (J · M^-1 · J^t)^-1

    Vec2 ra = Mul(bodyA->GetRotation(), localAnchorA - bodyA->GetLocalCenter());
    Vec2 rb = Mul(bodyB->GetRotation(), localAnchorB - bodyB->GetLocalCenter());
    Vec2 pa = bodyA->motion.c + ra;
    Vec2 pb = bodyB->motion.c + rb;
    Vec2 d = pb - pa;

    t = Mul(bodyA->GetRotation(), localYAxis);
    sa = Cross(ra + d, t);
    sb = Cross(rb, t);

    // clang-format off
    float k = bodyA->invMass + bodyB->invMass
            + bodyA->invInertia * sa * sa
            + bodyB->invInertia * sb * sb
            + gamma;
    // clang-format on

    if (k != 0.0f)
    {
        m = 1.0f / k;
    }

    float error = Dot(d, t);
    bias = error * beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void LineJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Compute corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    float jv = Dot(t, bodyB->linearVelocity - bodyA->linearVelocity) + sb * bodyB->angularVelocity - sa * bodyA->angularVelocity;

    float lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void LineJoint::ApplyImpulse(float lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    Vec2 p = t * lambda;

    bodyA->linearVelocity -= p * bodyA->invMass;
    bodyA->angularVelocity -= lambda * sa * bodyA->invInertia;
    bodyB->linearVelocity += p * bodyB->invMass;
    bodyB->angularVelocity += lambda * sb * bodyB->invInertia;
}

} // namespace muli