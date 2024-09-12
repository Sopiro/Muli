#include "muli/prismatic_joint.h"
#include "muli/world.h"

namespace muli
{

// TODO: Implement limit constraint

PrismaticJoint::PrismaticJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec2& anchor, const Vec2& dir, float frequency, float dampingRatio, float jointMass
)
    : Joint(prismatic_joint, bodyA, bodyB, frequency, dampingRatio, jointMass)
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

    angleOffset = bodyB->GetAngle() - bodyA->GetAngle();
}

void PrismaticJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass M
    // J = [-t^t, -(ra + u)×t, t^t, rb×t] // Line
    //     [   0,          -1,   0,    1] // Angle
    // M = (J · M^-1 · J^t)^-1

    Vec2 ra = Mul(bodyA->GetRotation(), localAnchorA - bodyA->sweep.localCenter);
    Vec2 rb = Mul(bodyB->GetRotation(), localAnchorB - bodyB->sweep.localCenter);
    Vec2 pa = bodyA->sweep.c + ra;
    Vec2 pb = bodyB->sweep.c + rb;
    Vec2 d = pb - pa;

    // tangent/perpendicular vector
    t = Mul(bodyA->GetRotation(), localYAxis);
    sa = Cross(ra + d, t);
    sb = Cross(rb, t);

    Mat2 k;

    k[0][0] = bodyA->invMass + bodyB->invMass + sa * sa * bodyA->invInertia + sb * sb * bodyB->invInertia;
    k[1][0] = sa * bodyA->invInertia + sb * bodyB->invInertia;
    k[0][1] = k[1][0];
    k[1][1] = bodyA->invInertia + bodyB->invInertia;

    k[0][0] += gamma;
    k[1][1] += gamma;

    m = k.GetInverse();

    float error0 = Dot(d, t);
    float error1 = bodyB->GetAngle() - bodyA->GetAngle() - angleOffset;

    bias.Set(error0, error1);
    bias *= beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void PrismaticJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Compute corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    Vec2 jv;
    jv.x = Dot(t, bodyB->linearVelocity - bodyA->linearVelocity) + sb * bodyB->angularVelocity - sa * bodyA->angularVelocity;
    jv.y = bodyB->angularVelocity - bodyA->angularVelocity;

    Vec2 lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void PrismaticJoint::ApplyImpulse(const Vec2& lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    Vec2 p = t * lambda.x;

    bodyA->linearVelocity -= p * bodyA->invMass;
    bodyA->angularVelocity -= (lambda.x * sa + lambda.y) * bodyA->invInertia;
    bodyB->linearVelocity += p * bodyB->invMass;
    bodyB->angularVelocity += (lambda.x * sb + lambda.y) * bodyB->invInertia;
}

} // namespace muli