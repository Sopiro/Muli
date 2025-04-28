#include "muli/grab_joint.h"
#include "muli/world.h"

namespace muli
{

GrabJoint::GrabJoint(
    RigidBody* body,
    const Vec2& anchor,
    const Vec2& targetPosition,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
    : Joint(grab_joint, body, body, jointFrequency, jointDampingRatio, jointMass)
    , impulseSum{ 0.0f }
{
    localAnchor = MulT(body->GetTransform(), anchor);
    target = targetPosition;
}

void GrabJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass W
    // J = [I, skew(r)]
    // W = (J · M^-1 · J^t)^-1

    r = Mul(bodyA->GetRotation(), localAnchor - bodyA->GetLocalCenter());
    Vec2 p = bodyA->motion.c + r;

    Mat2 k;

    k[0][0] = bodyA->invMass + bodyA->invInertia * r.y * r.y;
    k[1][0] = -bodyA->invInertia * r.y * r.x;
    k[0][1] = -bodyA->invInertia * r.x * r.y;
    k[1][1] = bodyA->invMass + bodyA->invInertia * r.x * r.x;

    k[0][0] += gamma;
    k[1][1] += gamma;

    m = k.GetInverse();

    Vec2 error = p - target;
    bias = error * beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void GrabJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Compute corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    Vec2 jv = bodyA->linearVelocity + Cross(bodyA->angularVelocity, r);

    Vec2 lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void GrabJoint::ApplyImpulse(const Vec2& lambda)
{
    bodyA->linearVelocity += lambda * bodyA->invMass;
    bodyA->angularVelocity += bodyA->invInertia * Cross(r, lambda);
}

} // namespace muli