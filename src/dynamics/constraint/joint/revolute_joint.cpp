#include "muli/revolute_joint.h"
#include "muli/world.h"

namespace muli
{

RevoluteJoint::RevoluteJoint(
    RigidBody* _bodyA, RigidBody* _bodyB, const Vec2& _anchor, float _frequency, float _dampingRatio, float _jointMass
)
    : Joint(revolute_joint, _bodyA, _bodyB, _frequency, _dampingRatio, _jointMass)
    , impulseSum{ 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), _anchor);
    localAnchorB = MulT(bodyB->GetTransform(), _anchor);
}

void RevoluteJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass M
    // J = [-I, -skew(ra), I, skew(rb)]
    // M = (J · M^-1 · J^t)^-1

    ra = Mul(bodyA->GetRotation(), localAnchorA - bodyA->sweep.localCenter);
    rb = Mul(bodyB->GetRotation(), localAnchorB - bodyB->sweep.localCenter);

    Mat2 k;

    k[0][0] = bodyA->invMass + bodyB->invMass + bodyA->invInertia * ra.y * ra.y + bodyB->invInertia * rb.y * rb.y;

    k[1][0] = -bodyA->invInertia * ra.y * ra.x - bodyB->invInertia * rb.y * rb.x;
    k[0][1] = -bodyA->invInertia * ra.x * ra.y - bodyB->invInertia * rb.x * rb.y;

    k[1][1] = bodyA->invMass + bodyB->invMass + bodyA->invInertia * ra.x * ra.x + bodyB->invInertia * rb.x * rb.x;

    k[0][0] += gamma;
    k[1][1] += gamma;

    m = k.GetInverse();

    Vec2 pa = bodyA->sweep.c + ra;
    Vec2 pb = bodyB->sweep.c + rb;

    Vec2 error = pb - pa;
    bias = error * beta * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void RevoluteJoint::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Compute corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    Vec2 jv =
        (bodyB->linearVelocity + Cross(bodyB->angularVelocity, rb)) - (bodyA->linearVelocity + Cross(bodyA->angularVelocity, ra));

    // You don't have to clamp the impulse. It's equality constraint!
    Vec2 lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
}

void RevoluteJoint::ApplyImpulse(const Vec2& lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    bodyA->linearVelocity -= lambda * bodyA->invMass;
    bodyA->angularVelocity -= bodyA->invInertia * Cross(ra, lambda);
    bodyB->linearVelocity += lambda * bodyB->invMass;
    bodyB->angularVelocity += bodyB->invInertia * Cross(rb, lambda);
}

} // namespace muli