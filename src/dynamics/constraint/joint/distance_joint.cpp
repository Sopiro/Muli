#include "muli/distance_joint.h"
#include "muli/world.h"

namespace muli
{

DistanceJoint::DistanceJoint(
    RigidBody* _bodyA,
    RigidBody* _bodyB,
    const Vec2& _anchorA,
    const Vec2& _anchorB,
    float _length,
    float _frequency,
    float _dampingRatio,
    float _jointMass
)
    : Joint(distance_joint, _bodyA, _bodyB, _frequency, _dampingRatio, _jointMass)
    , impulseSum{ 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), _anchorA);
    localAnchorB = MulT(bodyB->GetTransform(), _anchorB);
    length = _length < 0.0f ? Length(_anchorB - _anchorA) : _length;
}

void DistanceJoint::Prepare(const Timestep& step)
{
    ComputeBetaAndGamma(step);

    // Compute Jacobian J and effective mass M
    // J = [-d, -d×ra, d, d×rb] ( d = (anchorB-anchorA) / ||anchorB-anchorA|| )
    // M = (J · M^-1 · J^t)^-1

    ra = Mul(bodyA->GetRotation(), localAnchorA - bodyA->sweep.localCenter);
    rb = Mul(bodyB->GetRotation(), localAnchorB - bodyB->sweep.localCenter);

    Vec2 pa = bodyA->sweep.c + ra;
    Vec2 pb = bodyB->sweep.c + rb;

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

    float error = currentLength - length;
    bias = error * step.inv_dt;

    if (step.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void DistanceJoint::SolveVelocityConstraints(const Timestep& step)
{
    muliNotUsed(step);

    // Compute corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    float jv =
        Dot((bodyB->linearVelocity + Cross(bodyB->angularVelocity, rb)) -
                (bodyA->linearVelocity + Cross(bodyA->angularVelocity, ra)),
            d);

    // You don't have to clamp the impulse. It's equality constraint!
    float lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);
    impulseSum += lambda;
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