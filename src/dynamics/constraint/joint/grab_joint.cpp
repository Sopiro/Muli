#include "muli/grab_joint.h"
#include "muli/world.h"

namespace muli
{

GrabJoint::GrabJoint(
    RigidBody* _body, const Vec2& _anchor, const Vec2& _target, float _frequency, float _dampingRatio, float _jointMass)
    : Joint(grab_joint, _body, _body, _frequency, _dampingRatio, _jointMass)
    , impulseSum{ 0.0f }
{
    localAnchor = MulT(_body->GetTransform(), _anchor);
    target = _target;
}

void GrabJoint::Prepare()
{
    ComputeBetaAndGamma();

    // Compute Jacobian J and effective mass M
    // J = [I, skew(r)]
    // M = (J · M^-1 · J^t)^-1

    r = Mul(bodyA->GetRotation(), localAnchor - bodyA->sweep.localCenter);
    Vec2 p = bodyA->sweep.c + r;

    Mat2 k;

    k[0][0] = bodyA->invMass + bodyA->invInertia * r.y * r.y;
    k[1][0] = -bodyA->invInertia * r.y * r.x;
    k[0][1] = -bodyA->invInertia * r.x * r.y;
    k[1][1] = bodyA->invMass + bodyA->invInertia * r.x * r.x;

    k[0][0] += gamma;
    k[1][1] += gamma;

    m = k.GetInverse();

    const WorldSettings& settings = bodyA->GetWorld()->GetWorldSettings();

    Vec2 error = p - target;
    bias = error * beta * settings.inv_dt;

    if (settings.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void GrabJoint::SolveVelocityConstraints()
{
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