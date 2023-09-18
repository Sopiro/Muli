#include "muli/line_joint.h"
#include "muli/world.h"

namespace muli
{

LineJoint::LineJoint(RigidBody* _bodyA,
                     RigidBody* _bodyB,
                     Vec2 _anchor,
                     Vec2 _dir,
                     const WorldSettings& _settings,
                     float _frequency,
                     float _dampingRatio,
                     float _jointMass)
    : Joint(line_joint, _bodyA, _bodyB, _settings, _frequency, _dampingRatio, _jointMass)
    , impulseSum{ 0.0f }
{
    localAnchorA = MulT(bodyA->GetTransform(), _anchor);
    localAnchorB = MulT(bodyB->GetTransform(), _anchor);

    if (_dir.Length2() < epsilon)
    {
        Vec2 d = MulT(bodyA->GetRotation(), Normalize(_bodyB->GetPosition() - _bodyA->GetPosition()));
        localYAxis = Cross(1.0f, d);
    }
    else
    {
        localYAxis = MulT(bodyA->GetRotation(), Cross(1.0f, Normalize(_dir)));
    }
}

void LineJoint::Prepare()
{
    ComputeBetaAndGamma();

    // Compute Jacobian J and effective mass M
    // J = [-t^t, -(ra + d)×t, t^t, rb×t]
    // M = (J · M^-1 · J^t)^-1

    Vec2 ra = Mul(bodyA->GetRotation(), localAnchorA - bodyA->sweep.localCenter);
    Vec2 rb = Mul(bodyB->GetRotation(), localAnchorB - bodyB->sweep.localCenter);
    Vec2 pa = bodyA->sweep.c + ra;
    Vec2 pb = bodyB->sweep.c + rb;
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

    bias = error * beta * settings.inv_dt;

    if (settings.warm_starting)
    {
        ApplyImpulse(impulseSum);
    }
}

void LineJoint::SolveVelocityConstraints()
{
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