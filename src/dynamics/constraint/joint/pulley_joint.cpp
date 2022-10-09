#include "muli/pulley_joint.h"
#include "muli/world.h"

namespace muli
{

PulleyJoint::PulleyJoint(RigidBody* _bodyA,
                         RigidBody* _bodyB,
                         const Vec2& _anchorA,
                         const Vec2& _anchorB,
                         const Vec2& _groundAnchorA,
                         const Vec2& _groundAnchorB,
                         const WorldSettings& _settings,
                         float _ratio,
                         float _frequency,
                         float _dampingRatio,
                         float _jointMass)
    : Joint(Joint::Type::JointPulley, _bodyA, _bodyB, _settings, _frequency, _dampingRatio, _jointMass)
{
    localAnchorA = MulT(bodyA->GetTransform(), _anchorA);
    localAnchorB = MulT(bodyB->GetTransform(), _anchorB);
    groundAnchorA = _groundAnchorA;
    groundAnchorB = _groundAnchorB;

    ratio = _ratio;
    length = Dist(_anchorA, _groundAnchorA) + Dist(_anchorB, _groundAnchorB);
}

void PulleyJoint::Prepare()
{
    // Calculate Jacobian J and effective mass M
    // J = -[ua, ra×ua, r*ub, r*rb×ub]
    // K = (J · M^-1 · J^t)
    //   = iMa + iIa * (ra×ua)^2 + ratio*(iMb + iIb * (rb×ub)^2)
    // M = K^-1

    ra = bodyA->GetRotation() * localAnchorA;
    rb = bodyB->GetRotation() * localAnchorB;

    ua = (bodyA->GetPosition() + ra) - groundAnchorA;
    ub = (bodyB->GetPosition() + rb) - groundAnchorB;

    float lengthA = ua.Length();
    float lengthB = ub.Length();

    if (lengthA > settings.PENETRATION_SLOP)
    {
        ua *= 1.0f / lengthA;
    }
    else
    {
        ua.SetZero();
    }

    if (lengthB > settings.PENETRATION_SLOP)
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

    bias = length - (lengthA + lengthB);
    bias *= beta * settings.INV_DT;

    if (settings.WARM_STARTING)
    {
        ApplyImpulse(impulseSum);
    }
}

void PulleyJoint::SolveVelocityConstraint()
{
    // Calculate corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    float jv = -(ratio * (Dot(ub, bodyB->linearVelocity + Cross(bodyB->angularVelocity, rb))) +
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