#include "muli/distance_joint.h"
#include "muli/world.h"

namespace muli
{

DistanceJoint::DistanceJoint(RigidBody* _bodyA,
                             RigidBody* _bodyB,
                             const Vec2& _anchorA,
                             const Vec2& _anchorB,
                             float _length,
                             const WorldSettings& _settings,
                             float _frequency,
                             float _dampingRatio,
                             float _jointMass)
    : Joint(Joint::Type::JointDistance, _bodyA, _bodyB, _settings, _frequency, _dampingRatio, _jointMass)
{
    localAnchorA = MulT(bodyA->GetTransform(), _anchorA);
    localAnchorB = MulT(bodyB->GetTransform(), _anchorB);
    length = _length < 0.0f ? Length(_anchorB - _anchorA) : _length;
}

void DistanceJoint::Prepare()
{
    // Calculate Jacobian J and effective mass M
    // J = [-d, -d×ra, d, d×rb] ( d = (anchorB-anchorA) / ||anchorB-anchorA|| )
    // M = (J · M^-1 · J^t)^-1

    ra = bodyA->GetRotation() * localAnchorA;
    rb = bodyB->GetRotation() * localAnchorB;

    Vec2 pa = bodyA->GetPosition() + ra;
    Vec2 pb = bodyB->GetPosition() + rb;

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

    bias = currentLength - length;
    bias *= beta * settings.INV_DT;

    if (settings.WARM_STARTING)
    {
        ApplyImpulse(impulseSum);
    }
}

void DistanceJoint::SolveVelocityConstraint()
{
    // Calculate corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    float jv = Dot((bodyB->linearVelocity + Cross(bodyB->angularVelocity, rb)) -
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