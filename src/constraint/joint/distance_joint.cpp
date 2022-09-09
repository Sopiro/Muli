#include "spe/distance_joint.h"
#include "spe/world.h"

namespace spe
{

DistanceJoint::DistanceJoint(RigidBody* _bodyA,
                             RigidBody* _bodyB,
                             Vec2 _anchorA,
                             Vec2 _anchorB,
                             float _length,
                             const Settings& _settings,
                             float _frequency,
                             float _dampingRatio,
                             float _jointMass)
    : Joint(Joint::Type::JointDistance, _bodyA, _bodyB, _settings, _frequency, _dampingRatio, _jointMass)
{
    localAnchorA = mul_t(bodyA->GetTransform(), _anchorA);
    localAnchorB = mul_t(bodyB->GetTransform(), _anchorB);
    length = _length < 0.0f ? spe::length(_anchorB - _anchorA) : _length;
}

void DistanceJoint::Prepare()
{
    // Calculate Jacobian J and effective mass M
    // J = [-n, -n·cross(ra), n, n·cross(rb)] ( n = (anchorB-anchorA) / ||anchorB-anchorA|| )
    // M = (J · M^-1 · J^t)^-1

    ra = bodyA->GetRotation() * localAnchorA;
    rb = bodyB->GetRotation() * localAnchorB;

    Vec2 pa = bodyA->GetPosition() + ra;
    Vec2 pb = bodyB->GetPosition() + rb;

    u = pb - pa;
    float currentLength = u.Normalize();

    // clang-format off
    float k
        = bodyA->invMass + bodyB->invMass
        + bodyA->invInertia * cross(u, ra) * cross(u, ra)
        + bodyB->invInertia * cross(u, rb) * cross(u, rb)
        + gamma;
    // clang-format on

    m = 1.0f / k;

    float error = currentLength - length;
    bias = error * beta * settings.INV_DT;

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

    float jv = dot((bodyB->linearVelocity + cross(bodyB->angularVelocity, rb)) -
                       (bodyA->linearVelocity + cross(bodyA->angularVelocity, ra)),
                   u);

    // You don't have to clamp the impulse. It's equality constraint!
    float lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);

    if (settings.WARM_STARTING)
    {
        impulseSum += lambda;
    }
}

void DistanceJoint::ApplyImpulse(float lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    bodyA->linearVelocity -= u * (lambda * bodyA->invMass);
    bodyA->angularVelocity -= dot(u, cross(lambda, ra)) * bodyA->invInertia;
    bodyB->linearVelocity += u * (lambda * bodyB->invMass);
    bodyB->angularVelocity += dot(u, cross(lambda, rb)) * bodyB->invInertia;
}

} // namespace spe