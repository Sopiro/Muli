#include "spe/distance_joint.h"
#include "spe/world.h"

namespace spe
{
DistanceJoint::DistanceJoint(
    RigidBody* _bodyA,
    RigidBody* _bodyB,
    glm::vec2 _anchorA,
    glm::vec2 _anchorB,
    float _length,
    const Settings& _settings,
    float _frequency,
    float _dampingRatio,
    float _jointMass
) :
    Joint(_bodyA, _bodyB, _settings, _frequency, _dampingRatio, _jointMass)
{
    localAnchorA = bodyA->GlobalToLocal() * _anchorA;
    localAnchorB = bodyB->GlobalToLocal() * _anchorB;
    length = _length < 0.0f ? glm::length(_anchorB - _anchorA) : _length;

    type = JointType::JointDistance;
}

void DistanceJoint::Prepare()
{
    // Calculate Jacobian J and effective mass M
    // J = [-n, -n·cross(ra), n, n·cross(rb)] ( n = (anchorB-anchorA) / ||anchorB-anchorA|| )
    // M = (J · M^-1 · J^t)^-1

    ra = glm::mul(bodyA->LocalToGlobal(), localAnchorA, 0.0f);
    rb = glm::mul(bodyB->LocalToGlobal(), localAnchorB, 0.0f);

    glm::vec2 pa = bodyA->position + ra;
    glm::vec2 pb = bodyB->position + rb;

    glm::vec2 u = pb - pa;

    n = glm::normalize(u);

    float k = bodyA->invMass + bodyB->invMass
        + bodyA->invInertia * glm::cross(n, ra) * glm::cross(n, ra)
        + bodyB->invInertia * glm::cross(n, rb) * glm::cross(n, rb)
        + gamma;

    m = 1.0f / k;

    float error = glm::length(u) - length;

    if (settings.POSITION_CORRECTION)
        bias = error * beta * settings.INV_DT;
    else
        bias = 0.0f;

    if (settings.WARM_STARTING)
        ApplyImpulse(impulseSum);
}

void DistanceJoint::Solve()
{
    // Calculate corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    float jv = glm::dot((bodyB->linearVelocity + glm::cross(bodyB->angularVelocity, rb))
        - (bodyA->linearVelocity + glm::cross(bodyA->angularVelocity, ra))
        , n);

    // You don't have to clamp the impulse. It's equality constraint!
    float lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);

    if (settings.WARM_STARTING)
        impulseSum += lambda;
}

void DistanceJoint::ApplyImpulse(float lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    bodyA->linearVelocity -= n * (lambda * bodyA->invMass);
    bodyA->angularVelocity -= glm::dot(n, glm::cross(lambda, ra)) * bodyA->invInertia;
    bodyB->linearVelocity += n * (lambda * bodyB->invMass);
    bodyB->angularVelocity += glm::dot(n, glm::cross(lambda, rb)) * bodyB->invInertia;
}

}