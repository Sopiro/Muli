#include "spe/revolute_joint.h"
#include "spe/world.h"

namespace spe
{
RevoluteJoint::RevoluteJoint(
    RigidBody* _bodyA,
    RigidBody* _bodyB,
    glm::vec2 _anchor,
    const Settings& _settings,
    float _frequency,
    float _dampingRatio,
    float _jointMass
) :
    Joint(_bodyA, _bodyB, _settings, _frequency, _dampingRatio, _jointMass)
{
    localAnchorA = bodyA->GlobalToLocal() * _anchor;
    localAnchorB = bodyB->GlobalToLocal() * _anchor;
}

void RevoluteJoint::Prepare()
{
    // Calculate Jacobian J and effective mass M
    // J = [-I, -skew(ra), I, skew(rb)]
    // M = (J · M^-1 · J^t)^-1

    ra = glm::mul(bodyA->LocalToGlobal(), localAnchorA, 0.0f);
    rb = glm::mul(bodyB->LocalToGlobal(), localAnchorB, 0.0f);

    glm::mat2 k{ 1.0f };

    k[0][0] = bodyA->invMass + bodyB->invMass +
                bodyA->invInertia * ra.y * ra.y + bodyB->invInertia * rb.y * rb.y;

    k[1][0] = -bodyA->invInertia * ra.y * ra.x - bodyB->invInertia * rb.y * rb.x;
    k[0][1] = -bodyA->invInertia * ra.x * ra.y - bodyB->invInertia * rb.x * rb.y;

    k[1][1] = bodyA->invMass + bodyB->invMass +
                bodyA->invInertia * ra.x * ra.x + bodyB->invInertia * rb.x * rb.x;

    k[0][0] += gamma;
    k[1][1] += gamma;

    m = glm::inverse(k);

    glm::vec2 pa = bodyA->position + ra;
    glm::vec2 pb = bodyB->position + rb;

    glm::vec2 error = pb - pa;

    if (settings.POSITION_CORRECTION)
        bias = error * beta * settings.INV_DT;
    else
        glm::clear(bias);

    if (settings.WARM_STARTING)
        ApplyImpulse(impulseSum);
}

void RevoluteJoint::Solve()
{
    // Calculate corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    glm::vec2 jv = (bodyB->linearVelocity + glm::cross(bodyB->angularVelocity, rb))
                    - (bodyA->linearVelocity + glm::cross(bodyA->angularVelocity, ra));

    // You don't have to clamp the impulse. It's equality constraint!
    glm::vec2 lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);

    if (settings.WARM_STARTING)
        impulseSum += lambda;
}

void RevoluteJoint::ApplyImpulse(const glm::vec2& lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    bodyA->linearVelocity -= lambda * bodyA->invMass;
    bodyA->angularVelocity -= bodyA->invInertia * glm::cross(ra, lambda);
    bodyB->linearVelocity += lambda * bodyB->invMass;
    bodyB->angularVelocity += bodyB->invInertia * glm::cross(rb, lambda);
}

const glm::vec2& RevoluteJoint::GetLocalAnchorA() const
{
    return localAnchorA;
}

const glm::vec2& RevoluteJoint::GetLocalAnchorB() const
{
    return localAnchorB;
}

}