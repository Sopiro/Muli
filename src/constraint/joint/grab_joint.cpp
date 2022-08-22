#include "spe/grab_joint.h"
#include "spe/world.h"

namespace spe
{
GrabJoint::GrabJoint(
    RigidBody* _body,
    glm::vec2 _anchor,
    glm::vec2 _target,
    const Settings& _settings,
    float _frequency,
    float _dampingRatio,
    float _jointMass
) :
    Joint(_body, _body, _settings, _frequency, _dampingRatio, _jointMass)
{
    localAnchor = _body->GlobalToLocal() * _anchor;
    target = _target;

    type = JointType::JointGrab;
}

void GrabJoint::Prepare()
{
    // Calculate Jacobian J and effective mass M
    // J = [I, skew(r)]
    // M = (J · M^-1 · J^t)^-1

    r = glm::mul(bodyA->LocalToGlobal(), localAnchor, 0.0f);
    glm::vec2 p = bodyA->position + r;

    glm::mat2 k{ 1.0f };

    k[0][0] = bodyA->invMass + bodyA->invInertia * r.y * r.y;
    k[1][0] = -bodyA->invInertia * r.y * r.x;
    k[0][1] = -bodyA->invInertia * r.x * r.y;
    k[1][1] = bodyA->invMass + bodyA->invInertia * r.x * r.x;

    k[0][0] += gamma;
    k[1][1] += gamma;

    m = glm::inverse(k);

    glm::vec2 error = p - target;

    if (settings.POSITION_CORRECTION)
        bias = error * beta * settings.INV_DT;
    else
        glm::clear(bias);

    if (settings.WARM_STARTING)
        ApplyImpulse(impulseSum);
}

void GrabJoint::Solve()
{
    // Calculate corrective impulse: Pc
    // Pc = J^t · λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    glm::vec2 jv = bodyA->linearVelocity + glm::cross(bodyA->angularVelocity, r);
    glm::vec2 lambda = m * -(jv + bias + impulseSum * gamma);

    ApplyImpulse(lambda);

    if (settings.WARM_STARTING)
        impulseSum += lambda;
}

void GrabJoint::ApplyImpulse(const glm::vec2& lambda)
{
    bodyA->linearVelocity += lambda * bodyA->invMass;
    bodyA->angularVelocity += bodyA->invInertia * glm::cross(r, lambda);
}

}