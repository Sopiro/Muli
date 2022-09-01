#include "spe/contact_solver.h"
#include "spe/contact.h"
#include "spe/world.h"

namespace spe
{

void ContactSolver::Prepare(Contact* _contact, const glm::vec2& _contactPoint, const glm::vec2& _dir, Type _contactType)
{
    // Calculate Jacobian J and effective mass M
    // J = [-dir, -ra × dir, dir, rb × dir] (dir: Contact vector, normal or tangent)
    // M = (J · M^-1 · J^t)^-1

    contact = _contact;
    contactPoint = _contactPoint;
    contactType = _contactType;

    beta = contact->settings.POSITION_CORRECTION_BETA;
    restitution = contact->manifold.bodyA->restitution * contact->manifold.bodyB->restitution;
    friction = contact->manifold.bodyA->friction * contact->manifold.bodyB->friction;

    ra = contactPoint - contact->manifold.bodyA->position;
    rb = contactPoint - contact->manifold.bodyB->position;

    jacobian.va = -_dir;
    jacobian.wa = glm::cross(-ra, _dir);
    jacobian.vb = _dir;
    jacobian.wb = glm::cross(rb, _dir);

    bias = 0.0f;
    if (contactType == Normal)
    {
        // Relative velocity at contact point
        glm::vec2 relativeVelocity =
            (contact->manifold.bodyB->linearVelocity + glm::cross(contact->manifold.bodyB->angularVelocity, rb)) -
            (contact->manifold.bodyA->linearVelocity + glm::cross(contact->manifold.bodyA->angularVelocity, ra));

        float normalVelocity = glm::dot(contact->manifold.contactNormal, relativeVelocity);

        if (contact->settings.POSITION_CORRECTION)
            bias = -(beta * contact->settings.INV_DT) *
                   glm::max(contact->manifold.penetrationDepth - contact->settings.PENETRATION_SLOP, 0.0f);

        bias += restitution * glm::min(normalVelocity + contact->settings.RESTITUTION_SLOP, 0.0f);
    }
    else
    {
        bias = -(contact->manifold.bodyB->surfaceSpeed - contact->manifold.bodyA->surfaceSpeed);

        if (contact->manifold.featureFlipped) bias *= -1;
    }

    // clang-format off
    float k
        = contact->manifold.bodyA->invMass
        + jacobian.wa * contact->manifold.bodyA->invInertia * jacobian.wa
        + contact->manifold.bodyB->invMass
        + jacobian.wb * contact->manifold.bodyB->invInertia * jacobian.wb;
    // clang-format on

    effectiveMass = k > 0.0f ? 1.0f / k : 0.0f;

    if (contact->settings.WARM_STARTING) ApplyImpulse(impulseSum);
}

void ContactSolver::Solve(const ContactSolver* normalContact)
{
    // Calculate corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    // clang-format off
    // Jacobian * velocity vector (Normal velocity)
    float jv = glm::dot(jacobian.va, contact->manifold.bodyA->linearVelocity)
        + jacobian.wa * contact->manifold.bodyA->angularVelocity
        + glm::dot(jacobian.vb, contact->manifold.bodyB->linearVelocity)
        + jacobian.wb * contact->manifold.bodyB->angularVelocity;
    // clang-format on

    float lambda = effectiveMass * -(jv + bias);

    float oldImpulseSum = impulseSum;
    switch (contactType)
    {
    case Normal:
        if (contact->settings.IMPULSE_ACCUMULATION)
            impulseSum = glm::max(0.0f, impulseSum + lambda);
        else
            impulseSum = glm::max(0.0f, lambda);
        break;
    case Tangent:
        float maxFriction = friction * normalContact->impulseSum;
        if (contact->settings.IMPULSE_ACCUMULATION)
            impulseSum = glm::clamp(impulseSum + lambda, -maxFriction, maxFriction);
        else
            impulseSum = glm::clamp(lambda, -maxFriction, maxFriction);
        break;
    }

    if (contact->settings.IMPULSE_ACCUMULATION)
        lambda = impulseSum - oldImpulseSum;
    else
        lambda = impulseSum;

    ApplyImpulse(lambda);
}

void ContactSolver::ApplyImpulse(float lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    contact->manifold.bodyA->linearVelocity += jacobian.va * (contact->manifold.bodyA->invMass * lambda);
    contact->manifold.bodyA->angularVelocity += contact->manifold.bodyA->invInertia * jacobian.wa * lambda;
    contact->manifold.bodyB->linearVelocity += jacobian.vb * (contact->manifold.bodyB->invMass * lambda);
    contact->manifold.bodyB->angularVelocity += contact->manifold.bodyB->invInertia * jacobian.wb * lambda;
}

} // namespace spe