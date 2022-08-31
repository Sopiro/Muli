#include "spe/contact_solver.h"
#include "spe/contact.h"
#include "spe/world.h"

namespace spe
{

void ContactSolver::Prepare(const glm::vec2& dir, ContactType _contactType)
{
    // Calculate Jacobian J and effective mass M
    // J = [-dir, -ra × dir, dir, rb × dir] (dir: Contact vector, normal or tangent)
    // M = (J · M^-1 · J^t)^-1

    beta = contact->settings.POSITION_CORRECTION_BETA;
    restitution = contact->bodyA->restitution * contact->bodyB->restitution;
    friction = contact->bodyA->friction * contact->bodyB->friction;

    contactType = _contactType;

    ra = contactPoint - contact->bodyA->position;
    rb = contactPoint - contact->bodyB->position;

    jacobian.va = -dir;
    jacobian.wa = glm::cross(-ra, dir);
    jacobian.vb = dir;
    jacobian.wb = glm::cross(rb, dir);

    bias = 0.0f;
    if (contactType == Normal)
    {
        // Relative velocity at contact point
        glm::vec2 relativeVelocity = (contact->bodyB->linearVelocity + glm::cross(contact->bodyB->angularVelocity, rb)) -
                                     (contact->bodyA->linearVelocity + glm::cross(contact->bodyA->angularVelocity, ra));

        float normalVelocity = glm::dot(contact->manifold.contactNormal, relativeVelocity);

        if (contact->settings.POSITION_CORRECTION)
            bias = -(beta * contact->settings.INV_DT) *
                   glm::max(contact->manifold.penetrationDepth - contact->settings.PENETRATION_SLOP, 0.0f);

        bias += restitution * glm::min(normalVelocity + contact->settings.RESTITUTION_SLOP, 0.0f);
    }
    else
    {
        bias = -(contact->bodyB->surfaceSpeed - contact->bodyA->surfaceSpeed);

        if (contact->manifold.featureFlipped) bias *= -1;
    }

    // clang-format off
    float k
        = contact->bodyA->invMass
        + jacobian.wa * contact->bodyA->invInertia * jacobian.wa
        + contact->bodyB->invMass
        + jacobian.wb * contact->bodyB->invInertia * jacobian.wb;
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
    float jv = glm::dot(jacobian.va, contact->bodyA->linearVelocity)
        + jacobian.wa * contact->bodyA->angularVelocity
        + glm::dot(jacobian.vb, contact->bodyB->linearVelocity)
        + jacobian.wb * contact->bodyB->angularVelocity;
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

    contact->bodyA->linearVelocity += jacobian.va * (contact->bodyA->invMass * lambda);
    contact->bodyA->angularVelocity += contact->bodyA->invInertia * jacobian.wa * lambda;
    contact->bodyB->linearVelocity += jacobian.vb * (contact->bodyB->invMass * lambda);
    contact->bodyB->angularVelocity += contact->bodyB->invInertia * jacobian.wb * lambda;
}

} // namespace spe