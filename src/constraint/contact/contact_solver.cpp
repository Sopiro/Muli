#include "spe/contact_solver.h"
#include "spe/contact.h"
#include "spe/world.h"

namespace spe
{

void ContactSolver::Prepare(Contact* contact, const glm::vec2& contactPoint, const glm::vec2& dir, Type contactType)
{
    // Calculate Jacobian J and effective mass M
    // J = [-dir, -ra × dir, dir, rb × dir] (dir: Contact vector, normal or tangent)
    // M = (J · M^-1 · J^t)^-1

    c = contact;
    p = contactPoint;
    type = contactType;

    ra = p - c->manifold.bodyA->position;
    rb = p - c->manifold.bodyB->position;

    j.va = -dir;
    j.wa = glm::cross(-ra, dir);
    j.vb = dir;
    j.wb = glm::cross(rb, dir);

    bias = 0.0f;
    if (type == Normal)
    {
        // Relative velocity at c point
        glm::vec2 relativeVelocity = (c->manifold.bodyB->linearVelocity + glm::cross(c->manifold.bodyB->angularVelocity, rb)) -
                                     (c->manifold.bodyA->linearVelocity + glm::cross(c->manifold.bodyA->angularVelocity, ra));

        float normalVelocity = glm::dot(c->manifold.contactNormal, relativeVelocity);

        if (c->settings.POSITION_CORRECTION)
            bias = -(c->beta * c->settings.INV_DT) * glm::max(c->manifold.penetrationDepth - c->settings.PENETRATION_SLOP, 0.0f);

        bias += c->restitution * glm::min(normalVelocity + c->settings.RESTITUTION_SLOP, 0.0f);
    }
    else
    {
        bias = -(c->manifold.bodyB->surfaceSpeed - c->manifold.bodyA->surfaceSpeed);

        if (c->manifold.featureFlipped) bias *= -1;
    }

    // clang-format off
    float k
        = c->manifold.bodyA->invMass
        + j.wa * c->manifold.bodyA->invInertia * j.wa
        + c->manifold.bodyB->invMass
        + j.wb * c->manifold.bodyB->invInertia * j.wb;
    // clang-format on

    effectiveMass = k > 0.0f ? 1.0f / k : 0.0f;

    if (c->settings.WARM_STARTING) ApplyImpulse(impulseSum);
}

void ContactSolver::Solve(const ContactSolver* normalContact)
{
    // Calculate corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    // clang-format off
    // Jacobian * velocity vector (Normal velocity)
    float jv = glm::dot(j.va, c->manifold.bodyA->linearVelocity)
        + j.wa * c->manifold.bodyA->angularVelocity
        + glm::dot(j.vb, c->manifold.bodyB->linearVelocity)
        + j.wb * c->manifold.bodyB->angularVelocity;
    // clang-format on

    float lambda = effectiveMass * -(jv + bias);

    float oldImpulseSum = impulseSum;
    switch (type)
    {
    case Normal:
        if (c->settings.IMPULSE_ACCUMULATION)
            impulseSum = glm::max(0.0f, impulseSum + lambda);
        else
            impulseSum = glm::max(0.0f, lambda);
        break;
    case Tangent:
        float maxFriction = c->friction * normalContact->impulseSum;
        if (c->settings.IMPULSE_ACCUMULATION)
            impulseSum = glm::clamp(impulseSum + lambda, -maxFriction, maxFriction);
        else
            impulseSum = glm::clamp(lambda, -maxFriction, maxFriction);
        break;
    }

    if (c->settings.IMPULSE_ACCUMULATION)
        lambda = impulseSum - oldImpulseSum;
    else
        lambda = impulseSum;

    ApplyImpulse(lambda);
}

inline void ContactSolver::ApplyImpulse(float lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    c->manifold.bodyA->linearVelocity += j.va * (c->manifold.bodyA->invMass * lambda);
    c->manifold.bodyA->angularVelocity += c->manifold.bodyA->invInertia * j.wa * lambda;
    c->manifold.bodyB->linearVelocity += j.vb * (c->manifold.bodyB->invMass * lambda);
    c->manifold.bodyB->angularVelocity += c->manifold.bodyB->invInertia * j.wb * lambda;
}

} // namespace spe