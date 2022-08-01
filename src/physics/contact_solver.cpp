#include "spe/contact_solver.h"
#include "spe/world.h"
#include "spe/contact_constraint.h"

namespace spe
{

ContactSolver::ContactSolver(ContactConstraint& _cc, const glm::vec2& _contactPoint) :
    cc{ _cc },
    contactPoint{ _contactPoint }
{
    beta = cc.settings.POSITION_CORRECTION_BETA;
    restitution = cc.bodyA->restitution * cc.bodyB->restitution;
    friction = cc.bodyA->friction * cc.bodyB->friction;
}

void ContactSolver::Prepare(const glm::vec2& dir, ContactType _contactType)
{
    // Calculate Jacobian J and effective mass M
    // J = [-dir, -ra × dir, dir, rb × dir] (dir: Contact vector, normal or tangent)
    // M = (J · M^-1 · J^t)^-1

    contactType = _contactType;

    ra = contactPoint - cc.bodyA->position;
    rb = contactPoint - cc.bodyB->position;

    jacobian.va = -dir;
    jacobian.wa = glm::cross(-ra, dir);
    jacobian.vb = dir;
    jacobian.wb = glm::cross(rb, dir);

    bias = 0.0f;
    if (contactType == Normal)
    {
        // Relative velocity at contact point
        glm::vec2 relativeVelocity = (cc.bodyB->linearVelocity + glm::cross(cc.bodyB->angularVelocity, rb)) -
            (cc.bodyA->linearVelocity + glm::cross(cc.bodyA->angularVelocity, ra));

        float normalVelocity = glm::dot(cc.contactNormal, relativeVelocity);

        if (cc.settings.POSITION_CORRECTION)
            bias = -(beta * cc.settings.INV_DT) * glm::max(cc.penetrationDepth - cc.settings.PENETRATION_SLOP, 0.0f);

        bias += restitution * glm::min(normalVelocity + cc.settings.RESTITUTION_SLOP, 0.0f);
    }
    else
    {
        bias = -(cc.bodyB->surfaceSpeed - cc.bodyA->surfaceSpeed);

        if (cc.featureFlipped)
            bias *= -1;
    }

    float k = cc.bodyA->invMass
        + jacobian.wa * cc.bodyA->invInertia * jacobian.wa
        + cc.bodyB->invMass
        + jacobian.wb * cc.bodyB->invInertia * jacobian.wb;

    effectiveMass = k > 0.0f ? 1.0f / k : 0.0f;

    if (cc.settings.WARM_STARTING) ApplyImpulse(impulseSum);
}

void ContactSolver::Solve(const ContactSolver* normalContact)
{
    // Calculate corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    // Jacobian * velocity vector (Normal velocity)
    float jv = glm::dot(jacobian.va, cc.bodyA->linearVelocity)
        + jacobian.wa * cc.bodyA->angularVelocity
        + glm::dot(jacobian.vb, cc.bodyB->linearVelocity)
        + jacobian.wb * cc.bodyB->angularVelocity;


    float lambda = effectiveMass * -(jv + bias);

    float oldImpulseSum = impulseSum;
    switch (contactType)
    {
    case Normal:
        if (cc.settings.IMPULSE_ACCUMULATION)
            impulseSum = glm::max(0.0f, impulseSum + lambda);
        else
            impulseSum = glm::max(0.0f, lambda);
        break;
    case Tangent:
        float maxFriction = friction * normalContact->impulseSum;
        if (cc.settings.IMPULSE_ACCUMULATION)
            impulseSum = glm::clamp(impulseSum + lambda, -maxFriction, maxFriction);
        else
            impulseSum = glm::clamp(lambda, -maxFriction, maxFriction);
        break;
    }

    if (cc.settings.IMPULSE_ACCUMULATION)
        lambda = impulseSum - oldImpulseSum;
    else
        lambda = impulseSum;

    ApplyImpulse(lambda);
}

void ContactSolver::ApplyImpulse(float lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    cc.bodyA->linearVelocity += jacobian.va * (cc.bodyA->invMass * lambda);
    cc.bodyA->angularVelocity += cc.bodyA->invInertia * jacobian.wa * lambda;
    cc.bodyB->linearVelocity += jacobian.vb * (cc.bodyB->invMass * lambda);
    cc.bodyB->angularVelocity += cc.bodyB->invInertia * jacobian.wb * lambda;
}

}