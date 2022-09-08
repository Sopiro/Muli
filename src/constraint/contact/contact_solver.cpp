#include "spe/contact_solver.h"
#include "spe/contact.h"
#include "spe/world.h"

namespace spe
{

void ContactSolver::Prepare(Contact* contact, uint32_t index, const Vec2& dir, Type contactType)
{
    // Calculate Jacobian J and effective mass M
    // J = [-dir, -ra × dir, dir, rb × dir] (dir: Contact vector, normal or tangent)
    // M = (J · M^-1 · J^t)^-1

    c = contact;
    p = contact->manifold.contactPoints[index].position;
    type = contactType;

    ra = p - c->manifold.bodyA->GetPosition();
    rb = p - c->manifold.bodyB->GetPosition();

    j.va = -dir;
    j.wa = cross(-ra, dir);
    j.vb = dir;
    j.wb = cross(rb, dir);

    bias = 0.0f;
    if (type == Normal)
    {
        // Relative velocity at contact point
        Vec2 relativeVelocity = (c->manifold.bodyB->linearVelocity + cross(c->manifold.bodyB->angularVelocity, rb)) -
                                (c->manifold.bodyA->linearVelocity + cross(c->manifold.bodyA->angularVelocity, ra));

        // Normal velocity == veclocity constraint: jv
        float normalVelocity = dot(c->manifold.contactNormal, relativeVelocity);

        // Position correction by velocity steering
        // if (c->settings.POSITION_CORRECTION)
        // {
        //     bias = -c->settings.POSITION_CORRECTION_BETA * c->settings.INV_DT *
        //            spe::max(c->manifold.penetrationDepth - c->settings.PENETRATION_SLOP, 0.0f);
        // }

#if 0
        if (-c->settings.RESTITUTION_SLOP > normalVelocity)
        {
            bias += c->restitution * normalVelocity;
        }
#else
        bias += c->restitution * spe::min(normalVelocity + c->settings.RESTITUTION_SLOP, 0.0f);
#endif
    }
    else
    {
        bias = -(c->manifold.bodyB->surfaceSpeed - c->manifold.bodyA->surfaceSpeed);

        if (c->manifold.featureFlipped)
        {
            bias *= -1;
        }
    }

    // clang-format off
    float k
        = c->manifold.bodyA->invMass
        + j.wa * c->manifold.bodyA->invInertia * j.wa
        + c->manifold.bodyB->invMass
        + j.wb * c->manifold.bodyB->invInertia * j.wb;
    // clang-format on

    effectiveMass = k > 0.0f ? 1.0f / k : 0.0f;

    if (c->settings.WARM_STARTING)
    {
        ApplyImpulse(impulseSum);
    }
}

void ContactSolver::Solve(const ContactSolver* normalContact)
{
    // Calculate corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    // clang-format off
    // Jacobian * velocity vector (Normal velocity)
    float jv = dot(j.va, c->manifold.bodyA->linearVelocity)
             + j.wa * c->manifold.bodyA->angularVelocity
             + dot(j.vb, c->manifold.bodyB->linearVelocity)
             + j.wb * c->manifold.bodyB->angularVelocity;
    // clang-format on

    // Clamp impulse correctly and accumulate it
    float lambda = effectiveMass * -(jv + bias);
    float oldImpulseSum = impulseSum;

    switch (type)
    {
    case Normal:
        impulseSum = spe::max(0.0f, impulseSum + lambda);
        break;
    case Tangent:
        float maxFriction = c->friction * normalContact->impulseSum;
        impulseSum = spe::clamp(impulseSum + lambda, -maxFriction, maxFriction);
        break;
    }

    lambda = impulseSum - oldImpulseSum;

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