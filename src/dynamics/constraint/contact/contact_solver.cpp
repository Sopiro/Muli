#include "muli/contact_solver.h"
#include "muli/contact.h"
#include "muli/world.h"

namespace muli
{

void ContactSolver::Prepare(Contact* contact, uint32 index, const Vec2& dir, Type contactType)
{
    // Calculate Jacobian J and effective mass M
    // J = [-dir, -ra × dir, dir, rb × dir] (dir: Contact vector, normal or tangent)
    // M = (J · M^-1 · J^t)^-1

    c = contact;
    p = contact->manifold.contactPoints[index].position;
    type = contactType;

    ra = p - c->b1->GetPosition();
    rb = p - c->b2->GetPosition();

    j.va = -dir;
    j.wa = Cross(-ra, dir);
    j.vb = dir;
    j.wb = Cross(rb, dir);

    bias = 0.0f;
    if (type == Normal)
    {
        // Relative velocity at contact point
        Vec2 relativeVelocity = (c->b2->linearVelocity + Cross(c->b2->angularVelocity, rb)) -
                                (c->b1->linearVelocity + Cross(c->b1->angularVelocity, ra));

        // Normal velocity == veclocity constraint: jv
        float normalVelocity = Dot(c->manifold.contactNormal, relativeVelocity);

        // Position correction by velocity steering
        // if (c->settings.POSITION_CORRECTION)
        // {
        //     bias = -c->settings.POSITION_CORRECTION_BETA * c->settings.INV_DT *
        //            Max(c->manifold.penetrationDepth - c->settings.PENETRATION_SLOP, 0.0f);
        // }

#if 0
        if (-c->settings.RESTITUTION_SLOP > normalVelocity)
        {
            bias += c->restitution * normalVelocity;
        }
#else
        bias += c->restitution * Min(normalVelocity + c->settings.RESTITUTION_SLOP, 0.0f);
#endif
    }
    else
    {
        bias = -(c->colliderB->GetSurfaceSpeed() - c->colliderA->GetSurfaceSpeed());
    }

    // clang-format off
    float k
        = c->b1->invMass
        + j.wa * c->b1->invInertia * j.wa
        + c->b2->invMass
        + j.wb * c->b2->invInertia * j.wb;
    // clang-format on

    m = k > 0.0f ? 1.0f / k : 0.0f;

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
    float jv = Dot(j.va, c->b1->linearVelocity)
             + j.wa * c->b1->angularVelocity
             + Dot(j.vb, c->b2->linearVelocity)
             + j.wb * c->b2->angularVelocity;
    // clang-format on

    // Clamp impulse correctly and accumulate it
    float lambda = m * -(jv + bias);
    float oldImpulseSum = impulseSum;

    switch (type)
    {
    case Normal:
        impulseSum = Max(0.0f, impulseSum + lambda);
        break;
    case Tangent:
        float maxFriction = c->friction * normalContact->impulseSum;
        impulseSum = Clamp(impulseSum + lambda, -maxFriction, maxFriction);
        break;
    }

    lambda = impulseSum - oldImpulseSum;

    ApplyImpulse(lambda);
}

inline void ContactSolver::ApplyImpulse(float lambda)
{
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    c->b1->linearVelocity += j.va * (c->b1->invMass * lambda);
    c->b1->angularVelocity += c->b1->invInertia * j.wa * lambda;
    c->b2->linearVelocity += j.vb * (c->b2->invMass * lambda);
    c->b2->angularVelocity += c->b2->invInertia * j.wb * lambda;
}

} // namespace muli