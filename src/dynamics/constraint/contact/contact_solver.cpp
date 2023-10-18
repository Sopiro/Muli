#include "muli/contact_solver.h"
#include "muli/contact.h"
#include "muli/world.h"

namespace muli
{

void ContactSolver::Prepare(Contact* contact, int32 index, const Vec2& dir, Type contactType, const WorldSettings& settings)
{
    // Compute Jacobian J and effective mass M
    // J = [-dir, -ra × dir, dir, rb × dir] (dir: Contact vector, normal or tangent)
    // M = (J · M^-1 · J^t)^-1

    c = contact;
    p = contact->manifold.contactPoints[index].position;
    type = contactType;

    Vec2 ra = p - c->b1->sweep.c;
    Vec2 rb = p - c->b2->sweep.c;

    j.va = -dir;
    j.wa = -Cross(ra, dir);
    j.vb = dir;
    j.wb = Cross(rb, dir);

    bias = 0.0f;
    if (type == Type::normal)
    {
        // Relative velocity at contact point
        Vec2 relativeVelocity = (c->b2->linearVelocity + Cross(c->b2->angularVelocity, rb)) -
                                (c->b1->linearVelocity + Cross(c->b1->angularVelocity, ra));

        // Normal velocity == veclocity constraint: jv
        float normalVelocity = Dot(c->manifold.contactNormal, relativeVelocity);

#if 1
        if (-normalVelocity > c->restitutionThreshold)
        {
            bias = c->restitution * normalVelocity;
        }
#else
        bias = c->restitution * Min(normalVelocity + c->restitutionThreshold, 0.0f);
#endif

        // Position correction by velocity steering
        // bias += -position_correction * settings.inv_dt * Max(c->manifold.penetrationDepth - linear_slop, 0.0f);
    }
    else
    {
        bias = -c->surfaceSpeed;
    }

    // clang-format off
    float k = c->b1->invMass
            + j.wa * c->b1->invInertia * j.wa
            + c->b2->invMass
            + j.wb * c->b2->invInertia * j.wb;
    // clang-format on

    m = k > 0.0f ? 1.0f / k : 0.0f;

    if (settings.warm_starting)
    {
        // Warm start
        c->b1->linearVelocity += j.va * (c->b1->invMass * impulse);
        c->b1->angularVelocity += c->b1->invInertia * j.wa * impulse;
        c->b2->linearVelocity += j.vb * (c->b2->invMass * impulse);
        c->b2->angularVelocity += c->b2->invInertia * j.wb * impulse;
    }
}

void ContactSolver::Solve(const ContactSolver* normalContact)
{
    // Compute corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    // clang-format off
    // Velocity constraint: C = jv
    // Jacobian * velocity vector (Normal velocity)
    float jv = Dot(j.va, c->b1->linearVelocity)
             + j.wa * c->b1->angularVelocity
             + Dot(j.vb, c->b2->linearVelocity)
             + j.wb * c->b2->angularVelocity;
    // clang-format on

    float lambda = m * -(jv + bias);

    // Clamp impulse correctly and accumulate it
    float oldImpulse = impulse;
    switch (type)
    {
    case Type::normal:
        impulse = Max(0.0f, impulse + lambda);
        break;
    case Type::tangent:
        float maxFriction = c->friction * normalContact->impulse;
        impulse = Clamp(impulse + lambda, -maxFriction, maxFriction);
        break;
    }

    lambda = impulse - oldImpulse;

    // Apply impulse
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    c->b1->linearVelocity += j.va * (c->b1->invMass * lambda);
    c->b1->angularVelocity += c->b1->invInertia * j.wa * lambda;
    c->b2->linearVelocity += j.vb * (c->b2->invMass * lambda);
    c->b2->angularVelocity += c->b2->invInertia * j.wb * lambda;
}

} // namespace muli