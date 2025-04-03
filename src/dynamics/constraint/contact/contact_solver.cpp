#include "muli/contact_solver.h"
#include "muli/contact.h"
#include "muli/world.h"

namespace muli
{

void ContactSolverNormal::Prepare(const Contact* c, const Vec2& normal, int32 index, const Timestep& step)
{
    // Compute Jacobian J and effective mass M
    // J = [-n, -ra × n, n, rb × n]
    // M = (J · M^-1 · J^t)^-1

    Vec2 point = c->manifold.contactPoints[index].p;
    Vec2 ra = point - c->b1->motion.c;
    Vec2 rb = point - c->b2->motion.c;

    // Setup jacobian
    j.va = -normal;
    j.wa = -Cross(ra, normal);
    j.vb = normal;
    j.wb = Cross(rb, normal);

    bias = 0.0f;

    // Relative velocity at contact point
    Vec2 relativeVelocity =
        (c->b2->linearVelocity + Cross(c->b2->angularVelocity, rb)) - (c->b1->linearVelocity + Cross(c->b1->angularVelocity, ra));

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

#if 0
    // Position correction by velocity steering
    bias += -position_correction * step.inv_dt * Max(c->manifold.penetrationDepth - linear_slop, 0.0f);
#endif

    // clang-format off
    float k = c->b1->invMass
            + j.wa * c->b1->invInertia * j.wa
            + c->b2->invMass
            + j.wb * c->b2->invInertia * j.wb;
    // clang-format on

    m = k > 0.0f ? 1.0f / k : 0.0f;

    if (step.warm_starting)
    {
        // Warm start
        c->b1->linearVelocity += j.va * (c->b1->invMass * impulse);
        c->b1->angularVelocity += c->b1->invInertia * j.wa * impulse;
        c->b2->linearVelocity += j.vb * (c->b2->invMass * impulse);
        c->b2->angularVelocity += c->b2->invInertia * j.wb * impulse;
    }
}

void ContactSolverNormal::Solve(Contact* c)
{
    // Compute corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    // clang-format off
    // Velocity constraint: C' = jv
    // Jacobian * velocity vector (Normal velocity)
    float jv = Dot(j.va, c->b1->linearVelocity)
             + j.wa * c->b1->angularVelocity
             + Dot(j.vb, c->b2->linearVelocity)
             + j.wb * c->b2->angularVelocity;
    // clang-format on

    float lambda = m * -(jv + bias);

    // Clamp impulse correctly and accumulate it
    float oldImpulse = impulse;
    impulse = Max(0.0f, impulse + lambda);
    lambda = impulse - oldImpulse;

    // Apply impulse
    // V2 = V2' + M^-1 ⋅ Pc
    // Pc = J^t ⋅ λ

    c->b1->linearVelocity += j.va * (c->b1->invMass * lambda);
    c->b1->angularVelocity += c->b1->invInertia * j.wa * lambda;
    c->b2->linearVelocity += j.vb * (c->b2->invMass * lambda);
    c->b2->angularVelocity += c->b2->invInertia * j.wb * lambda;
}

void ContactSolverTangent::Prepare(const Contact* c, const Vec2& tangent, int32 index, const Timestep& step)
{
    // Compute Jacobian J and effective mass M
    // J = [-n, -ra × n, n, rb × n]
    // M = (J · M^-1 · J^t)^-1

    Vec2 point = c->manifold.contactPoints[index].p;
    Vec2 ra = point - c->b1->motion.c;
    Vec2 rb = point - c->b2->motion.c;

    // Setup jacobian
    j.va = -tangent;
    j.wa = -Cross(ra, tangent);
    j.vb = tangent;
    j.wb = Cross(rb, tangent);

    bias = -c->surfaceSpeed;

    // clang-format off
    float k = c->b1->invMass
            + j.wa * c->b1->invInertia * j.wa
            + c->b2->invMass
            + j.wb * c->b2->invInertia * j.wb;
    // clang-format on

    m = k > 0.0f ? 1.0f / k : 0.0f;

    if (step.warm_starting)
    {
        // Warm start
        c->b1->linearVelocity += j.va * (c->b1->invMass * impulse);
        c->b1->angularVelocity += c->b1->invInertia * j.wa * impulse;
        c->b2->linearVelocity += j.vb * (c->b2->invMass * impulse);
        c->b2->angularVelocity += c->b2->invInertia * j.wb * impulse;
    }
}

void ContactSolverTangent::Solve(Contact* c, const ContactSolverNormal* normalContact)
{
    // Compute corrective impulse: Pc
    // Pc = J^t * λ (λ: lagrangian multiplier)
    // λ = (J · M^-1 · J^t)^-1 ⋅ -(J·v+b)

    // clang-format off
    // Velocity constraint: C' = jv
    // Jacobian * velocity vector (Normal velocity)
    float jv = Dot(j.va, c->b1->linearVelocity)
             + j.wa * c->b1->angularVelocity
             + Dot(j.vb, c->b2->linearVelocity)
             + j.wb * c->b2->angularVelocity;
    // clang-format on

    float lambda = m * -(jv + bias);

    // Clamp impulse correctly and accumulate it
    float oldImpulse = impulse;
    float maxFriction = c->friction * normalContact->impulse;
    impulse = Clamp(impulse + lambda, -maxFriction, maxFriction);
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