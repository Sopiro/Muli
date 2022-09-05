#include "spe/position_solver.h"
#include "spe/contact.h"
#include "spe/util.h"
#include "spe/world.h"

namespace spe
{

void PositionSolver::Prepare(Contact* contact, uint32_t index)
{
    c = contact;

    localPA = c->manifold.bodyA->GlobalToLocal() * contact->manifold.contactPoints[index].point;
    localPB = c->manifold.bodyB->GlobalToLocal() *
              (contact->manifold.contactPoints[index].point - c->manifold.contactNormal * contact->manifold.penetrationDepth);
    localNormal = glm::mul(c->manifold.bodyA->GlobalToLocal(), c->manifold.contactNormal, 0.0f);
}

void PositionSolver::Solve()
{
    glm::vec2 normal = glm::mul(c->manifold.bodyA->LocalToGlobal(), localNormal, 0.0f);
    glm::vec2 pa = c->manifold.bodyA->LocalToGlobal() * localPA; // penetration point
    glm::vec2 pb = c->manifold.bodyB->LocalToGlobal() * localPB;

    float penetration = glm::dot(pa - pb, normal);
    assert(penetration > 0.0f);

    glm::vec2 ra = pa - c->manifold.bodyA->position;
    glm::vec2 rb = pa - c->manifold.bodyB->position;

    // Track max constraint error
    float correction = glm::max(c->settings.POSITION_CORRECTION_BETA * (penetration - c->settings.PENETRATION_SLOP), 0.0f);

    float ran = glm::cross(ra, normal);
    float rbn = glm::cross(rb, normal);

    // clang-format off
    float k = c->manifold.bodyA->invMass
            + ran * c->manifold.bodyA->invInertia * ran
            + c->manifold.bodyB->invMass
            + rbn * c->manifold.bodyB->invInertia * rbn;
    // clang-format on

    // Compute normal impulse
    float impulse = k > 0.0f ? correction / k : 0.0f;
    glm::vec2 pv = normal * impulse;

    c->cPosA -= c->manifold.bodyA->invMass * pv;
    c->cRotA -= c->manifold.bodyA->invInertia * glm::cross(ra, pv);
    c->cPosB += c->manifold.bodyB->invMass * pv;
    c->cRotB += c->manifold.bodyB->invInertia * glm::cross(rb, pv);
}

} // namespace spe
