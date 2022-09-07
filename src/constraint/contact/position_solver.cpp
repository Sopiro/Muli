#include "spe/position_solver.h"
#include "spe/contact.h"
#include "spe/util.h"
#include "spe/world.h"

namespace spe
{

void PositionSolver::Prepare(Contact* _contact, uint32_t index)
{
    contact = _contact;

    localPlainPoint = contact->manifold.bodyA->GlobalToLocal() * contact->manifold.referenceEdge.p1.position;
    localClipPoint = contact->manifold.bodyB->GlobalToLocal() * contact->manifold.contactPoints[index].position;
    localNormal = glm::mul(contact->manifold.bodyA->GlobalToLocal(), contact->manifold.contactNormal, 0.0f);
}

void PositionSolver::Solve()
{
    glm::vec2 normal = glm::mul(contact->manifold.bodyA->LocalToGlobal(), localNormal, 0.0f);
    glm::vec2 planePoint = contact->manifold.bodyA->LocalToGlobal() * localPlainPoint;
    glm::vec2 clipPoint = contact->manifold.bodyB->LocalToGlobal() * localClipPoint; // penetration point

    float separation = glm::dot(clipPoint - planePoint, normal);
    assert(separation < 0.0f);

    glm::vec2 ra = clipPoint - contact->manifold.bodyA->position;
    glm::vec2 rb = clipPoint - contact->manifold.bodyB->position;

    float ran = glm::cross(ra, normal);
    float rbn = glm::cross(rb, normal);

    // clang-format off
    // effective mass = 1 / k;
    float k = contact->manifold.bodyA->invMass
            + ran * contact->manifold.bodyA->invInertia * ran
            + contact->manifold.bodyB->invMass
            + rbn * contact->manifold.bodyB->invInertia * rbn;
    // clang-format on

    // Constraint (bias)
    float c = glm::min(contact->settings.POSITION_CORRECTION_BETA * (separation + contact->settings.PENETRATION_SLOP), 0.0f);

    // Compute normal impulse
    float lambda = k > 0.0f ? -c / k : 0.0f;
    glm::vec2 impulse = normal * lambda;

    contact->cLinearImpulseA -= impulse;
    contact->cAngularImpulseA -= glm::cross(ra, impulse);
    contact->cLinearImpulseB += impulse;
    contact->cAngularImpulseB += glm::cross(rb, impulse);
}

} // namespace spe
