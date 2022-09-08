#include "spe/position_solver.h"
#include "spe/contact.h"
#include "spe/util.h"
#include "spe/world.h"

namespace spe
{

void PositionSolver::Prepare(Contact* _contact, uint32_t index)
{
    contact = _contact;

    localPlainPoint = mul_t(contact->manifold.bodyA->GetTransform(), contact->manifold.referenceEdge.p1.position);
    localClipPoint = mul_t(contact->manifold.bodyB->GetTransform(), contact->manifold.contactPoints[index].position);
    localNormal = mul_t(contact->manifold.bodyA->GetRotation(), contact->manifold.contactNormal);
}

void PositionSolver::Solve()
{
    glm::vec2 normal = contact->manifold.bodyA->GetRotation() * localNormal;
    glm::vec2 planePoint = contact->manifold.bodyA->GetTransform() * localPlainPoint;
    glm::vec2 clipPoint = contact->manifold.bodyB->GetTransform() * localClipPoint; // penetration point

    float separation = glm::dot(clipPoint - planePoint, normal);
    assert(separation < 0.0f);

    glm::vec2 ra = clipPoint - contact->manifold.bodyA->GetPosition();
    glm::vec2 rb = clipPoint - contact->manifold.bodyB->GetPosition();

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
