#include "spe/position_solver.h"
#include "spe/contact.h"
#include "spe/util.h"
#include "spe/world.h"

namespace spe
{

void PositionSolver::Prepare(Contact* _contact, uint32 index)
{
    contact = _contact;

    localPlainPoint = MulT(contact->manifold.bodyA->GetTransform(), contact->manifold.referencePoint.position);
    localClipPoint = MulT(contact->manifold.bodyB->GetTransform(), contact->manifold.contactPoints[index].position);
    localNormal = MulT(contact->manifold.bodyA->GetRotation(), contact->manifold.contactNormal);
}

bool PositionSolver::Solve()
{
    Vec2 normal = contact->manifold.bodyA->GetRotation() * localNormal;
    Vec2 planePoint = contact->manifold.bodyA->GetTransform() * localPlainPoint;
    Vec2 clipPoint = contact->manifold.bodyB->GetTransform() * localClipPoint; // penetration point

    float separation = Dot(clipPoint - planePoint, normal);

    Vec2 ra = clipPoint - contact->manifold.bodyA->GetPosition();
    Vec2 rb = clipPoint - contact->manifold.bodyB->GetPosition();

    float ran = Cross(ra, normal);
    float rbn = Cross(rb, normal);

    // clang-format off
    // effective mass = 1 / k;
    float k = contact->manifold.bodyA->invMass
            + ran * contact->manifold.bodyA->invInertia * ran
            + contact->manifold.bodyB->invMass
            + rbn * contact->manifold.bodyB->invInertia * rbn;
    // clang-format on

    // Constraint (bias)
    float c = Min(contact->settings.POSITION_CORRECTION_BETA * (separation + contact->settings.PENETRATION_SLOP), 0.0f);

    // Compute normal impulse
    float lambda = k > 0.0f ? -c / k : 0.0f;
    Vec2 impulse = normal * lambda;

    contact->cLinearImpulseA -= impulse;
    contact->cAngularImpulseA -= Cross(ra, impulse);
    contact->cLinearImpulseB += impulse;
    contact->cAngularImpulseB += Cross(rb, impulse);

    // We can't expect speparation >= -PENETRATION_SLOP
    // because we don't push the separation above -PENETRATION_SLOP
    return -separation <= contact->settings.PENETRATION_SLOP * 2.0f;
}

} // namespace spe
