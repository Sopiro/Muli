#include "muli/position_solver.h"
#include "muli/contact.h"
#include "muli/util.h"
#include "muli/world.h"

namespace muli
{

void PositionSolver::Prepare(Contact* _contact, uint32 index)
{
    contact = _contact;

    localPlainPoint = MulT(contact->b1->GetTransform(), contact->manifold.referencePoint.position);
    localClipPoint = MulT(contact->b2->GetTransform(), contact->manifold.contactPoints[index].position);
    localNormal = MulT(contact->b1->GetRotation(), contact->manifold.contactNormal);
}

bool PositionSolver::Solve()
{
    Vec2 normal = contact->b1->GetRotation() * localNormal;
    Vec2 planePoint = contact->b1->GetTransform() * localPlainPoint;
    Vec2 clipPoint = contact->b2->GetTransform() * localClipPoint; // penetration point

    float separation = Dot(clipPoint - planePoint, normal);

    Vec2 ra = clipPoint - contact->b1->GetPosition();
    Vec2 rb = clipPoint - contact->b2->GetPosition();

    float ran = Cross(ra, normal);
    float rbn = Cross(rb, normal);

    // clang-format off
    // effective mass = 1 / k;
    float k = contact->b1->invMass
            + ran * contact->b1->invInertia * ran
            + contact->b2->invMass
            + rbn * contact->b2->invInertia * rbn;
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

    // We can't expect separation >= -PENETRATION_SLOP
    // because we don't push the separation above -PENETRATION_SLOP
    return -separation <= contact->settings.PENETRATION_SLOP * 3.0f;
}

} // namespace muli
