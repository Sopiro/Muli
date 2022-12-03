#include "muli/position_solver.h"
#include "muli/contact.h"
#include "muli/util.h"
#include "muli/world.h"

namespace muli
{

void PositionSolver::Prepare(Contact* _contact, uint32 index)
{
    contact = _contact;

    Transform tA{ contact->b1->position, contact->b1->angle };
    Transform tB{ contact->b2->position, contact->b2->angle };

    localPlainPoint = MulT(tA, contact->manifold.referencePoint.position);
    localClipPoint = MulT(tB, contact->manifold.contactPoints[index].position);
    localNormal = MulT(tA.rotation, contact->manifold.contactNormal);
}

bool PositionSolver::Solve()
{
    Transform tA{ contact->b1->position, contact->b1->angle };
    Transform tB{ contact->b2->position, contact->b2->angle };

    Vec2 planePoint = tA * localPlainPoint;
    Vec2 clipPoint = tB * localClipPoint; // penetration point
    Vec2 normal = tA.rotation * localNormal;

    float separation = Dot(clipPoint - planePoint, normal);

    Vec2 ra = clipPoint - tA.position;
    Vec2 rb = clipPoint - tB.position;

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
    return -separation <= contact->settings.PENETRATION_SLOP * 4.0f;
}

} // namespace muli
