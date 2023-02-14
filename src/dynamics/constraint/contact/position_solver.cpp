#include "muli/position_solver.h"
#include "muli/contact.h"
#include "muli/util.h"
#include "muli/world.h"

namespace muli
{

void PositionSolver::Prepare(Contact* _contact, int32 index)
{
    contact = _contact;

    Transform tfA{ contact->b1->sweep.c, contact->b1->sweep.a };
    Transform tfB{ contact->b2->sweep.c, contact->b2->sweep.a };

    localPlainPoint = MulT(tfA, contact->manifold.referencePoint.position);
    localClipPoint = MulT(tfB, contact->manifold.contactPoints[index].position);
    localNormal = MulT(tfA.rotation, contact->manifold.contactNormal);
}

bool PositionSolver::Solve()
{
    Transform tfA{ contact->b1->sweep.c, contact->b1->sweep.a };
    Transform tfB{ contact->b2->sweep.c, contact->b2->sweep.a };

    Vec2 planePoint = Mul(tfA, localPlainPoint);
    Vec2 clipPoint = Mul(tfB, localClipPoint); // penetration point
    Vec2 normal = Mul(tfA.rotation, localNormal);

    float separation = Dot(clipPoint - planePoint, normal);

    Vec2 ra = clipPoint - tfA.position;
    Vec2 rb = clipPoint - tfB.position;

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
    float c = Clamp(position_correction * (separation + linear_slop), -max_position_correction, 0.0f);

    // Compute normal impulse
    float lambda = k > 0.0f ? -c / k : 0.0f;
    Vec2 impulse = normal * lambda;

    contact->cLinearImpulseA -= impulse;
    contact->cAngularImpulseA -= Cross(ra, impulse);
    contact->cLinearImpulseB += impulse;
    contact->cAngularImpulseB += Cross(rb, impulse);

    // We can't expect separation >= -linear_slop
    // because we don't push the separation above -linear_slop
    return -separation <= position_solver_threshold;
}

bool PositionSolver::SolveTOI()
{
    Transform tfA{ contact->b1->sweep.c, contact->b1->sweep.a };
    Transform tfB{ contact->b2->sweep.c, contact->b2->sweep.a };

    Vec2 planePoint = Mul(tfA, localPlainPoint);
    Vec2 clipPoint = Mul(tfB, localClipPoint); // penetration point
    Vec2 normal = Mul(tfA.rotation, localNormal);

    float separation = Dot(clipPoint - planePoint, normal);

    Vec2 ra = clipPoint - tfA.position;
    Vec2 rb = clipPoint - tfB.position;

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
    float c = Clamp(toi_position_correction * (separation + linear_slop), -max_position_correction, 0.0f);

    // Compute normal impulse
    float lambda = k > 0.0f ? -c / k : 0.0f;
    Vec2 impulse = normal * lambda;

    contact->cLinearImpulseA -= impulse;
    contact->cAngularImpulseA -= Cross(ra, impulse);
    contact->cLinearImpulseB += impulse;
    contact->cAngularImpulseB += Cross(rb, impulse);

    // TOI position solver must push further than the discrete position solver
    return -separation <= toi_position_solver_threshold;
}

} // namespace muli
