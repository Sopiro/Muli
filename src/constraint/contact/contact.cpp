#include "spe/contact.h"
#include "spe/block_solver.h"
#include "spe/contact_solver.h"
#include "spe/world.h"

namespace spe
{

void Contact::Update()
{
    ContactManifold oldManifold = manifold;

    bool wasTouching = touching;
    touching = detect_collision(bodyA, bodyB, &manifold);

    if (!touching) return;

    // Warm start the contact solver
    float oldNormalImpulse[2];
    float oldTangentImpulse[2];

    for (uint32_t i = 0; i < oldManifold.numContacts; i++)
    {
        oldNormalImpulse[i] = normalContacts[i].impulseSum;
        normalContacts[i].impulseSum = 0.0f;
        oldTangentImpulse[i] = tangentContacts[i].impulseSum;
        tangentContacts[i].impulseSum = 0.0f;
    }

    for (uint32_t n = 0; n < manifold.numContacts; n++)
    {
        uint32_t o = 0;
        for (; o < oldManifold.numContacts; o++)
        {
            if (manifold.contactPoints[n].id == oldManifold.contactPoints[o].id)
            {
                if (settings.APPLY_WARM_STARTING_THRESHOLD)
                {
                    float dist = glm::distance2(manifold.contactPoints[n].point, oldManifold.contactPoints[o].point);
                    // If contact points are close enough, warm start.
                    // Otherwise, it means it's penetrating too deeply, skip the warm starting to prevent the overshoot
                    if (dist < settings.WARM_STARTING_THRESHOLD) break;
                }
                else
                {
                    break;
                }
            }
        }

        if (o < oldManifold.numContacts)
        {
            normalContacts[n].impulseSum = oldNormalImpulse[o];
            tangentContacts[n].impulseSum = oldTangentImpulse[o];

            persistent = true;
        }
    }
}

void Contact::Prepare()
{
    for (uint32_t i = 0; i < manifold.numContacts; i++)
    {
        normalContacts[i].Prepare(this, manifold.contactPoints[i].point, manifold.contactNormal, ContactSolver::Type::Normal);
        tangentContacts[i].Prepare(this, manifold.contactPoints[i].point, manifold.contactTangent, ContactSolver::Type::Tangent);
    }

    if (manifold.numContacts == 2 && settings.BLOCK_SOLVE)
    {
        blockSolver.Prepare(this);
    }
}

void Contact::Solve()
{
    // Solve tangential constraint first
    for (uint32_t i = 0; i < manifold.numContacts; i++)
    {
        tangentContacts[i].Solve(&normalContacts[i]);
    }

    if (manifold.numContacts == 1 || !settings.BLOCK_SOLVE)
    {
        for (uint32_t i = 0; i < manifold.numContacts; i++)
        {
            normalContacts[i].Solve();
        }
    }
    else // Solve two contact constraint in one shot using block solver
    {
        blockSolver.Solve();
    }
}

} // namespace spe
