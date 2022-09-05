#include "spe/contact.h"
#include "spe/block_solver.h"
#include "spe/contact_solver.h"
#include "spe/world.h"

namespace spe
{

Contact::Contact(RigidBody* _bodyA, RigidBody* _bodyB, const Settings& _settings)
    : Constraint(_bodyA, _bodyB, _settings)
{
    manifold.numContacts = 0;

    beta = settings.POSITION_CORRECTION_BETA;
    restitution = mix_restitution(bodyA->restitution, bodyB->restitution);
    friction = mix_friction(bodyA->friction, bodyB->friction);
}

void Contact::Update()
{
    ContactManifold oldManifold = manifold;
    float oldNormalImpulse[MAX_CONTACT_POINT];
    float oldTangentImpulse[MAX_CONTACT_POINT];

    bool wasTouching = touching;
    touching = detect_collision(bodyA, bodyB, &manifold);

    for (uint32_t i = 0; i < MAX_CONTACT_POINT; i++)
    {
        oldNormalImpulse[i] = normalSolvers[i].impulseSum;
        normalSolvers[i].impulseSum = 0.0f;
        oldTangentImpulse[i] = tangentSolvers[i].impulseSum;
        tangentSolvers[i].impulseSum = 0.0f;
    }

    if (!touching)
    {
        return;
    }

    // Warm start the contact solver
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
            normalSolvers[n].impulseSum = oldNormalImpulse[o];
            tangentSolvers[n].impulseSum = oldTangentImpulse[o];

            persistent = true;
        }
    }
}

void Contact::Prepare()
{
    for (uint32_t i = 0; i < manifold.numContacts; i++)
    {
        normalSolvers[i].Prepare(this, i, manifold.contactNormal, ContactSolver::Type::Normal);
        tangentSolvers[i].Prepare(this, i, manifold.contactTangent, ContactSolver::Type::Tangent);
        positionSolvers[i].Prepare(this, i);
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
        tangentSolvers[i].Solve(&normalSolvers[i]);
    }

    if (manifold.numContacts == 1 || !settings.BLOCK_SOLVE)
    {
        for (uint32_t i = 0; i < manifold.numContacts; i++)
        {
            normalSolvers[i].Solve();
        }
    }
    else // Solve two contact constraint in one shot using block solver
    {
        blockSolver.Solve();
    }
}

void Contact::Solve2()
{
    cPosA = manifold.bodyA->position;
    cRotA = manifold.bodyA->rotation;
    cPosB = manifold.bodyB->position;
    cRotB = manifold.bodyB->rotation;
    // cPosA = { 0.0f, 0.0f };
    // cRotA = 0.0f;
    // cPosB = { 0.0f, 0.0f };
    // cRotB = 0.0f;

    // Solve position constraint
    for (uint32_t i = 0; i < manifold.numContacts; i++)
    {
        positionSolvers[i].Solve();
    }

    manifold.bodyA->position = cPosA;
    manifold.bodyA->rotation = cRotA;
    manifold.bodyB->position = cPosB;
    manifold.bodyB->rotation = cRotB;
}

} // namespace spe
