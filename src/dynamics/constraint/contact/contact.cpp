#include "muli/contact.h"
#include "muli/block_solver.h"
#include "muli/callbacks.h"
#include "muli/contact_solver.h"
#include "muli/settings.h"
#include "muli/world.h"

namespace muli
{

bool block_solve = true;
extern CollideFunction* collide_function_map[Shape::Type::shape_count][Shape::Type::shape_count];

Contact::Contact(Collider* colliderA, Collider* colliderB)
    : Constraint(colliderA->body, colliderB->body)
    , colliderA{ colliderA }
    , colliderB{ colliderB }
    , flag{ 0 }
    , toiCount{ 0 }
    , toi{ 0.0f }
{
    MuliAssert(colliderA->GetType() >= colliderB->GetType());

    manifold.contactCount = 0;

    friction = MixFriction(colliderA->GetFriction(), colliderB->GetFriction());
    restitution = MixRestitution(colliderA->GetRestitution(), colliderB->GetRestitution());
    restitutionThreshold = MixRestitutionTreshold(colliderA->GetRestitutionTreshold(), colliderB->GetRestitutionTreshold());
    surfaceSpeed = colliderB->GetSurfaceSpeed() - colliderA->GetSurfaceSpeed();

    collideFunction = collide_function_map[colliderA->GetType()][colliderB->GetType()];
    MuliAssert(collideFunction != nullptr);
}

void Contact::Update()
{
    flag |= flag_enabled;

    ContactManifold oldManifold = manifold;
    for (int32 i = 0; i < max_contact_point_count; ++i)
    {
        normalSolvers[i].impulseSave = normalSolvers[i].impulse;
        tangentSolvers[i].impulseSave = tangentSolvers[i].impulse;
        normalSolvers[i].impulse = 0.0f;
        tangentSolvers[i].impulse = 0.0f;
    }

    // clang-format off
    bool wasTouching = (flag & flag_touching) == flag_touching;
    bool touching = collideFunction(colliderA->shape, bodyA->transform,
                                    colliderB->shape, bodyB->transform,
                                    &manifold);
    // clang-format on

    if (touching == true)
    {
        flag |= flag_touching;
    }
    else
    {
        flag &= ~flag_touching;
    }

    if (touching == false)
    {
        if (wasTouching == true)
        {
            if (colliderA->ContactListener) colliderA->ContactListener->OnContactEnd(colliderA, colliderB, this);
            if (colliderB->ContactListener) colliderB->ContactListener->OnContactEnd(colliderB, colliderA, this);
        }

        return;
    }

    if (manifold.featureFlipped)
    {
        b1 = bodyB;
        b2 = bodyA;
    }
    else
    {
        b1 = bodyA;
        b2 = bodyB;
    }

    // Restore the impulses to warm start the solver
    for (int32 n = 0; n < manifold.contactCount; ++n)
    {
        for (int32 o = 0; o < oldManifold.contactCount; ++o)
        {
            if (manifold.contactPoints[n].id == oldManifold.contactPoints[o].id)
            {
                normalSolvers[n].impulse = normalSolvers[o].impulseSave;
                tangentSolvers[n].impulse = tangentSolvers[o].impulseSave;
                break;
            }
        }
    }

    if (touching == true)
    {
        if (wasTouching == false)
        {
            if (colliderA->ContactListener) colliderA->ContactListener->OnContactBegin(colliderA, colliderB, this);
            if (colliderB->ContactListener) colliderB->ContactListener->OnContactBegin(colliderB, colliderA, this);
        }
        else
        {
            if (colliderA->ContactListener) colliderA->ContactListener->OnContactTouching(colliderA, colliderB, this);
            if (colliderB->ContactListener) colliderB->ContactListener->OnContactTouching(colliderB, colliderA, this);
        }

        if (colliderA->ContactListener) colliderA->ContactListener->OnPreSolve(colliderA, colliderB, this);
        if (colliderB->ContactListener) colliderB->ContactListener->OnPreSolve(colliderB, colliderA, this);
    }

    if (colliderA->IsEnabled() == false || colliderB->IsEnabled() == false)
    {
        flag &= ~flag_enabled;
    }
}

void Contact::Prepare(const Timestep& step)
{
    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        normalSolvers[i].Prepare(this, ContactSolver::Type::normal, manifold.contactNormal, i, step);
        tangentSolvers[i].Prepare(this, ContactSolver::Type::tangent, manifold.contactTangent, i, step);
        positionSolvers[i].Prepare(this, i);
    }

    if (manifold.contactCount == 2 && block_solve == true)
    {
        blockSolver.Prepare(this);
    }
}

void Contact::SolveVelocityConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    // Solve tangential constraint first
    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        tangentSolvers[i].Solve(&normalSolvers[i]);
    }

    if (manifold.contactCount == 1 || block_solve == false || blockSolver.enabled == false)
    {
        for (int32 i = 0; i < manifold.contactCount; ++i)
        {
            normalSolvers[i].Solve();
        }
    }
    else
    {
        // Solve two contact constraints simultaneously (2-Contact LCP solver)
        blockSolver.Solve();
    }
}

bool Contact::SolvePositionConstraints(const Timestep& step)
{
    MuliNotUsed(step);

    bool solved = true;

    cLinearImpulseA.SetZero();
    cLinearImpulseB.SetZero();
    cAngularImpulseA = 0.0f;
    cAngularImpulseB = 0.0f;

    // Solve position constraint
    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        solved &= positionSolvers[i].Solve();
    }

    b1->sweep.c += b1->invMass * cLinearImpulseA;
    b1->sweep.a += b1->invInertia * cAngularImpulseA;
    b2->sweep.c += b2->invMass * cLinearImpulseB;
    b2->sweep.a += b2->invInertia * cAngularImpulseB;

    return solved;
}

bool Contact::SolveTOIPositionConstraints()
{
    bool solved = true;

    cLinearImpulseA.SetZero();
    cLinearImpulseB.SetZero();
    cAngularImpulseA = 0.0f;
    cAngularImpulseB = 0.0f;

    // Solve position constraint
    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        solved &= positionSolvers[i].SolveTOI();
    }

    // Push the body only if it's involved in TOI contact
    // TOI index == 0 or 1
    if (b1->islandIndex < 2)
    {
        b1->sweep.c += b1->invMass * cLinearImpulseA;
        b1->sweep.a += b1->invInertia * cAngularImpulseA;
    }
    if (b2->islandIndex < 2)
    {
        b2->sweep.c += b2->invMass * cLinearImpulseB;
        b2->sweep.a += b2->invInertia * cAngularImpulseB;
    }

    return solved;
}

} // namespace muli
