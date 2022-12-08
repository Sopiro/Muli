#include "muli/contact.h"
#include "muli/block_solver.h"
#include "muli/callbacks.h"
#include "muli/contact_solver.h"
#include "muli/world.h"

namespace muli
{

extern DetectionFunction* DetectionFunctionMap[Shape::Type::shape_count][Shape::Type::shape_count];

Contact::Contact(Collider* _colliderA, Collider* _colliderB, const WorldSettings& _settings)
    : Constraint(_colliderA->body, _colliderB->body, _settings)
    , colliderA{ _colliderA }
    , colliderB{ _colliderB }
    , touching{ false }
{
    muliAssert(colliderA->GetType() >= colliderB->GetType());

    manifold.numContacts = 0;

    beta = settings.position_correction;
    restitution = MixRestitution(colliderA->GetRestitution(), colliderB->GetRestitution());
    friction = MixFriction(colliderA->GetFriction(), colliderB->GetFriction());
    surfaceSpeed = colliderB->GetSurfaceSpeed() - colliderA->GetSurfaceSpeed();

    collisionDetectionFunction = DetectionFunctionMap[colliderA->GetType()][colliderB->GetType()];
    muliAssert(collisionDetectionFunction != nullptr);
}

void Contact::Update()
{
    ContactManifold oldManifold = manifold;
    float oldNormalImpulse[MAX_CONTACT_POINT];
    float oldTangentImpulse[MAX_CONTACT_POINT];

    bool wasTouching = touching;

    // clang-format off
    touching = collisionDetectionFunction(colliderA->shape, bodyA->transform,
                                          colliderB->shape, bodyB->transform,
                                          &manifold);
    // clang-format on

    for (int32 i = 0; i < MAX_CONTACT_POINT; ++i)
    {
        oldNormalImpulse[i] = normalSolvers[i].impulseSum;
        oldTangentImpulse[i] = tangentSolvers[i].impulseSum;
        normalSolvers[i].impulseSum = 0.0f;
        tangentSolvers[i].impulseSum = 0.0f;
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

    if (wasTouching == false && touching == true)
    {
        if (colliderA->ContactListener) colliderA->ContactListener->OnContactBegin(colliderA, colliderB, this);
        if (colliderB->ContactListener) colliderB->ContactListener->OnContactBegin(colliderB, colliderA, this);
    }

    if (wasTouching == true && touching == true)
    {
        if (colliderA->ContactListener) colliderA->ContactListener->OnContactTouching(colliderA, colliderB, this);
        if (colliderB->ContactListener) colliderB->ContactListener->OnContactTouching(colliderB, colliderA, this);
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

    // Warm start the contact solver
    for (int32 n = 0; n < manifold.numContacts; ++n)
    {
        int32 o = 0;
        for (; o < oldManifold.numContacts; ++o)
        {
            if (manifold.contactPoints[n].id == oldManifold.contactPoints[o].id)
            {
                break;
            }
        }

        if (o < oldManifold.numContacts)
        {
            normalSolvers[n].impulseSum = oldNormalImpulse[o];
            tangentSolvers[n].impulseSum = oldTangentImpulse[o];
        }
    }
}

void Contact::Prepare()
{
    for (int32 i = 0; i < manifold.numContacts; ++i)
    {
        normalSolvers[i].Prepare(this, i, manifold.contactNormal, ContactSolver::Type::normal);
        tangentSolvers[i].Prepare(this, i, manifold.contactTangent, ContactSolver::Type::tangent);
        positionSolvers[i].Prepare(this, i);
    }

    if (manifold.numContacts == 2 && settings.block_solve)
    {
        blockSolver.Prepare(this);
    }
}

void Contact::SolveVelocityConstraint()
{
    // Solve tangential constraint first
    for (int32 i = 0; i < manifold.numContacts; ++i)
    {
        tangentSolvers[i].Solve(&normalSolvers[i]);
    }

    if (manifold.numContacts == 1 || !settings.block_solve || (blockSolver.enabled == false))
    {
        for (int32 i = 0; i < manifold.numContacts; ++i)
        {
            normalSolvers[i].Solve();
        }
    }
    else
    {
        // Solve two contact constraint simultaneously (2-Contact LCP solver)
        blockSolver.Solve();
    }
}

bool Contact::SolvePositionConstraint()
{
    bool solved = true;

    cLinearImpulseA.SetZero();
    cLinearImpulseB.SetZero();
    cAngularImpulseA = 0.0f;
    cAngularImpulseB = 0.0f;

    // Solve position constraint
    for (int32 i = 0; i < manifold.numContacts; ++i)
    {
        solved &= positionSolvers[i].Solve();
    }

    b1->sweep.c += b1->invMass * cLinearImpulseA;
    b1->sweep.a += b1->invInertia * cAngularImpulseA;
    b2->sweep.c += b2->invMass * cLinearImpulseB;
    b2->sweep.a += b2->invInertia * cAngularImpulseB;

    return solved;
}

} // namespace muli
