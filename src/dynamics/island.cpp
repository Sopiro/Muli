#include "muli/island.h"

#define SOLVE_CONTACTS_BACKWARD 1
#define SOLVE_CONTACT_CONSTRAINT 1

namespace muli
{

Island::Island(World& _world, uint32 _bodyCapacity, uint32 _contactCapacity, uint32 _jointCapacity)
    : world{ _world }
    , bodyCapacity{ _bodyCapacity }
    , contactCapacity{ _contactCapacity }
    , jointCapacity{ _jointCapacity }
    , bodyCount{ 0 }
    , contactCount{ 0 }
    , jointCount{ 0 }
    , sleeping{ false }
{
    bodies = (RigidBody**)world.stackAllocator.Allocate(bodyCapacity * sizeof(RigidBody*));
    contacts = (Contact**)world.stackAllocator.Allocate(contactCapacity * sizeof(Contact*));
    joints = (Joint**)world.stackAllocator.Allocate(jointCapacity * sizeof(Joint*));
}

Island::~Island()
{
    world.stackAllocator.Free(joints);
    world.stackAllocator.Free(contacts);
    world.stackAllocator.Free(bodies);
}

void Island::Solve()
{
    bool awakeIsland = false;

    // Integrate forces, yield tentative velocities that possibly violate the constraint
    for (uint32 i = 0; i < bodyCount; ++i)
    {
        RigidBody* b = bodies[i];

        if (sleeping)
        {
            b->flag |= RigidBody::Flag::FlagSleeping;
        }
        else
        {
            b->flag &= ~RigidBody::Flag::FlagSleeping;
        }

        if (sleeping)
        {
            b->linearVelocity.SetZero();
            b->angularVelocity = 0.0f;
        }

        if (world.integrateForce)
        {
            // Force / mass * dt
            Vec2 linear_a = b->force * b->invMass * world.settings.DT;
            b->linearVelocity += linear_a;

            // Torque / inertia * dt
            float angular_a = b->torque * b->invInertia * world.settings.DT;
            b->angularVelocity += angular_a;

            bool linearAwake = Length2(linear_a) >= world.settings.REST_LINEAR_TOLERANCE;
            bool angularAwake = angular_a * angular_a >= world.settings.REST_ANGULAR_TOLERANCE;

            if (sleeping && (linearAwake || angularAwake))
            {
                sleeping = false;
                awakeIsland = true;
            }
        }

        bool linearSleep = Length2(b->linearVelocity) < world.settings.REST_LINEAR_TOLERANCE;
        bool angularSleep = b->angularVelocity * b->angularVelocity < world.settings.REST_ANGULAR_TOLERANCE;

        if ((sleeping && !world.integrateForce) || (linearSleep && angularSleep))
        {
            b->resting += world.settings.DT;
        }
        else
        {
            sleeping = false;
            awakeIsland = true;
        }

        // Apply gravity
        if (world.settings.APPLY_GRAVITY && !sleeping && b->GetType() == RigidBody::Type::Dynamic)
        {
            b->linearVelocity += world.settings.GRAVITY * world.settings.DT;
        }
    }

    // If island is sleeping, skip the extra computation
    if (sleeping)
    {
        return;
    }

    // Prepare constraints for solving step
    for (uint32 i = 0; i < contactCount; ++i)
    {
        contacts[i]->Prepare();
    }
    for (uint32 i = 0; i < jointCount; ++i)
    {
        joints[i]->Prepare();
    }

    // Iteratively solve the violated velocity constraint
    // Solving contacts backward converge fast
    for (uint32 i = 0; i < world.settings.VELOCITY_SOLVE_ITERATIONS; ++i)
    {
#if SOLVE_CONTACTS_BACKWARD
#if SOLVE_CONTACT_CONSTRAINT
        for (uint32 j = contactCount; j > 0; j--)
        {
            contacts[j - 1]->SolveVelocityConstraint();
        }
#endif
        for (uint32 j = jointCount; j > 0; j--)
        {
            joints[j - 1]->SolveVelocityConstraint();
        }
#else
#if SOLVE_CONTACT_CONSTRAINT
        for (uint32 j = 0; j < contactCount; ++j)
        {
            contacts[j]->SolveVelocityConstraint();
        }
#endif
        for (uint32 j = 0; j < jointCount; ++j)
        {
            joints[j]->SolveVelocityConstraint();
        }
#endif
    }

    // Update positions using corrected velocities (Semi-implicit euler integration)
    for (uint32 i = 0; i < bodyCount; ++i)
    {
        RigidBody* b = bodies[i];

        if (awakeIsland)
        {
            b->Awake();
        }

        b->force.SetZero();
        b->torque = 0.0f;

        b->transform.position += b->linearVelocity * world.settings.DT;
        b->transform.rotation += b->angularVelocity * world.settings.DT;

        if (!TestPointInsideAABB(world.settings.VALID_REGION, b->GetPosition()))
        {
            world.BufferDestroy(b);
        }
    }

    for (uint32 i = 0; i < world.settings.POSITION_SOLVE_ITERATIONS; ++i)
    {
        bool contactSolved = true;
        bool jointSolved = true;

#if SOLVE_CONTACTS_BACKWARD
#if SOLVE_CONTACT_CONSTRAINT
        for (uint32 j = contactCount; j > 0; j--)
        {
            contactSolved &= contacts[j - 1]->SolvePositionConstraint();
        }
#endif
        for (uint32 j = jointCount; j > 0; j--)
        {
            jointSolved &= joints[j - 1]->SolvePositionConstraint();
        }
#else
#if SOLVE_CONTACT_CONSTRAINT
        for (uint32 j = 0; j < contactCount; ++j)
        {
            contactSolved &= contacts[j]->SolvePositionConstraint();
        }
#endif
        for (uint32 j = 0; j < jointCount; ++j)
        {
            jointSolved &= joints[j]->SolvePositionConstraint();
        }
#endif
        if (contactSolved && jointSolved)
        {
            break;
        }
    }
}

} // namespace muli