#include "muli/island.h"

#define SOLVE_CONTACTS_BACKWARD 1
#define SOLVE_CONTACT_CONSTRAINT 1

namespace muli
{

Island::Island(World* _world, int32 _bodyCapacity, int32 _contactCapacity, int32 _jointCapacity)
    : world{ _world }
    , bodyCapacity{ _bodyCapacity }
    , contactCapacity{ _contactCapacity }
    , jointCapacity{ _jointCapacity }
    , bodyCount{ 0 }
    , contactCount{ 0 }
    , jointCount{ 0 }
    , sleeping{ false }
{
    bodies = (RigidBody**)world->stackAllocator.Allocate(bodyCapacity * sizeof(RigidBody*));
    contacts = (Contact**)world->stackAllocator.Allocate(contactCapacity * sizeof(Contact*));
    joints = (Joint**)world->stackAllocator.Allocate(jointCapacity * sizeof(Joint*));
}

Island::~Island()
{
    world->stackAllocator.Free(joints, jointCapacity * sizeof(Joint*));
    world->stackAllocator.Free(contacts, contactCapacity * sizeof(Contact*));
    world->stackAllocator.Free(bodies, bodyCapacity * sizeof(RigidBody*));
}

void Island::Solve()
{
    bool awakeIsland = false;
    float dt = world->settings.dt;

    // Integrate velocities, yield tentative velocities that possibly violate the constraint
    for (int32 i = 0; i < bodyCount; ++i)
    {
        RigidBody* b = bodies[i];

        // Save positions for continuous collision
        b->sweep.c0 = b->sweep.c;
        b->sweep.a0 = b->sweep.a;

        // All bodies on this island are resting more than sleep time, but flags are not set
        if (sleeping)
        {
            b->islandID = 0;
            b->islandIndex = 0;
            b->linearVelocity.SetZero();
            b->angularVelocity = 0.0f;
            b->flag |= RigidBody::flag_sleeping;
        }
        else
        {
            b->flag &= ~RigidBody::flag_sleeping;
        }

        if (((b->angularVelocity * b->angularVelocity < world->settings.reset_angular_tolerance) &&
             (Dot(b->linearVelocity, b->linearVelocity) < world->settings.rest_linear_tolerance)) &&
            (b->torque * b->torque == 0 && Dot(b->force, b->force) == 0))
        {
            b->resting += dt;
        }
        else
        {
            muliAssert(sleeping == false);
            awakeIsland = true;
        }

        if (b->type == RigidBody::Type::dynamic_body)
        {
            // Integrate velocites
            b->linearVelocity += b->invMass * dt * (b->force + world->settings.apply_gravity * world->settings.gravity * b->mass);
            b->angularVelocity += b->invInertia * dt * b->torque;

            /*
               Apply damping (found in box2d)
               ODE: dv/dt + c * v = 0
               dv/dt = -c * v
               (1/v) dv = -c dt ; integrate both sides
               ln|v| = -c * t + C ; exponentiate both sides (C is integration constant)
               v = C * exp(-c * t)
               v(0) = C
               Solution: v(t) = v0 * exp(-c * t)
               Time step: v(t + dt) = v0 * exp(-c * (t + dt))
                                    = v0 * exp(-c * t) * exp(-c * dt)
                                    = v * exp(-c * dt)
               v2 = exp(-c * dt) * v1
               Pade approximation:
               v2 = v1 * 1 / (1 + c * dt)
            */
            b->linearVelocity *= 1.0f / (1.0f + b->linearDamping * dt);
            b->angularVelocity *= 1.0f / (1.0f + b->angularDamping * dt);
        }
    }

    // Prepare constraints for solving step
    for (int32 i = 0; i < contactCount; ++i)
    {
        contacts[i]->Prepare();
    }
    for (int32 i = 0; i < jointCount; ++i)
    {
        joints[i]->Prepare();
    }

    // Iteratively solve the violated velocity constraint
    // Solving contacts backward converge fast
    for (int32 i = 0; i < world->settings.velocity_iterations; ++i)
    {
#if SOLVE_CONTACTS_BACKWARD
#if SOLVE_CONTACT_CONSTRAINT
        for (int32 j = contactCount; j > 0; j--)
        {
            contacts[j - 1]->SolveVelocityConstraints();
        }
#endif
        for (int32 j = jointCount; j > 0; j--)
        {
            joints[j - 1]->SolveVelocityConstraints();
        }
#else
#if SOLVE_CONTACT_CONSTRAINT
        for (int32 j = 0; j < contactCount; ++j)
        {
            contacts[j]->SolveVelocityConstraints();
        }
#endif
        for (int32 j = 0; j < jointCount; ++j)
        {
            joints[j]->SolveVelocityConstraints();
        }
#endif
    }

    // Update positions using corrected velocities (Semi-implicit euler integration)
    for (int32 i = 0; i < bodyCount; ++i)
    {
        RigidBody* b = bodies[i];

        if (awakeIsland)
        {
            b->Awake();
        }

        b->force.SetZero();
        b->torque = 0.0f;

        b->sweep.c += b->linearVelocity * world->settings.dt;
        b->sweep.a += b->angularVelocity * world->settings.dt;

        if (world->settings.world_bounds.TestPoint(b->GetPosition()) == false)
        {
            world->BufferDestroy(b);
        }
    }

    for (int32 i = 0; i < world->settings.position_iterations; ++i)
    {
        bool contactSolved = true;
        bool jointSolved = true;

#if SOLVE_CONTACTS_BACKWARD
#if SOLVE_CONTACT_CONSTRAINT
        for (int32 j = contactCount; j > 0; j--)
        {
            contactSolved &= contacts[j - 1]->SolvePositionConstraints();
        }
#endif
        for (int32 j = jointCount; j > 0; j--)
        {
            jointSolved &= joints[j - 1]->SolvePositionConstraints();
        }
#else
#if SOLVE_CONTACT_CONSTRAINT
        for (int32 j = 0; j < contactCount; ++j)
        {
            contactSolved &= contacts[j]->SolvePositionConstraints();
        }
#endif
        for (int32 j = 0; j < jointCount; ++j)
        {
            jointSolved &= joints[j]->SolvePositionConstraints();
        }
#endif
        if (contactSolved && jointSolved)
        {
            break;
        }
    }
}

constexpr int32 toi_postion_iteration = 20;
constexpr int32 toi_index_1 = 0;
constexpr int32 toi_index_2 = 1;

void Island::SolveTOI(float dt)
{
    bool warmStartingEnabled = world->settings.warm_starting;
    world->settings.warm_starting = false;

    for (int32 i = 0; i < contactCount; ++i)
    {
        // Save the impulses computed by the discrete solver
        contacts[i]->SaveImpulses();
        contacts[i]->Prepare();
    }

    // Move the TOI contact to a safe position so that the next ComputeTimeOfImpact() returns the separated state
    for (int32 i = 0; i < toi_postion_iteration; ++i)
    {
        bool solved = true;

        for (int32 j = 0; j < contactCount; ++j)
        {
            solved &= contacts[j]->SolveTOIPositionConstraints();
        }

        if (solved)
        {
            break;
        }
    }

    bodies[toi_index_1]->sweep.c0 = bodies[toi_index_1]->sweep.c;
    bodies[toi_index_1]->sweep.a0 = bodies[toi_index_1]->sweep.a;
    bodies[toi_index_2]->sweep.c0 = bodies[toi_index_2]->sweep.c;
    bodies[toi_index_2]->sweep.a0 = bodies[toi_index_2]->sweep.a;

    for (int32 i = 0; i < world->settings.velocity_iterations; ++i)
    {
#if SOLVE_CONTACTS_BACKWARD
#if SOLVE_CONTACT_CONSTRAINT
        for (int32 j = contactCount; j > 0; j--)
        {
            contacts[j - 1]->SolveVelocityConstraints();
        }
#endif
#else
#if SOLVE_CONTACT_CONSTRAINT
        for (int32 j = 0; j < contactCount; ++j)
        {
            contacts[j]->SolveVelocityConstraints();
        }
#endif
#endif
    }

    // We don't need position correction
    // Because we solved velocity constraints in a position that is already safe

    for (int32 i = 0; i < contactCount; ++i)
    {
        // Restore the impulses of the discrete solver
        contacts[i]->RestoreImpulses();
    }

    for (int32 i = 0; i < bodyCount; ++i)
    {
        RigidBody* b = bodies[i];

        b->sweep.c += b->linearVelocity * dt;
        b->sweep.a += b->angularVelocity * dt;
        b->SynchronizeTransform();
    }

    world->settings.warm_starting = warmStartingEnabled;
}

} // namespace muli