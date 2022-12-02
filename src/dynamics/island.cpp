#include "muli/island.h"

#define SOLVE_CONTACTS_BACKWARD 1
#define SOLVE_CONTACT_CONSTRAINT 1

namespace muli
{

Island::Island(World* _world, uint32 _bodyCapacity, uint32 _contactCapacity, uint32 _jointCapacity)
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
    world->stackAllocator.Free(joints, jointCapacity);
    world->stackAllocator.Free(contacts, contactCapacity);
    world->stackAllocator.Free(bodies, bodyCapacity);
}

void Island::Solve()
{
    bool awakeIsland = false;
    float dt = world->settings.DT;

    // Integrate velocities, yield tentative velocities that possibly violate the constraint
    for (uint32 i = 0; i < bodyCount; ++i)
    {
        RigidBody* b = bodies[i];

        // All bodies on this island are resting more than sleep time, but flags are not set
        if (sleeping)
        {
            b->linearVelocity.SetZero();
            b->angularVelocity = 0.0f;
            b->flag |= RigidBody::Flag::flag_sleeping;
        }

        if (((b->angularVelocity * b->angularVelocity < world->settings.REST_ANGULAR_TOLERANCE) &&
             (Dot(b->linearVelocity, b->linearVelocity) < world->settings.REST_LINEAR_TOLERANCE)) &&
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
            b->linearVelocity += b->invMass * dt * (b->force + world->settings.APPLY_GRAVITY * world->settings.GRAVITY * b->mass);
            b->angularVelocity += b->invInertia * dt * b->torque;

            // Apply damping (found in box2d)
            // ODE: dv/dt + c * v = 0
            // dv/dt = -c * v
            // (1/v) dv = -c dt // integrate both sides
            // ln|v| = -c * t + C //  exponentiate both sides (C is integration constant)
            // v = C * exp(-c * t)
            // v(0) = C
            // Solution: v(t) = v0 * exp(-c * t)
            // Time step: v(t + dt) = v0 * exp(-c * (t + dt))
            //                      = v0 * exp(-c * t) * exp(-c * dt)
            //                      = v * exp(-c * dt)
            // v2 = exp(-c * dt) * v1
            // Pade approximation:
            // v2 = v1 * 1 / (1 + c * dt)
            b->linearVelocity *= 1.0f / (1.0f + b->linearDamping * dt);
            b->angularVelocity *= 1.0f / (1.0f + b->angularDamping * dt);
        }
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
    for (uint32 i = 0; i < world->settings.VELOCITY_SOLVE_ITERATIONS; ++i)
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

        b->position += b->linearVelocity * world->settings.DT;
        b->angle += b->angularVelocity * world->settings.DT;

        if (!TestPointInsideAABB(world->settings.VALID_REGION, b->GetPosition()))
        {
            world->BufferDestroy(b);
        }
    }

    for (uint32 i = 0; i < world->settings.POSITION_SOLVE_ITERATIONS; ++i)
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

    for (uint32 i = 0; i < bodyCount; ++i)
    {
        RigidBody* b = bodies[i];
        b->SynchronizeTransform();
    }
}

} // namespace muli