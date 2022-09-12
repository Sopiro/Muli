#include "spe/island.h"

#define SOLVE_CONTACTS_BACKWARD 1

namespace spe
{

void Island::Solve()
{
    bool awakeIsland = false;

    // Integrate forces, yield tentative velocities that possibly violate the constraint
    for (uint32_t i = 0; i < bodies.size(); i++)
    {
        RigidBody* b = bodies[i];

        b->sleeping = sleeping;

        if (sleeping)
        {
            b->linearVelocity.SetZero();
            b->angularVelocity = 0.0f;
        }

        if (world.forceIntegration)
        {
            // Force / mass * dt
            Vec2 linear_a = b->force * b->invMass * world.settings.DT;
            b->linearVelocity += linear_a;

            // Torque / inertia * dt
            float angular_a = b->torque * b->invInertia * world.settings.DT;
            b->angularVelocity += angular_a;

            if (sleeping && (Length2(linear_a) >= world.settings.REST_LINEAR_TOLERANCE) ||
                (angular_a * angular_a >= world.settings.REST_ANGULAR_TOLERANCE))
            {
                sleeping = false;
                awakeIsland = true;
            }
        }

        if ((sleeping && !world.forceIntegration) ||
            ((Length2(b->linearVelocity) < world.settings.REST_LINEAR_TOLERANCE) &&
             (b->angularVelocity * b->angularVelocity < world.settings.REST_ANGULAR_TOLERANCE)))
        {
            b->resting += world.settings.DT;
        }
        else
        {
            sleeping = false;
            awakeIsland = true;
        }

        // Apply gravity
        if (world.settings.APPLY_GRAVITY && !sleeping)
        {
            b->linearVelocity += world.settings.GRAVITY * world.settings.DT;
        }
    }

    // If island is sleeping, skip the extra computation
    if (sleeping) return;

    // Prepare constraints for solving step
    for (uint32_t i = 0; i < contacts.size(); i++)
    {
        contacts[i]->Prepare();
    }
    for (uint32_t i = 0; i < joints.size(); i++)
    {
        joints[i]->Prepare();
    }

    // Iteratively solve the violated velocity constraint
    // Solving contacts backward converge fast
    for (uint32_t i = 0; i < world.settings.VELOCITY_SOLVE_ITERATIONS; i++)
    {
#if SOLVE_CONTACTS_BACKWARD
        for (size_t j = contacts.size(); j > 0; j--)
        {
            contacts[j - 1]->SolveVelocityConstraint();
        }
        for (size_t j = joints.size(); j > 0; j--)
        {
            joints[j - 1]->SolveVelocityConstraint();
        }
#else
        for (uint32_t j = 0; j < contacts.size(); j++)
        {
            contacts[j]->SolveVelocityConstraint();
        }
        for (uint32_t j = 0; j < joints.size(); j++)
        {
            joints[j]->SolveVelocityConstraint();
        }
#endif
    }

    // Update positions using corrected velocities (Semi-implicit euler integration)
    for (uint32_t i = 0; i < bodies.size(); i++)
    {
        RigidBody* b = bodies[i];

        if (awakeIsland) b->Awake();

        b->force.SetZero();
        b->torque = 0.0f;

        b->transform.position += b->linearVelocity * world.settings.DT;
        b->transform.rotation += b->angularVelocity * world.settings.DT;

        if (!TestPointInsideAABB(world.settings.VALID_REGION, b->GetPosition()))
        {
            world.BufferDestroy(b);
        }
    }

    if (world.settings.POSITION_CORRECTION)
    {
        for (uint32_t i = 0; i < world.settings.POSITION_SOLVE_ITERATIONS; i++)
        {
            bool contactSolved = true;
            bool jointSolved = true;

#if SOLVE_CONTACTS_BACKWARD
            for (size_t j = contacts.size(); j > 0; j--)
            {
                contactSolved &= contacts[j - 1]->SolvePositionConstraint();
            }
            for (size_t j = joints.size(); j > 0; j--)
            {
                jointSolved &= joints[j - 1]->SolvePositionConstraint();
            }
#else
            for (size_t j = 0; j < contacts.size(); j++)
            {
                contactSolved &= contacts[j]->SolvePositionConstraint();
            }
            for (size_t j = 0; j < joints.size(); j++)
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
}

} // namespace spe