#include "spe/island.h"

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
            glm::clear(b->linearVelocity);
            b->angularVelocity = 0.0f;
        }

        if (world.forceIntegration)
        {
            // Force / mass * dt
            glm::vec2 linear_a = b->force * b->invMass * world.settings.DT;
            b->linearVelocity += linear_a;

            // Torque / inertia * dt
            float angular_a = b->torque * b->invInertia * world.settings.DT;
            b->angularVelocity += angular_a;

            if (sleeping && (glm::length2(linear_a) >= world.settings.REST_LINEAR_TOLERANCE) ||
                (angular_a * angular_a >= world.settings.REST_ANGULAR_TOLERANCE))
            {
                sleeping = false;
                awakeIsland = true;
            }
        }

        if ((sleeping && !world.forceIntegration) ||
            ((glm::length2(b->linearVelocity) < world.settings.REST_LINEAR_TOLERANCE) &&
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

    // Prepare for solving
    {
        for (uint32_t i = 0; i < contacts.size(); i++)
            contacts[i]->Prepare();
        for (uint32_t i = 0; i < joints.size(); i++)
            joints[i]->Prepare();
    }

    // Iteratively solve the violated velocity constraint
    {
        for (uint32_t i = 0; i < world.settings.SOLVE_ITERATION; i++)
        {
            for (uint32_t j = 0; j < contacts.size(); j++)
                contacts[j]->Solve();
            for (uint32_t j = 0; j < joints.size(); j++)
                joints[j]->Solve();
        }
    }

    // Update positions using corrected velocities (Semi-implicit euler integration)
    for (uint32_t i = 0; i < bodies.size(); i++)
    {
        RigidBody* b = bodies[i];

        if (awakeIsland) b->Awake();

        glm::clear(b->force);
        b->torque = 0.0f;

        b->position += b->linearVelocity * world.settings.DT;
        b->rotation += b->angularVelocity * world.settings.DT;

        if (!test_point_inside_AABB(world.settings.VALID_REGION, b->position))
        {
            world.BufferDestroy(b);
        }
    }
}

} // namespace spe