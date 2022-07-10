#include "island.h"

using namespace spe;

Island::Island(World& _world) :
    world{ _world }
{

}

void Island::Solve(float dt)
{
    bool awakeIsland = false;

    // Integrate forces, yield tentative velocities that possibly violate the constraint
    for (size_t i = 0; i < bodies.size(); i++)
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

            if (sleeping &&
                (glm::length2(linear_a) >= world.settings.REST_LINEAR_TOLERANCE) ||
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
            b->resting += dt;
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
        for (size_t i = 0; i < ccs.size(); i++)
            ccs[i]->Prepare();
    }

    // Iteratively solve the violated velocity constraint
    {
        for (size_t i = 0; i < world.settings.SOLVE_ITERATION; i++)
        {
            for (size_t j = 0; j < ccs.size(); j++)
                ccs[j]->Solve();
        }
    }

    // Update positions using corrected velocities (Semi-implicit euler integration)
    for (size_t i = 0; i < bodies.size(); i++)
    {
        RigidBody* b = bodies[i];

        if (awakeIsland) b->Awake();

        glm::clear(b->force);
        b->torque = 0.0f;

        b->position += b->linearVelocity * world.settings.DT;
        b->rotation += b->angularVelocity * world.settings.DT;
    }
}

void Island::Clear()
{
    bodies.clear();
    ccs.clear();
    sleeping = false;
}
