#include "muli/broad_phase.h"
#include "muli/contact_manager.h"
#include "muli/util.h"
#include "muli/world.h"

namespace muli
{

BroadPhase::BroadPhase(World* _world, ContactManager* _contactManager, float _aabbMargin, float _velocityMultiplier)
    : world{ _world }
    , contactManager{ _contactManager }
    , aabbMargin{ _aabbMargin }
    , velocityMultiplier{ _velocityMultiplier }
{
}

BroadPhase::~BroadPhase() noexcept
{
    Reset();
}

void BroadPhase::UpdateDynamicTree(float dt)
{
    for (RigidBody* body = world->bodyList; body; body = body->next)
    {
        // Clear island flag
        body->flag &= ~RigidBody::Flag::flag_island;

        if (body->IsSleeping())
        {
            continue;
        }
        if (body->type == RigidBody::Type::static_body)
        {
            body->flag |= RigidBody::Flag::flag_sleeping;
        }

        for (Collider* collider = body->colliderList; collider; collider = collider->next)
        {
            int32 node = collider->node;
            AABB treeAABB = tree.nodes[node].aabb;
            AABB aabb = collider->GetAABB();

            if (ContainsAABB(treeAABB, aabb) && body->resting < world->settings.SLEEPING_TRESHOLD)
            {
                continue;
            }

            Vec2 d = body->linearVelocity * dt * velocityMultiplier;

            if (d.x > 0.0f)
            {
                aabb.max.x += d.x;
            }
            else
            {
                aabb.min.x += d.x;
            }

            if (d.y > 0.0f)
            {
                aabb.max.y += d.y;
            }
            else
            {
                aabb.min.y += d.y;
            }

            aabb.max += aabbMargin;
            aabb.min -= aabbMargin;

            tree.Remove(collider);
            tree.Insert(collider, aabb);
        }
    }
}

void BroadPhase::FindContacts()
{
    for (RigidBody* body = world->bodyList; body; body = body->next)
    {
        for (Collider* collider = body->colliderList; collider; collider = collider->next)
        {
            bodyA = body;
            colliderA = collider;
            typeA = collider->GetType();

            // This will callback our BroadPhase::QueryCallback(Collider*)
            tree.Query(tree.nodes[colliderA->node].aabb, this);
        }
    }
}

bool BroadPhase::QueryCallback(Collider* colliderB)
{
    RigidBody* bodyB = colliderB->body;

    if (bodyA == bodyB)
    {
        return true;
    }

    Shape::Type typeB = colliderB->GetType();
    if (typeA < typeB)
    {
        return true;
    }
    else if (typeA == typeB)
    {
        if (colliderA > colliderB)
        {
            return true;
        }
    }

    contactManager->OnNewContact(colliderA, colliderB);

    return true;
}

} // namespace muli