#include "muli/broad_phase.h"
#include "muli/util.h"
#include "muli/world.h"

namespace muli
{

void BroadPhase::UpdateDynamicTree(float dt)
{
    for (RigidBody* body = world.bodyList; body; body = body->next)
    {
        // Clear island flag
        body->flag &= ~RigidBody::Flag::FlagIsland;

        if (body->IsSleeping())
        {
            continue;
        }
        if (body->type == RigidBody::Type::Static)
        {
            body->flag |= RigidBody::Flag::FlagSleeping;
        }

        int32 node = body->node;
        AABB treeAABB = tree.nodes[node].aabb;
        AABB aabb = body->GetAABB();

        if (ContainsAABB(treeAABB, aabb) && body->resting < world.settings.SLEEPING_TRESHOLD)
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

        tree.Remove(body);
        tree.Insert(body, aabb);
    }
}

void BroadPhase::FindContacts(std::function<void(RigidBody* bodyA, RigidBody* bodyB)> callback) const
{
    for (RigidBody* bodyA = world.bodyList; bodyA; bodyA = bodyA->next)
    {
        tree.Query(tree.nodes[bodyA->node].aabb, [&](RigidBody* bodyB) -> bool {
            if (bodyA == bodyB)
            {
                return true;
            }

            if (bodyA->shape < bodyB->shape)
            {
                return true;
            }
            else if (bodyA->shape == bodyB->shape)
            {
                if (bodyA->id > bodyB->id)
                {
                    return true;
                }
            }

            callback(bodyA, bodyB);

            return true;
        });
    }
}

} // namespace muli