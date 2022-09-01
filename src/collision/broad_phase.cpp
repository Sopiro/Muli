#include "spe/broad_phase.h"
#include "spe/util.h"
#include "spe/world.h"

namespace spe
{

void BroadPhase::UpdateDynamicTree(float dt)
{
    for (RigidBody* body = world.bodyList; body; body = body->next)
    {
        if (body->sleeping) continue;
        if (body->type == RigidBody::Type::Static) body->sleeping = true;

        Node* node = body->node;
        AABB treeAABB = node->aabb;
        AABB aabb = body->GetAABB();

        if (contains_AABB(treeAABB, aabb))
        {
            continue;
        }

        glm::vec2 d = body->linearVelocity * dt * velocityMultiplier;
        if (d.x > 0.0f)
            aabb.max.x += d.x;
        else
            aabb.min.x += d.x;
        if (d.y > 0.0f)
            aabb.max.y += d.y;
        else
            aabb.min.y += d.y;
        aabb.max += margin;
        aabb.min -= margin;

        tree.Remove(body);
        tree.Insert(body, aabb);
    }
}

void BroadPhase::FindContacts(std::function<void(RigidBody* bodyA, RigidBody* bodyB)> callback)
{
    for (RigidBody* bodyA = world.bodyList; bodyA; bodyA = bodyA->next)
    {
        tree.Query(bodyA->node->aabb, [&](const Node* n) -> bool {
            RigidBody* bodyB = n->body;

            if (bodyA == bodyB || (bodyA->id > bodyB->id))
            {
                return true;
            }

            callback(bodyA, bodyB);

            return true;
        });
    }
}

} // namespace spe