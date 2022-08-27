#include "spe/broad_phase.h"
#include "spe/util.h"
#include "spe/world.h"

namespace spe
{

BroadPhase::BroadPhase(World& _world) :
    world{ _world }
{
}

void BroadPhase::Update(float dt)
{
    for (uint32_t i = 0; i < world.bodies.size(); i++)
    {
        RigidBody* body = world.bodies[i];
        body->contactConstraintIDs.clear();

        if (body->sleeping) continue;
        if (body->type == BodyType::Static) body->sleeping = true;

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

        // Remove the old, potentially false pair
        tree.Query(treeAABB,
            [&](const Node* n) -> bool
            {
                PairID pairID = combine_id(body->GetID(), n->body->GetID());
                pairs.erase(pairID.key);

                return true;
            });

        // Insert new pair
        tree.Query(aabb,
            [&](const Node* n) -> bool
            {
                assert(body != n->body);

                if (body->GetType() == BodyType::Static && n->body->GetType() == BodyType::Static)
                    return true;

                PairID pairID = combine_id(body->GetID(), n->body->GetID());
                pairs.insert(pairID.key);

                return true;
            });

        tree.Insert(body, aabb);
    }
}

void BroadPhase::Reset()
{
    pairs.clear();
    tree.Reset();
}

void BroadPhase::Add(RigidBody* body)
{
    AABB fatAABB = body->GetAABB();
    fatAABB.min -= margin;
    fatAABB.max += margin;

    tree.Insert(body, fatAABB);
}

void BroadPhase::Remove(RigidBody* body)
{
    tree.Query(body->node->aabb,
        [&](const Node* n) -> bool
        {
            if (body->GetID() == n->body->GetID() || (body->GetType() == BodyType::Static && n->body->GetType() == BodyType::Static))
                return true;

            PairID pairID = combine_id(body->GetID(), n->body->GetID());
            pairs.erase(pairID.key);

            return true;
        });

    tree.Remove(body);
}

}