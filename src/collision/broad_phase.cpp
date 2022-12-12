#include "muli/broad_phase.h"
#include "muli/contact_manager.h"
#include "muli/util.h"
#include "muli/world.h"

namespace muli
{

BroadPhase::BroadPhase(World* _world, ContactManager* _contactManager, float _aabbMargin, float _aabbMultiplier)
    : world{ _world }
    , contactManager{ _contactManager }
    , aabbMargin{ _aabbMargin }
    , aabbMultiplier{ _aabbMultiplier }
    , moveCapacity{ 16 }
    , moveCount{ 0 }
{
    moveBuffer = (Collider**)malloc(sizeof(Collider*) * moveCapacity);
}

BroadPhase::~BroadPhase()
{
    free(moveBuffer);
}

void BroadPhase::BufferMove(Collider* collider)
{
    if (moveCount == moveCapacity)
    {
        Collider** old = moveBuffer;
        moveCapacity *= 2;
        moveBuffer = (Collider**)malloc(moveCapacity * sizeof(Collider*));
        memcpy(moveBuffer, old, moveCount * sizeof(Collider*));
        free(old);
    }

    moveBuffer[moveCount] = collider;
    ++moveCount;

    collider->moved = true;
}

void BroadPhase::UnBufferMove(Collider* collider)
{
    for (int32 i = 0; i < moveCount; ++i)
    {
        if (moveBuffer[i] == collider)
        {
            moveBuffer[i] = nullptr;
        }
    }
}

void BroadPhase::Add(Collider* collider, AABB aabb)
{
    // Fatten the aabb
    aabb.min -= aabbMargin;
    aabb.max += aabbMargin;

    int32 index = tree.Insert(collider, aabb);
    BufferMove(collider);
}

void BroadPhase::Remove(Collider* collider)
{
    tree.Remove(collider);
    UnBufferMove(collider);
}

void BroadPhase::Update(Collider* collider, AABB aabb, const Vec2& displacement)
{
    int32 node = collider->node;
    AABB treeAABB = tree.nodes[node].aabb;

    RigidBody* body = collider->body;

    bool awake = body->resting < world->settings.sleeping_treshold;

    if (ContainsAABB(treeAABB, aabb) && awake)
    {
        return;
    }

    Vec2 d = displacement * aabbMultiplier;

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

    BufferMove(collider);
}

void BroadPhase::FindContacts()
{
    for (int32 i = 0; i < moveCount; ++i)
    {
        colliderA = moveBuffer[i];

        if (colliderA == nullptr)
        {
            continue;
        }

        bodyA = colliderA->body;
        typeA = colliderA->GetType();

        // This will callback our BroadPhase::QueryCallback(Collider*)
        tree.Query(tree.nodes[colliderA->node].aabb, this);
    }

    for (int i = 0; i < moveCount; ++i)
    {
        if (moveBuffer[i])
        {
            moveBuffer[i]->moved = false;
        }
    }

    moveCount = 0;
}

bool BroadPhase::QueryCallback(Collider* colliderB)
{
    if (colliderA == colliderB)
    {
        return true;
    }

    RigidBody* bodyB = colliderB->body;
    if (bodyA == bodyB)
    {
        return true;
    }

    // Avoid duplicate contact
    if (colliderB->moved && colliderA < colliderB)
    {
        return true;
    }

    Shape::Type typeB = colliderB->GetType();
    if (typeA < typeB)
    {
        contactManager->OnNewContact(colliderB, colliderA);
    }
    else
    {
        contactManager->OnNewContact(colliderA, colliderB);
    }

    return true;
}

} // namespace muli