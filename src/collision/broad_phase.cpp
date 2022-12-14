#include "muli/broad_phase.h"
#include "muli/contact_manager.h"
#include "muli/util.h"
#include "muli/world.h"

namespace muli
{

BroadPhase::BroadPhase(World* _world, ContactManager* _contactManager)
    : world{ _world }
    , contactManager{ _contactManager }
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

void BroadPhase::Add(Collider* collider, const AABB& aabb)
{
    int32 treeNode = tree.CreateNode(collider, aabb);
    collider->node = treeNode;

    BufferMove(collider);
}

void BroadPhase::Remove(Collider* collider)
{
    tree.RemoveNode(collider->node);

    UnBufferMove(collider);
}

void BroadPhase::Update(Collider* collider, const AABB& aabb, const Vec2& displacement)
{
    bool moved = tree.MoveNode(collider->node, aabb, displacement, collider->body->IsSleeping());
    if (moved)
    {
        BufferMove(collider);
    }
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

        const AABB& treeAABB = tree.GetAABB(colliderA->node);

        // This will callback our BroadPhase::QueryCallback(Collider*)
        tree.Query(treeAABB, this);
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