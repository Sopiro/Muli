#include "muli/broad_phase.h"
#include "muli/contact_graph.h"
#include "muli/world.h"

namespace muli
{

BroadPhase::BroadPhase(World* world, ContactGraph* contactManager)
    : world{ world }
    , contactManager{ contactManager }
    , moveCapacity{ 16 }
    , moveCount{ 0 }
{
    moveBuffer = (NodeProxy*)muli::Alloc(moveCapacity * sizeof(NodeProxy));
}

BroadPhase::~BroadPhase()
{
    muli::Free(moveBuffer);
}

void BroadPhase::BufferMove(NodeProxy node)
{
    // Grow the buffer as needed
    if (moveCount == moveCapacity)
    {
        NodeProxy* old = moveBuffer;
        moveCapacity *= 2;
        moveBuffer = (NodeProxy*)muli::Alloc(moveCapacity * sizeof(NodeProxy));
        memcpy(moveBuffer, old, moveCount * sizeof(NodeProxy));
        muli::Free(old);
    }

    moveBuffer[moveCount] = node;
    ++moveCount;
}

void BroadPhase::UnBufferMove(NodeProxy node)
{
    for (int32 i = 0; i < moveCount; ++i)
    {
        if (moveBuffer[i] == node)
        {
            moveBuffer[i] = AABBTree::nullNode;
        }
    }
}

void BroadPhase::FindNewContacts()
{
    for (int32 i = 0; i < moveCount; ++i)
    {
        nodeA = moveBuffer[i];
        if (nodeA == AABBTree::nullNode)
        {
            continue;
        }

        colliderA = tree.GetData(nodeA);
        bodyA = colliderA->body;
        typeA = colliderA->GetType();

        const AABB& treeAABB = tree.GetAABB(colliderA->node);

        // This will callback our BroadPhase::QueryCallback(NodeProxy, Collider*)
        tree.Query(treeAABB, this);
    }

    // Clear move buffer for next step
    for (int32 i = 0; i < moveCount; ++i)
    {
        NodeProxy node = moveBuffer[i];
        if (node != AABBTree::nullNode)
        {
            tree.ClearMoved(node);
        }
    }

    moveCount = 0;
}

void BroadPhase::Add(Collider* collider, const AABB& aabb)
{
    NodeProxy node = tree.CreateNode(collider, aabb);
    collider->node = node;

    BufferMove(node);
}

void BroadPhase::Remove(Collider* collider)
{
    NodeProxy node = collider->node;
    tree.RemoveNode(node);

    UnBufferMove(node);
}

void BroadPhase::Update(Collider* collider, const AABB& aabb, const Vec2& displacement)
{
    NodeProxy node = collider->node;
    bool rested = collider->body->resting > world->settings.sleeping_time;

    bool nodeMoved = tree.MoveNode(node, aabb, displacement, rested);
    if (nodeMoved)
    {
        BufferMove(node);
    }
}

void BroadPhase::Refresh(Collider* collider)
{
    NodeProxy node = collider->node;
    AABB aabb = collider->GetAABB();

    tree.MoveNode(node, aabb, Vec2::zero, true);
    BufferMove(node);
}

bool BroadPhase::QueryCallback(NodeProxy nodeB, Collider* colliderB)
{
    if (nodeA == nodeB)
    {
        return true;
    }

    RigidBody* bodyB = colliderB->body;
    if (bodyA == bodyB)
    {
        return true;
    }

    // Avoid duplicate contact
    if (tree.WasMoved(nodeB) && nodeA < nodeB)
    {
        return true;
    }

    Shape::Type typeB = colliderB->GetType();
    if (typeA <= typeB)
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