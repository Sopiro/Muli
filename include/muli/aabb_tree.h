#pragma once

#include "aabb.h"
#include "collider.h"
#include "collision.h"
#include "common.h"
#include "growable_array.h"
#include "settings.h"

#define nullNode (-1)

namespace muli
{

// You can use Area() or Perimeter() as surface area heuristic(SAH) function
inline float SAH(const AABB& aabb)
{
#if 1
    return aabb.GetArea();
#else
    return aabb.GetPerimeter();
#endif
}

typedef int32 NodeProxy;

struct Node
{
    bool IsLeaf() const
    {
        return child1 == nullNode;
    }

    int32 id;
    AABB aabb;

    NodeProxy parent;
    NodeProxy child1;
    NodeProxy child2;

    NodeProxy next;
    bool moved;

    Collider* collider; // user data
};

class AABBTree
{
public:
    AABBTree();
    ~AABBTree() noexcept;

    AABBTree(const AABBTree&) noexcept = delete;
    AABBTree& operator=(const AABBTree&) noexcept = delete;

    AABBTree(AABBTree&&) noexcept = delete;
    AABBTree& operator=(AABBTree&&) noexcept = delete;

    void Reset();

    NodeProxy CreateNode(Collider* collider, const AABB& aabb);
    bool MoveNode(NodeProxy node, AABB aabb, const Vec2& displacement, bool forceMove);
    void RemoveNode(NodeProxy node);

    bool TestOverlap(NodeProxy nodeA, NodeProxy nodeB) const;
    const AABB& GetAABB(NodeProxy node) const;
    void ClearMoved(NodeProxy node) const;
    bool WasMoved(NodeProxy node) const;
    Collider* GetData(NodeProxy node) const;

    void Query(const Vec2& point, const std::function<bool(NodeProxy, Collider*)>& callback) const;
    void Query(const AABB& aabb, const std::function<bool(NodeProxy, Collider*)>& callback) const;
    template <typename T>
    void Query(const Vec2& point, T* callback) const;
    template <typename T>
    void Query(const AABB& aabb, T* callback) const;

    void RayCast(const RayCastInput& input,
                 const std::function<float(const RayCastInput& input, Collider* collider)>& callback) const;
    template <typename T>
    void RayCast(const RayCastInput& input, T* callback) const;

    void Traverse(const std::function<void(const Node*)>& callback) const;
    template <typename T>
    void Traverse(T* callback) const;

    float ComputeTreeCost() const;
    void Rebuild();

private:
    NodeProxy nodeID;
    NodeProxy root;

    Node* nodes;
    int32 nodeCount;
    int32 nodeCapacity;

    NodeProxy freeList;

    NodeProxy AllocateNode();
    void FreeNode(NodeProxy node);

    NodeProxy InsertLeaf(NodeProxy leaf);
    void RemoveLeaf(NodeProxy leaf);

    void Rotate(NodeProxy node);
    void Swap(NodeProxy node1, NodeProxy node2);
};

inline bool AABBTree::TestOverlap(NodeProxy nodeA, NodeProxy nodeB) const
{
    muliAssert(0 <= nodeA && nodeA < nodeCapacity);
    muliAssert(0 <= nodeB && nodeB < nodeCapacity);

    return TestOverlapAABB(nodes[nodeA].aabb, nodes[nodeB].aabb);
}

inline const AABB& AABBTree::GetAABB(NodeProxy node) const
{
    muliAssert(0 <= node && node < nodeCapacity);

    return nodes[node].aabb;
}

inline void AABBTree::ClearMoved(NodeProxy node) const
{
    muliAssert(0 <= node && node < nodeCapacity);

    nodes[node].moved = false;
}

inline bool AABBTree::WasMoved(NodeProxy node) const
{
    muliAssert(0 <= node && node < nodeCapacity);

    return nodes[node].moved;
}

inline Collider* AABBTree::GetData(NodeProxy node) const
{
    muliAssert(0 <= node && node < nodeCapacity);

    return nodes[node].collider;
}

inline float AABBTree::ComputeTreeCost() const
{
    float cost = 0.0f;

    Traverse([&cost](const Node* node) -> void { cost += SAH(node->aabb); });

    return cost;
}

template <typename T>
void AABBTree::Query(const Vec2& point, T* callback) const
{
    if (root == nullNode)
    {
        return;
    }

    GrowableArray<NodeProxy, 256> stack;
    stack.Emplace(root);

    while (stack.Count() != 0)
    {
        NodeProxy current = stack.Pop();

        if (!TestPointInsideAABB(nodes[current].aabb, point))
        {
            continue;
        }

        if (nodes[current].IsLeaf())
        {
            bool proceed = callback->QueryCallback(current, nodes[current].collider);
            if (proceed == false)
            {
                return;
            }
        }
        else
        {
            stack.Emplace(nodes[current].child1);
            stack.Emplace(nodes[current].child2);
        }
    }
}

template <typename T>
void AABBTree::Query(const AABB& aabb, T* callback) const
{
    if (root == nullNode)
    {
        return;
    }

    GrowableArray<NodeProxy, 256> stack;
    stack.Emplace(root);

    while (stack.Count() != 0)
    {
        NodeProxy current = stack.Pop();

        if (!TestOverlapAABB(nodes[current].aabb, aabb))
        {
            continue;
        }

        if (nodes[current].IsLeaf())
        {
            bool proceed = callback->QueryCallback(current, nodes[current].collider);
            if (proceed == false)
            {
                return;
            }
        }
        else
        {
            stack.Emplace(nodes[current].child1);
            stack.Emplace(nodes[current].child2);
        }
    }
}

template <typename T>
void AABBTree::RayCast(const RayCastInput& input, T* callback) const
{
    Vec2 p1 = input.from;
    Vec2 p2 = input.to;
    float maxFraction = input.maxFraction;

    Vec2 d = p2 - p1;
    muliAssert(d.Length2() > 0.0f);
    d.Normalize();

    Vec2 perp = Cross(d, 1.0f); // separating axis
    Vec2 absPerp = Abs(perp);

    Vec2 end = p1 + maxFraction * (p2 - p1);
    AABB rayAABB;
    rayAABB.min = Min(p1, end);
    rayAABB.max = Max(p1, end);

    GrowableArray<NodeProxy, 256> stack;
    stack.Emplace(root);

    while (stack.Count() > 0)
    {
        NodeProxy nodeID = stack.Pop();
        if (nodeID == nullNode)
        {
            continue;
        }

        const Node* node = nodes + nodeID;
        if (TestOverlapAABB(node->aabb, rayAABB) == false)
        {
            continue;
        }

        Vec2 center = (node->aabb.min + node->aabb.max) * 0.5f;
        Vec2 extents = (node->aabb.max - node->aabb.min) * 0.5f;

        float separation = Abs(Dot(perp, p1 - center)) - Dot(absPerp, extents);
        if (separation > 0.0f) // Separating axis test
        {
            continue;
        }

        if (node->IsLeaf())
        {
            RayCastInput subInput;
            subInput.from = p1;
            subInput.to = p2;
            subInput.maxFraction = maxFraction;

            float value = callback->RayCastCallback(subInput, node->collider);
            if (value == 0.0f)
            {
                return;
            }

            if (value > 0.0f)
            {
                // Update ray AABB
                maxFraction = value;
                Vec2 newEnd = p1 + maxFraction * (p2 - p1);
                rayAABB.min = Min(p1, newEnd);
                rayAABB.max = Max(p1, newEnd);
            }
        }
        else
        {
            stack.Emplace(node->child1);
            stack.Emplace(node->child2);
        }
    }
}

template <typename T>
void AABBTree::Traverse(T* callback) const
{
    if (root == nullNode)
    {
        return;
    }

    GrowableArray<NodeProxy, 256> stack;
    stack.Emplace(root);

    while (stack.Count() != 0)
    {
        NodeProxy current = stack.Pop();

        if (!nodes[current].IsLeaf())
        {
            stack.Emplace(nodes[current].child1);
            stack.Emplace(nodes[current].child2);
        }

        const Node* node = nodes + current;
        callback->TraverseCallback(node);
    }
}

} // namespace muli