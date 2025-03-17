#pragma once

#include "aabb.h"
#include "collider.h"
#include "collision.h"
#include "growable_array.h"
#include "settings.h"

namespace muli
{

// You can use either Area() or Perimeter() as a surface area heuristic(SAH) function
inline float SurfaceArea(const AABB& aabb)
{
#if 0
    return aabb.GetArea();
#else
    return aabb.GetPerimeter();
#endif
}

typedef int32 NodeProxy;
typedef Collider Data;

class AABBTree
{
public:
    static constexpr inline int32 nullNode = -1;

    struct Node
    {
        bool IsLeaf() const
        {
            return child1 == nullNode;
        }

        AABB aabb;

        NodeProxy parent;
        NodeProxy child1;
        NodeProxy child2;

        NodeProxy next;
        bool moved;

        Data* data; // user data
    };

    AABBTree();
    ~AABBTree() noexcept;

    AABBTree(const AABBTree&) = delete;
    AABBTree& operator=(const AABBTree&) = delete;

    AABBTree(AABBTree&&) noexcept;
    AABBTree& operator=(AABBTree&&) noexcept;

    void Reset();

    NodeProxy CreateNode(Data* data, const AABB& aabb);
    bool MoveNode(NodeProxy node, AABB aabb, const Vec2& displacement, bool forceMove);
    void RemoveNode(NodeProxy node);

    bool TestOverlap(NodeProxy nodeA, NodeProxy nodeB) const;
    const AABB& GetAABB(NodeProxy node) const;
    void ClearMoved(NodeProxy node) const;
    bool WasMoved(NodeProxy node) const;
    Data* GetData(NodeProxy node) const;

    template <typename T>
    void Traverse(T* callback) const;
    template <typename T>
    void Query(const Vec2& point, T* callback) const;
    template <typename T>
    void Query(const AABB& aabb, T* callback) const;
    template <typename T>
    void AABBCast(const AABBCastInput& input, T* callback) const;

    void Traverse(std::function<void(const Node*)> callback) const;
    void Query(const Vec2& point, std::function<bool(NodeProxy, Data*)> callback) const;
    void Query(const AABB& aabb, std::function<bool(NodeProxy, Data*)> callback) const;
    void AABBCast(const AABBCastInput& input, std::function<float(const AABBCastInput& input, Data* data)> callback) const;

    float ComputeTreeCost() const;
    void Rebuild();

private:
    NodeProxy root;

    Node* nodes;
    int32 nodeCapacity;
    int32 nodeCount;

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
    MuliAssert(0 <= nodeA && nodeA < nodeCapacity);
    MuliAssert(0 <= nodeB && nodeB < nodeCapacity);

    return nodes[nodeA].aabb.TestOverlap(nodes[nodeB].aabb);
}

inline const AABB& AABBTree::GetAABB(NodeProxy node) const
{
    MuliAssert(0 <= node && node < nodeCapacity);

    return nodes[node].aabb;
}

inline void AABBTree::ClearMoved(NodeProxy node) const
{
    MuliAssert(0 <= node && node < nodeCapacity);

    nodes[node].moved = false;
}

inline bool AABBTree::WasMoved(NodeProxy node) const
{
    MuliAssert(0 <= node && node < nodeCapacity);

    return nodes[node].moved;
}

inline Data* AABBTree::GetData(NodeProxy node) const
{
    MuliAssert(0 <= node && node < nodeCapacity);

    return nodes[node].data;
}

inline float AABBTree::ComputeTreeCost() const
{
    float cost = 0.0f;

    Traverse([&cost](const Node* node) -> void { cost += SurfaceArea(node->aabb); });

    return cost;
}

template <typename T>
void AABBTree::Traverse(T* callback) const
{
    if (root == nullNode)
    {
        return;
    }

    GrowableArray<NodeProxy, 64> stack;
    stack.EmplaceBack(root);

    while (stack.Count() != 0)
    {
        NodeProxy current = stack.PopBack();

        if (nodes[current].IsLeaf() == false)
        {
            stack.EmplaceBack(nodes[current].child1);
            stack.EmplaceBack(nodes[current].child2);
        }

        const Node* node = nodes + current;
        callback->TraverseCallback(node);
    }
}

template <typename T>
void AABBTree::Query(const Vec2& point, T* callback) const
{
    if (root == nullNode)
    {
        return;
    }

    GrowableArray<NodeProxy, 64> stack;
    stack.EmplaceBack(root);

    while (stack.Count() != 0)
    {
        NodeProxy current = stack.PopBack();

        if (nodes[current].aabb.TestPoint(point) == false)
        {
            continue;
        }

        if (nodes[current].IsLeaf())
        {
            bool proceed = callback->QueryCallback(current, nodes[current].data);
            if (proceed == false)
            {
                return;
            }
        }
        else
        {
            stack.EmplaceBack(nodes[current].child1);
            stack.EmplaceBack(nodes[current].child2);
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

    GrowableArray<NodeProxy, 64> stack;
    stack.EmplaceBack(root);

    while (stack.Count() != 0)
    {
        NodeProxy current = stack.PopBack();

        if (nodes[current].aabb.TestOverlap(aabb) == false)
        {
            continue;
        }

        if (nodes[current].IsLeaf())
        {
            bool proceed = callback->QueryCallback(current, nodes[current].data);
            if (proceed == false)
            {
                return;
            }
        }
        else
        {
            stack.EmplaceBack(nodes[current].child1);
            stack.EmplaceBack(nodes[current].child2);
        }
    }
}

template <typename T>
void AABBTree::AABBCast(const AABBCastInput& input, T* callback) const
{
    const Vec2 p1 = input.from;
    const Vec2 p2 = input.to;
    const Vec2 halfExtents = input.halfExtents;

    float maxFraction = input.maxFraction;

    Vec2 d = p2 - p1;
    float length = d.NormalizeSafe();
    if (length == 0.0f)
    {
        return;
    }

    GrowableArray<NodeProxy, 64> stack;
    stack.EmplaceBack(root);

    while (stack.Count() > 0)
    {
        NodeProxy current = stack.PopBack();
        if (current == nullNode)
        {
            continue;
        }

        const Node* node = nodes + current;

        if (node->IsLeaf())
        {
            AABBCastInput subInput;
            subInput.from = p1;
            subInput.to = p2;
            subInput.maxFraction = maxFraction;
            subInput.halfExtents = halfExtents;

            float newFraction = callback->AABBCastCallback(subInput, node->data);
            if (newFraction == 0.0f)
            {
                return;
            }

            if (newFraction > 0.0f)
            {
                // Shorten the ray
                maxFraction = newFraction;
            }
        }
        else
        {
            // Ordered traversal
            NodeProxy child1 = node->child1;
            NodeProxy child2 = node->child2;

            float dist1 = nodes[child1].aabb.RayCast(p1, p2, 0.0f, maxFraction, halfExtents);
            float dist2 = nodes[child2].aabb.RayCast(p1, p2, 0.0f, maxFraction, halfExtents);

            if (dist2 < dist1)
            {
                std::swap(dist1, dist2);
                std::swap(child1, child2);
            }

            if (dist1 == max_value)
            {
                continue;
            }
            else
            {
                if (dist2 != max_value)
                {
                    stack.EmplaceBack(child2);
                }
                stack.EmplaceBack(child1);
            }
        }
    }
}

} // namespace muli