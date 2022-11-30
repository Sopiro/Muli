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
    return Area(aabb);
#else
    return Perimeter(aabb);
#endif
}

// typedef int32 NodePointer;

struct Node
{
    uint32 id;
    AABB aabb;
    bool isLeaf;

    int32 parent;
    int32 child1;
    int32 child2;

    Collider* collider;
    int32 next;
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

    int32 Insert(Collider* collider, const AABB& aabb);
    void Remove(Collider* collider);
    void Reset();

    void Traverse(std::function<void(const Node*)> callback) const;
    void GetCollisionPairs(std::vector<std::pair<Collider*, Collider*>>& outPairs) const;

    void Query(const AABB& aabb, const std::function<bool(Collider*)>& callback) const;
    void Query(const Vec2& point, const std::function<bool(Collider*)>& callback) const;
    std::vector<Collider*> Query(const AABB& aabb) const;
    std::vector<Collider*> Query(const Vec2& point) const;
    template <typename T>
    void Query(const AABB& aabb, T* callback) const;
    template <typename T>
    void Query(const Vec2& point, T* callback) const;

    void RayCast(const RayCastInput& input, const std::function<float(const RayCastInput& input, Collider*)>& callback) const;

    float ComputeTreeCost() const;
    void Rebuild();

private:
    friend class BroadPhase;

    uint32 nodeID = 0;

    Node* nodes;
    int32 root;

    int32 nodeCount;
    int32 nodeCapacity;

    int32 freeList;

    int32 AllocateNode();
    void FreeNode(int32 node);

    void Rotate(int32 node);
    void Swap(int32 node1, int32 node2);
    void CheckCollision(int32 nodeA,
                        int32 nodeB,
                        std::vector<std::pair<Collider*, Collider*>>& pairs,
                        std::unordered_set<uint64>& checked) const;
};

inline float AABBTree::ComputeTreeCost() const
{
    float cost = 0.0f;

    Traverse([&cost](const Node* node) -> void { cost += SAH(node->aabb); });

    return cost;
}

template <typename T>
void AABBTree::Query(const AABB& aabb, T* callback) const
{
    if (root == nullNode)
    {
        return;
    }

    GrowableArray<int32, 256> stack;
    stack.Emplace(root);

    while (stack.Count() != 0)
    {
        int32 current = stack.Pop();

        if (!TestOverlapAABB(nodes[current].aabb, aabb))
        {
            continue;
        }

        if (nodes[current].isLeaf)
        {
            bool proceed = callback->QueryCallback(nodes[current].collider);
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
void AABBTree::Query(const Vec2& point, T* callback) const
{
    if (root == nullNode)
    {
        return;
    }

    GrowableArray<int32, 256> stack;
    stack.Emplace(root);

    while (stack.Count() != 0)
    {
        int32 current = stack.Pop();

        if (!TestPointInsideAABB(nodes[current].aabb, point))
        {
            continue;
        }

        if (nodes[current].isLeaf)
        {
            bool proceed = callback->QueryCallback(nodes[current].collider);
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

} // namespace muli