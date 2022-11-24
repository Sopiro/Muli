#pragma once

#include "aabb.h"
#include "collision.h"
#include "common.h"
#include "rigidbody.h"
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

    RigidBody* body;
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

    int32 Insert(RigidBody* body, const AABB& aabb);
    void Remove(RigidBody* body);
    void Reset();

    void Traverse(std::function<void(const Node*)> callback) const;
    void GetCollisionPairs(std::vector<std::pair<RigidBody*, RigidBody*>>& outPairs) const;

    std::vector<RigidBody*> Query(const Vec2& point) const;
    std::vector<RigidBody*> Query(const AABB& aabb) const;
    void Query(const AABB& aabb, const std::function<bool(RigidBody*)>& callback) const;

    void RayCast(const RayCastInput& input, const std::function<float(const RayCastInput& input, RigidBody*)>& callback) const;

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
                        std::vector<std::pair<RigidBody*, RigidBody*>>& pairs,
                        std::unordered_set<uint64>& checked) const;
};

inline float AABBTree::ComputeTreeCost() const
{
    float cost = 0.0f;

    Traverse([&cost](const Node* node) -> void { cost += SAH(node->aabb); });

    return cost;
}

} // namespace muli