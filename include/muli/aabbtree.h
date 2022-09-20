#pragma once

#include "aabb.h"
#include "common.h"
#include "rigidbody.h"
#include "settings.h"

namespace muli
{

struct Node
{
    friend class AABBTree;
    friend class BroadPhase;

public:
    uint32 id;
    AABB aabb;
    bool isLeaf;

    Node* parent = nullptr;
    Node* child1 = nullptr;
    Node* child2 = nullptr;

    RigidBody* body = nullptr;

private:
    Node(uint32 _id, const AABB& _aabb, bool _isLeaf);
    ~Node() noexcept = default;

    Node(const Node&) noexcept = delete;
    Node& operator=(const Node&) noexcept = delete;

    Node(Node&&) noexcept = default;
    Node& operator=(Node&&) noexcept = default;
};

class AABBTree
{
public:
    AABBTree() = default;
    ~AABBTree() noexcept;

    AABBTree(const AABBTree&) noexcept = delete;
    AABBTree& operator=(const AABBTree&) noexcept = delete;

    AABBTree(AABBTree&&) noexcept = delete;
    AABBTree& operator=(AABBTree&&) noexcept = delete;

    const Node* Insert(RigidBody* body, const AABB& aabb);
    void Remove(RigidBody* body);
    void Reset();

    void Traverse(std::function<void(const Node*)> callback) const;
    void GetCollisionPairs(std::vector<std::pair<RigidBody*, RigidBody*>>& outPairs) const;

    std::vector<Node*> Query(const Vec2& point) const;
    std::vector<Node*> Query(const AABB& aabb) const;
    void Query(const AABB& aabb, std::function<bool(const Node*)> callback) const;

    float ComputeTreeCost() const;

private:
    uint32 nodeID = 0;

    Node* root = nullptr;
    float aabbMargin;

    void Rotate(Node* node);
    void Swap(Node* node1, Node* node2);
    void CheckCollision(Node* a,
                        Node* b,
                        std::vector<std::pair<RigidBody*, RigidBody*>>& pairs,
                        std::unordered_set<uint64>& checked) const;
};

inline AABBTree::~AABBTree()
{
    Reset();
}

inline void AABBTree::Reset()
{
    Traverse([&](const Node* n) -> void { delete n; });

    nodeID = 0;
    root = nullptr;
}

inline float AABBTree::ComputeTreeCost() const
{
    float cost = 0.0f;

    Traverse([&](const Node* node) -> void { cost += Area(node->aabb); });

    return cost;
}

} // namespace muli