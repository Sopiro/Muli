#pragma once

#include "common.h"
#include "rigidbody.h"
#include "aabb.h"
#include "settings.h"

namespace spe
{
struct Node
{
    friend class AABBTree;
    friend class BroadPhase;

public:
    uint32_t id;
    AABB aabb;
    bool isLeaf;

    Node* parent = nullptr;
    Node* child1 = nullptr;
    Node* child2 = nullptr;

    RigidBody* body = nullptr;

private:
    Node(uint32_t _id, AABB _aabb, bool _isLeaf);
    ~Node() noexcept = default;

    Node(const Node&) noexcept = delete;
    Node& operator=(const Node&) noexcept = delete;

    Node(Node&&) noexcept = default;
    Node& operator=(Node&&) noexcept = default;
};

class AABBTree
{
public:
    AABBTree(float _aabbMargin = DEFAULT_AABB_MARGIN);
    ~AABBTree() noexcept;

    AABBTree(const AABBTree&) noexcept = delete;
    AABBTree& operator=(const AABBTree&) noexcept = delete;

    AABBTree(AABBTree&&) noexcept = delete;
    AABBTree& operator=(AABBTree&&) noexcept = delete;

    void Reset();

    const Node* Insert(RigidBody* body, AABB aabb);
    void Remove(RigidBody* body);

    // BFS tree traversal
    void Traverse(std::function<void(const Node*)> callback) const;

    void GetCollisionPairs(std::vector<std::pair<RigidBody*, RigidBody*>>& outPairs) const;

    std::vector<Node*> Query(const glm::vec2& point) const;
    std::vector<Node*> Query(const AABB& region)  const;
    void Query(const AABB& aabb, std::function<bool(const Node*)> callback) const;

    float GetTreeCost() const;
    float GetMarginSize() const;

private:
    uint32_t nodeID = 0;

    Node* root = nullptr;
    float aabbMargin;

    void Rotate(Node* node);
    void Swap(Node* node1, Node* node2);
    void CheckCollision(Node* a, Node* b, std::vector<std::pair<RigidBody*, RigidBody*>>& pairs, std::unordered_set<uint64_t>& checked) const;
};

}