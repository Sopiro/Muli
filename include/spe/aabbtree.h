#pragma once

#include "common.h"
#include "rigidbody.h"
#include "aabb.h"

namespace spe
{
struct Node
{
    friend class AABBTree;

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
    AABBTree();
    ~AABBTree() noexcept;

    AABBTree(const AABBTree&) noexcept = delete;
    AABBTree& operator=(const AABBTree&) noexcept = delete;

    AABBTree(AABBTree&&) noexcept = delete;
    AABBTree& operator=(AABBTree&&) noexcept = delete;

    void Reset();

    const Node* Add(RigidBody* body);
    void Remove(RigidBody* body);

    // BFS tree traversal
    void Traverse(std::function<void(const Node*)> callback) const;

    void GetCollisionPairs(std::vector<std::pair<RigidBody*, RigidBody*>>& outPairs) const;

    std::vector<Node*> QueryPoint(const glm::vec2& point) const;
    std::vector<Node*> QueryRegion(const AABB& region)  const;

    float GetTreeCost() const;
    float GetMarginSize() const;

private:
    uint32_t nodeID = 0;

    Node* root = nullptr;
    float aabbMargin = 0.05f;

    void Rotate(Node* node);
    void Swap(Node* node1, Node* node2);
    void CheckCollision(Node* a, Node* b, std::vector<std::pair<RigidBody*, RigidBody*>>& pairs, std::unordered_set<uint32_t>& checked) const;
};
}