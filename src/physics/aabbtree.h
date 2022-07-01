#pragma once

#include "../common.h"
#include "rigidbody.h"
#include "aabb.h"

namespace spe
{
    struct Node
    {
        friend class AABBTree;

    public:
        ~Node() noexcept = default;

        Node(const Node&) noexcept = delete;
        Node& operator=(const Node&) noexcept = delete;

        Node(Node&&) noexcept = default;
        Node& operator=(Node&&) noexcept = default;

        uint32_t id;
        Node* parent = nullptr;
        Node* child1 = nullptr;
        Node* child2 = nullptr;
        bool isLeaf;
        AABB aabb;
        RigidBody* body = nullptr;
    private:
        Node(uint32_t _id, AABB&& _aabb, bool _isLeaf);
    };

    class AABBTree
    {
    public:
        AABBTree();
        ~AABBTree() noexcept;

        AABBTree(const AABBTree&) noexcept = delete;
        AABBTree(AABBTree&&) noexcept = delete;

        AABBTree& operator=(const AABBTree&) noexcept = delete;
        AABBTree& operator=(AABBTree&&) noexcept = delete;

        void Reset();

        const Node* Add(RigidBody* body);
        void Remove(const Node* node);

        // BFS tree traversal
        void Traverse(std::function<void(const Node*)> callback);

        std::vector<std::pair<const RigidBody*, const RigidBody*>> GetCollisionPairs();

        std::vector<const Node*> QueryPoint(const glm::vec2& point);
        std::vector<const Node*> QueryRegion(const AABB& region);

        float GetTreeCost();
    private:
        uint32_t nodeID = 0;

        Node* root = nullptr;
        float aabbMargin = 0.05f;

        void Rotate(Node* node);
        void Swap(Node* node1, Node* node2);
        void CheckCollision(Node* a, Node* b, std::vector<std::pair<const RigidBody*, const RigidBody*>>& pairs, std::unordered_set<uint32_t>& checked);
    };
}