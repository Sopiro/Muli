#pragma once

#include "../common.h"
#include "rigidbody.h"
#include "aabb.h"

namespace spe
{
    struct Node
    {
        Node(uint32_t _id, AABB _aabb, bool _isLeaf);
        ~Node();

        uint32_t id;
        Node* parent = nullptr;
        Node* child1 = nullptr;
        Node* child2 = nullptr;
        bool isLeaf;
        AABB aabb;
        RigidBody* body = nullptr;
    };

    class AABBTree
    {
    public:
        AABBTree();
        ~AABBTree();

        void Reset();

        Node* Add(RigidBody& body);
        void Remove(Node* node);

        // BFS tree traversal
        void Traverse(std::function<void(Node*)> callback);

        std::vector<std::pair<RigidBody*, RigidBody*>> GetCollisionPairs();

        std::vector<Node*> QueryPoint(const glm::vec2& point);
        std::vector<Node*> QueryRegion(const AABB& region);

        float GetTreeCost();

        // std::vector<Node*> QueryPoint(const glm::vec2& point);
        // std::vector<Node*> QueryRegion(const AABB& region);
    private:
        uint32_t nodeID = 0;

        Node* root = nullptr;
        float aabbMargin = 0.05f;

        void Rotate(Node* node);
        void Swap(Node* node1, Node* node2);
        void CheckCollision(Node* a, Node* b, std::vector<std::pair<RigidBody*, RigidBody*>>& pairs, std::set<uint32_t>& checked);
    };
}