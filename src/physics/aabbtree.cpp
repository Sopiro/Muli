#include "aabbtree.h"

using namespace spe;

Node::Node(uint32_t _id, AABB&& _aabb, bool _isLeaf) :
    id{ std::move(_id) },
    aabb{ std::move(_aabb) },
    isLeaf{ std::move(_isLeaf) }
{

}

AABBTree::AABBTree()
{
}

AABBTree::~AABBTree()
{
    Reset();
}

void AABBTree::Reset()
{
    Traverse
    (
        [&](const Node* n) -> void
        {
            delete n;
        }
    );

    nodeID = 0;
    root = nullptr;
}

const Node* AABBTree::Add(RigidBody* body)
{
    AABB aabb = create_AABB(*body, body->GetType() == Static ? 0.0f : aabbMargin);

    Node* newNode = new Node(nodeID++, std::move(aabb), true);
    newNode->body = body;
    body->node = newNode;

    if (root == nullptr)
    {
        root = newNode;
    }
    else
    {
        // Find the best sibling for the new leaf
        Node* bestSibling = root;
        float bestCost = area(union_of(root->aabb, aabb));

        std::queue<Node*> q;
        q.push(root);

        while (q.size() != 0)
        {
            Node* current = q.front();
            q.pop();

            float directCost = area(union_of(current->aabb, aabb));
            float inheritedCost = 0.0f;

            Node* ancestor = current->parent;
            while (ancestor != nullptr)
            {
                inheritedCost += area(union_of(current->aabb, aabb)) - area(ancestor->aabb);
                ancestor = ancestor->parent;
            }

            float costForCurrent = directCost + inheritedCost;

            if (costForCurrent < bestCost)
            {
                bestCost = costForCurrent;
                bestSibling = current;
            }

            float lowerBoundCost = area(aabb) + (area(union_of(current->aabb, aabb)) - area(current->aabb)) + inheritedCost;

            if (lowerBoundCost < bestCost)
            {
                if (!current->isLeaf)
                {
                    q.push(current->child1);
                    q.push(current->child2);
                }
            }
        }

        // Create a new parent
        Node* oldParent = bestSibling->parent;
        Node* newParent = new Node(nodeID++, union_of(aabb, bestSibling->aabb), false);
        newParent->parent = oldParent;

        if (oldParent != nullptr)
        {
            if (oldParent->child1 == bestSibling)
            {
                oldParent->child1 = newParent;
            }
            else
            {
                oldParent->child2 = newParent;
            }

            newParent->child1 = bestSibling;
            newParent->child2 = newNode;
            bestSibling->parent = newParent;
            newNode->parent = newParent;
        }
        else
        {
            newParent->child1 = bestSibling;
            newParent->child2 = newNode;
            bestSibling->parent = newParent;
            newNode->parent = newParent;
            root = newParent;
        }

        // Walk back up the tree refitting ancestors' AABB and applying rotations
        Node* ancestor = newNode->parent;
        while (ancestor != nullptr)
        {
            Node* child1 = ancestor->child1;
            Node* child2 = ancestor->child2;

            ancestor->aabb = union_of(child1->aabb, child2->aabb);

            Rotate(ancestor);

            ancestor = ancestor->parent;
        }
    }

    return newNode;
}

void AABBTree::Remove(RigidBody* body)
{
    if (body->node == nullptr) return;

    Node* node = body->node;
    Node* parent = body->node->parent;
    body->node = nullptr;

    if (parent != nullptr) // node is not root
    {
        Node* sibling = parent->child1 == node ? parent->child2 : parent->child1;

        if (parent->parent != nullptr) // sibling has grandparent
        {
            sibling->parent = parent->parent;
            if (parent->parent->child1 == parent)
            {
                parent->parent->child1 = sibling;
            }
            else
            {
                parent->parent->child2 = sibling;
            }
        }
        else // sibling has no grandparent
        {
            root = sibling;

            sibling->parent = nullptr;
        }

        delete node;
        delete parent;

        Node* ancestor = sibling->parent;
        while (ancestor != nullptr)
        {
            Node* child1 = ancestor->child1;
            Node* child2 = ancestor->child2;

            ancestor->aabb = union_of(child1->aabb, child2->aabb);

            ancestor = ancestor->parent;
        }
    }
    else // node is root
    {
        if (root == node)
        {
            root = nullptr;
            delete node;
        }
    }
}

void AABBTree::Rotate(Node* node)
{
    if (node->parent == nullptr) return;

    Node* parent = node->parent;
    Node* sibling = parent->child1 == node ? parent->child2 : parent->child1;

    uint8_t count = 2;
    std::array<float, 4> costDiffs;
    float nodeArea = area(node->aabb);

    costDiffs[0] = area(union_of(sibling->aabb, node->child1->aabb)) - nodeArea;
    costDiffs[1] = area(union_of(sibling->aabb, node->child2->aabb)) - nodeArea;

    if (!sibling->isLeaf)
    {
        float siblingArea = area(sibling->aabb);
        costDiffs[2] = area(union_of(node->aabb, sibling->child1->aabb)) - siblingArea;
        costDiffs[3] = area(union_of(node->aabb, sibling->child2->aabb)) - siblingArea;

        count += 2;
    }

    size_t bestDiffIndex = 0;
    for (size_t i = 1; i < count; i++)
    {
        if (costDiffs[i] < costDiffs[bestDiffIndex])
            bestDiffIndex = i;
    }

    if (costDiffs[bestDiffIndex] < 0.0)
    {
        // SPDLOG_INFO("Tree rotation: tpye {}", bestDiffIndex);

        switch (bestDiffIndex)
        {
        case 0: // Swap(sibling, node->child2);
            if (parent->child1 == sibling)
                parent->child1 = node->child2;
            else
                parent->child2 = node->child2;

            node->child2->parent = parent;

            node->child2 = sibling;
            sibling->parent = node;

            node->aabb = union_of(sibling->aabb, node->child1->aabb);
            break;
        case 1: // Swap(sibling, node->child1);
            if (parent->child1 == sibling)
                parent->child1 = node->child1;
            else
                parent->child2 = node->child1;

            node->child1->parent = parent;

            node->child1 = sibling;
            sibling->parent = node;

            node->aabb = union_of(sibling->aabb, node->child2->aabb);
            break;
        case 2: // Swap(node, sibling->child2);
            if (parent->child1 == node)
                parent->child1 = sibling->child2;
            else
                parent->child2 = sibling->child2;

            sibling->child2->parent = parent;

            sibling->child2 = node;
            node->parent = sibling;

            sibling->aabb = union_of(node->aabb, sibling->child2->aabb);
            break;
        case 3: // Swap(node, sibling->child1);
            if (parent->child1 == node)
                parent->child1 = sibling->child1;
            else
                parent->child2 = sibling->child1;

            sibling->child1->parent = parent;

            sibling->child1 = node;
            node->parent = sibling;

            sibling->aabb = union_of(node->aabb, sibling->child1->aabb);
            break;
        }
    }
}

void AABBTree::Swap(Node* node1, Node* node2)
{
    Node* parent1 = node1->parent;
    Node* parent2 = node2->parent;

    if (parent1 == parent2)
    {
        parent1->child1 = node2;
        parent1->child2 = node1;
        return;
    }

    if (parent1->child1 == node1)
        parent1->child1 = node2;
    else
        parent1->child2 = node2;
    node2->parent = parent1;

    if (parent2->child1 == node2)
        parent2->child1 = node1;
    else
        parent2->child2 = node1;
    node1->parent = parent2;
}

void AABBTree::Traverse(std::function<void(const Node*)> callback)
{
    if (root == nullptr) return;

    std::queue<const Node*> q;
    q.push(root);

    while (q.size() != 0)
    {
        const Node* current = q.front();
        q.pop();

        if (!current->isLeaf)
        {
            q.push(current->child1);
            q.push(current->child2);
        }

        callback(current);
    }
}

std::vector<std::pair<const RigidBody*, const RigidBody*>> AABBTree::GetCollisionPairs()
{
    std::vector<std::pair<const RigidBody*, const RigidBody*>> pairs;

    if (root == nullptr) return pairs;

    pairs.reserve(nodeID / 2 + 1);

    std::unordered_set<uint32_t> checked;

    if (!root->isLeaf)
    {
        CheckCollision(root->child1, root->child2, pairs, checked);
    }

    return pairs;
}

void AABBTree::CheckCollision(Node* a, Node* b, std::vector<std::pair<const RigidBody*, const RigidBody*>>& pairs, std::unordered_set<uint32_t>& checked)
{
    const uint32_t key = make_pair_natural(a->id, b->id);

    if (checked.find(key) != checked.end()) return;

    checked.insert(key);

    if (a->isLeaf && b->isLeaf)
    {
        if (detect_collision_AABB(a->aabb, b->aabb))
        {
            pairs.emplace_back(a->body, b->body);
        }
    }
    else if (!a->isLeaf && !b->isLeaf)
    {
        CheckCollision(a->child1, a->child2, pairs, checked);
        CheckCollision(b->child1, b->child2, pairs, checked);

        if (detect_collision_AABB(a->aabb, b->aabb))
        {
            CheckCollision(a->child1, b->child1, pairs, checked);
            CheckCollision(a->child1, b->child2, pairs, checked);
            CheckCollision(a->child2, b->child1, pairs, checked);
            CheckCollision(a->child2, b->child2, pairs, checked);
        }
    }
    else if (a->isLeaf && !b->isLeaf)
    {
        CheckCollision(b->child1, b->child2, pairs, checked);

        if (detect_collision_AABB(a->aabb, b->aabb))
        {
            CheckCollision(a, b->child1, pairs, checked);
            CheckCollision(a, b->child2, pairs, checked);
        }
    }
    else if (!a->isLeaf && b->isLeaf)
    {
        CheckCollision(a->child1, a->child2, pairs, checked);

        if (detect_collision_AABB(a->aabb, b->aabb))
        {
            CheckCollision(b, a->child1, pairs, checked);
            CheckCollision(b, a->child2, pairs, checked);
        }
    }
}

std::vector<Node*> AABBTree::QueryPoint(const glm::vec2& point)
{
    std::vector<Node*> res;

    if (root == nullptr) return res;

    std::queue<Node*> q;
    q.push(root);

    while (q.size() != 0)
    {
        Node* current = q.front();
        q.pop();

        if (!test_point_inside_AABB(current->aabb, point))
            continue;

        if (current->isLeaf)
        {
            res.push_back(current);
        }
        else
        {
            q.push(current->child1);
            q.push(current->child2);
        }
    }

    return res;
}

std::vector<Node*> AABBTree::QueryRegion(const AABB& region)
{
    std::vector<Node*> res;

    if (root == nullptr) return res;

    std::queue<Node*> q;
    q.push(root);

    while (q.size() != 0)
    {
        Node* current = q.front();
        q.pop();

        if (!detect_collision_AABB(current->aabb, region))
            continue;

        if (current->isLeaf)
        {
            res.push_back(current);
        }
        else
        {
            q.push(current->child1);
            q.push(current->child2);
        }
    }

    return res;
}

float AABBTree::GetTreeCost()
{
    float res = 0;

    Traverse
    (
        [&res](const Node* node) -> void
        {
            res += area(node->aabb);
        }
    );

    return res;
}