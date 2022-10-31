#include "muli/aabbtree.h"
#include "muli/growable_array.h"
#include "muli/util.h"

namespace muli
{

AABBTree::AABBTree()
{
    nodeID = 0;
    root = nullNode;
    nodeCapacity = 32;
    nodeCount = 0;
    nodes = (Node*)malloc(nodeCapacity * sizeof(Node));
    memset(nodes, 0, nodeCapacity * sizeof(Node));

    // Build a linked list for the free list.
    for (int32 i = 0; i < nodeCapacity - 1; ++i)
    {
        nodes[i].next = i + 1;
    }
    nodes[nodeCapacity - 1].next = nullNode;
    freeList = 0;
}

AABBTree::~AABBTree()
{
    free(nodes);
    root = nullNode;
    nodeCount = 0;
}

int32 AABBTree::Insert(RigidBody* body, const AABB& aabb)
{
    int32 newNode = AllocateNode();

    nodes[newNode].id = nodeID++;
    nodes[newNode].aabb = aabb;
    nodes[newNode].isLeaf = true;
    nodes[newNode].body = body;
    nodes[newNode].parent = nullNode;
    body->node = newNode;

    if (root == nullNode)
    {
        root = newNode;
        return newNode;
    }

    // Find the best sibling for the new leaf
    int32 bestSibling = root;
    float bestCost = SAH(Union(nodes[root].aabb, aabb));

    GrowableArray<std::pair<int32, float>, 256> stack;
    stack.Emplace(root, 0.0f);

    while (stack.Count() != 0)
    {
        int32 current = stack.Back().first;
        float inheritedCost = stack.Back().second;
        stack.Pop();

        AABB combined = Union(nodes[current].aabb, aabb);
        float directCost = SAH(combined);

        float costForCurrent = directCost + inheritedCost;
        if (costForCurrent < bestCost)
        {
            bestCost = costForCurrent;
            bestSibling = current;
        }

        inheritedCost += directCost - SAH(nodes[current].aabb);

        float lowerBoundCost = SAH(aabb) + inheritedCost;
        if (lowerBoundCost < bestCost)
        {
            if (!nodes[current].isLeaf)
            {
                stack.Emplace(nodes[current].child1, inheritedCost);
                stack.Emplace(nodes[current].child2, inheritedCost);
            }
        }
    }

    // Create a new parent
    int32 oldParent = nodes[bestSibling].parent;
    int32 newParent = AllocateNode();
    nodes[newParent].id = nodeID++;
    nodes[newParent].aabb = Union(aabb, nodes[bestSibling].aabb);
    nodes[newParent].isLeaf = false;
    nodes[newParent].body = nullptr;
    nodes[newParent].parent = oldParent;

    if (oldParent != nullNode)
    {
        if (nodes[oldParent].child1 == bestSibling)
        {
            nodes[oldParent].child1 = newParent;
        }
        else
        {
            nodes[oldParent].child2 = newParent;
        }

        nodes[newParent].child1 = bestSibling;
        nodes[newParent].child2 = newNode;
        nodes[bestSibling].parent = newParent;
        nodes[newNode].parent = newParent;
    }
    else
    {
        nodes[newParent].child1 = bestSibling;
        nodes[newParent].child2 = newNode;
        nodes[bestSibling].parent = newParent;
        nodes[newNode].parent = newParent;
        root = newParent;
    }

    // Walk back up the tree refitting ancestors' AABB and applying rotations
    int32 ancestor = nodes[newNode].parent;
    while (ancestor != nullNode)
    {
        int32 child1 = nodes[ancestor].child1;
        int32 child2 = nodes[ancestor].child2;

        nodes[ancestor].aabb = Union(nodes[child1].aabb, nodes[child2].aabb);

        Rotate(ancestor);

        ancestor = nodes[ancestor].parent;
    }

    return newNode;
}

void AABBTree::Remove(RigidBody* body)
{
    if (body->node == nullNode)
    {
        return;
    }

    int32 node = body->node;
    int32 parent = nodes[node].parent;
    body->node = nullNode;

    if (parent != nullNode) // node is not root
    {
        int32 sibling = nodes[parent].child1 == node ? nodes[parent].child2 : nodes[parent].child1;

        if (nodes[parent].parent != nullNode) // sibling has grandparent
        {
            nodes[sibling].parent = nodes[parent].parent;

            int32 grandParent = nodes[parent].parent;
            if (nodes[grandParent].child1 == parent)
            {
                nodes[grandParent].child1 = sibling;
            }
            else
            {
                nodes[grandParent].child2 = sibling;
            }
        }
        else // sibling has no grandparent
        {
            root = sibling;

            nodes[sibling].parent = nullNode;
        }

        FreeNode(node);
        FreeNode(parent);

        int32 ancestor = nodes[sibling].parent;
        while (ancestor != nullNode)
        {
            int32 child1 = nodes[ancestor].child1;
            int32 child2 = nodes[ancestor].child2;

            nodes[ancestor].aabb = Union(nodes[child1].aabb, nodes[child2].aabb);

            ancestor = nodes[ancestor].parent;
        }
    }
    else // node is root
    {
        if (root == node)
        {
            root = nullNode;
            FreeNode(node);
        }
    }
}

void AABBTree::Rotate(int32 node)
{
    if (nodes[node].parent == nullNode)
    {
        return;
    }

    int32 parent = nodes[node].parent;
    int32 sibling = nodes[parent].child1 == node ? nodes[parent].child2 : nodes[parent].child1;

    uint32 count = 2;
    float costDiffs[4];
    float nodeArea = SAH(nodes[node].aabb);

    costDiffs[0] = SAH(Union(nodes[sibling].aabb, nodes[nodes[node].child1].aabb)) - nodeArea;
    costDiffs[1] = SAH(Union(nodes[sibling].aabb, nodes[nodes[node].child2].aabb)) - nodeArea;

    if (!nodes[sibling].isLeaf)
    {
        float siblingArea = SAH(nodes[sibling].aabb);
        costDiffs[2] = SAH(Union(nodes[node].aabb, nodes[nodes[sibling].child1].aabb)) - siblingArea;
        costDiffs[3] = SAH(Union(nodes[node].aabb, nodes[nodes[sibling].child2].aabb)) - siblingArea;

        count += 2;
    }

    uint32 bestDiffIndex = 0;
    for (uint32 i = 1; i < count; ++i)
    {
        if (costDiffs[i] < costDiffs[bestDiffIndex]) bestDiffIndex = i;
    }

    if (costDiffs[bestDiffIndex] < 0.0)
    {
        // printf("Tree rotation: %d\n", bestDiffIndex);

        switch (bestDiffIndex)
        {
        case 0: // Swap(sibling, node->child2);
            if (nodes[parent].child1 == sibling)
                nodes[parent].child1 = nodes[node].child2;
            else
                nodes[parent].child2 = nodes[node].child2;

            nodes[nodes[node].child2].parent = parent;

            nodes[node].child2 = sibling;
            nodes[sibling].parent = node;

            nodes[node].aabb = Union(nodes[sibling].aabb, nodes[nodes[node].child1].aabb);
            break;
        case 1: // Swap(sibling, node->child1);
            if (nodes[parent].child1 == sibling)
                nodes[parent].child1 = nodes[node].child1;
            else
                nodes[parent].child2 = nodes[node].child1;

            nodes[nodes[node].child1].parent = parent;

            nodes[node].child1 = sibling;
            nodes[sibling].parent = node;

            nodes[node].aabb = Union(nodes[sibling].aabb, nodes[nodes[node].child2].aabb);
            break;
        case 2: // Swap(node, sibling->child2);
            if (nodes[parent].child1 == node)
                nodes[parent].child1 = nodes[sibling].child2;
            else
                nodes[parent].child2 = nodes[sibling].child2;

            nodes[nodes[sibling].child2].parent = parent;

            nodes[sibling].child2 = node;
            nodes[node].parent = sibling;

            nodes[sibling].aabb = Union(nodes[node].aabb, nodes[nodes[sibling].child2].aabb);
            break;
        case 3: // Swap(node, sibling->child1);
            if (nodes[parent].child1 == node)
                nodes[parent].child1 = nodes[sibling].child1;
            else
                nodes[parent].child2 = nodes[sibling].child1;

            nodes[nodes[sibling].child1].parent = parent;

            nodes[sibling].child1 = node;
            nodes[node].parent = sibling;

            nodes[sibling].aabb = Union(nodes[node].aabb, nodes[nodes[sibling].child1].aabb);
            break;
        }
    }
}

void AABBTree::Swap(int32 node1, int32 node2)
{
    int32 parent1 = nodes[node1].parent;
    int32 parent2 = nodes[node2].parent;

    if (parent1 == parent2)
    {
        nodes[parent1].child1 = node2;
        nodes[parent1].child2 = node1;
        return;
    }

    if (nodes[parent1].child1 == node1)
        nodes[parent1].child1 = node2;
    else
        nodes[parent1].child2 = node2;
    nodes[node2].parent = parent1;

    if (nodes[parent2].child1 == node2)
        nodes[parent2].child1 = node1;
    else
        nodes[parent2].child2 = node1;
    nodes[node1].parent = parent2;
}

void AABBTree::Traverse(std::function<void(const Node*)> callback) const
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

        if (!nodes[current].isLeaf)
        {
            stack.Emplace(nodes[current].child1);
            stack.Emplace(nodes[current].child2);
        }

        const Node* node = nodes + current;
        callback(node);
    }
}

void AABBTree::GetCollisionPairs(std::vector<std::pair<RigidBody*, RigidBody*>>& outPairs) const
{
    if (root == nullNode)
    {
        return;
    }

    std::unordered_set<uint64> checked;

    if (!nodes[root].isLeaf)
    {
        CheckCollision(nodes[root].child1, nodes[root].child2, outPairs, checked);
    }
}

void AABBTree::CheckCollision(int32 nodeA,
                              int32 nodeB,
                              std::vector<std::pair<RigidBody*, RigidBody*>>& pairs,
                              std::unordered_set<uint64>& checked) const
{
    const uint64 key = CombineID(nodes[nodeA].id, nodes[nodeB].id).key;

    if (checked.find(key) != checked.end())
    {
        return;
    }

    checked.insert(key);

    if (nodes[nodeA].isLeaf && nodes[nodeB].isLeaf)
    {
        if (TestOverlapAABB(nodes[nodeA].aabb, nodes[nodeB].aabb))
        {
            pairs.emplace_back(nodes[nodeA].body, nodes[nodeB].body);
        }
    }
    else if (!nodes[nodeA].isLeaf && !nodes[nodeB].isLeaf)
    {
        CheckCollision(nodes[nodeA].child1, nodes[nodeA].child2, pairs, checked);
        CheckCollision(nodes[nodeB].child1, nodes[nodeB].child2, pairs, checked);

        if (TestOverlapAABB(nodes[nodeA].aabb, nodes[nodeB].aabb))
        {
            CheckCollision(nodes[nodeA].child1, nodes[nodeB].child1, pairs, checked);
            CheckCollision(nodes[nodeA].child1, nodes[nodeB].child2, pairs, checked);
            CheckCollision(nodes[nodeA].child2, nodes[nodeB].child1, pairs, checked);
            CheckCollision(nodes[nodeA].child2, nodes[nodeB].child2, pairs, checked);
        }
    }
    else if (nodes[nodeA].isLeaf && !nodes[nodeB].isLeaf)
    {
        CheckCollision(nodes[nodeB].child1, nodes[nodeB].child2, pairs, checked);

        if (TestOverlapAABB(nodes[nodeA].aabb, nodes[nodeB].aabb))
        {
            CheckCollision(nodeA, nodes[nodeB].child1, pairs, checked);
            CheckCollision(nodeA, nodes[nodeB].child2, pairs, checked);
        }
    }
    else if (!nodes[nodeA].isLeaf && nodes[nodeB].isLeaf)
    {
        CheckCollision(nodes[nodeA].child1, nodes[nodeA].child2, pairs, checked);

        if (TestOverlapAABB(nodes[nodeA].aabb, nodes[nodeB].aabb))
        {
            CheckCollision(nodeB, nodes[nodeA].child1, pairs, checked);
            CheckCollision(nodeB, nodes[nodeA].child2, pairs, checked);
        }
    }
}

std::vector<RigidBody*> AABBTree::Query(const Vec2& point) const
{
    std::vector<RigidBody*> res;
    res.reserve(8);

    if (root == nullNode)
    {
        return res;
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
            res.push_back(nodes[current].body);
        }
        else
        {
            stack.Emplace(nodes[current].child1);
            stack.Emplace(nodes[current].child2);
        }
    }

    return res;
}

std::vector<RigidBody*> AABBTree::Query(const AABB& aabb) const
{
    std::vector<RigidBody*> res;
    res.reserve(8);

    if (root == nullNode)
    {
        return res;
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
            res.push_back(nodes[current].body);
        }
        else
        {
            stack.Emplace(nodes[current].child1);
            stack.Emplace(nodes[current].child2);
        }
    }

    return res;
}

void AABBTree::Query(const AABB& aabb, const std::function<bool(RigidBody*)>& callback) const
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
            bool proceed = callback(nodes[current].body);
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

void AABBTree::RayCast(const RayCastInput& input, const std::function<float(const RayCastInput&, RigidBody*)>& callback) const
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

    GrowableArray<int32, 256> stack;
    stack.Emplace(root);

    while (stack.Count() > 0)
    {
        int32 nodeID = stack.Pop();
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

        if (node->isLeaf)
        {
            RayCastInput subInput;
            subInput.from = p1;
            subInput.to = p2;
            subInput.maxFraction = maxFraction;

            float value = callback(subInput, node->body);
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

void AABBTree::Reset()
{
    nodeID = 0;
    root = nullNode;
    nodeCount = 0;
    memset(nodes, 0, nodeCapacity * sizeof(Node));

    // Build a linked list for the free list.
    for (int32 i = 0; i < nodeCapacity - 1; ++i)
    {
        nodes[i].next = i + 1;
    }
    nodes[nodeCapacity - 1].next = nullNode;
    freeList = 0;
}

int32 AABBTree::AllocateNode()
{
    if (freeList == nullNode)
    {
        muliAssert(nodeCount == nodeCapacity);

        // Grow the node pool
        Node* oldNodes = nodes;
        nodeCapacity *= 2;
        nodes = (Node*)malloc(nodeCapacity * sizeof(Node));
        memcpy(nodes, oldNodes, nodeCount * sizeof(Node));
        memset(nodes + nodeCount, 0, nodeCount * sizeof(Node));
        free(oldNodes);

        // Build a linked list for the free list.
        for (int32 i = nodeCount; i < nodeCapacity - 1; ++i)
        {
            nodes[i].next = i + 1;
        }
        nodes[nodeCapacity - 1].next = nullNode;
        freeList = nodeCount;
    }

    int32 node = freeList;
    freeList = nodes[node].next;
    nodes[node].parent = nullNode;
    nodes[node].child1 = nullNode;
    nodes[node].child2 = nullNode;
    ++nodeCount;

    return node;
}

void AABBTree::FreeNode(int32 node)
{
    muliAssert(0 <= node && node <= nodeCapacity);
    muliAssert(0 < nodeCount);

    nodes[node].next = freeList;
    freeList = node;
    --nodeCount;
}

} // namespace muli