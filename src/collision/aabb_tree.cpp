#include "muli/aabb_tree.h"
#include "muli/growable_array.h"

namespace muli
{

AABBTree::AABBTree()
    : root{ nullNode }
    , nodeCapacity{ 32 }
    , nodeCount{ 0 }
{
    nodes = (Node*)muli::Alloc(nodeCapacity * sizeof(Node));
    memset(nodes, 0, nodeCapacity * sizeof(Node));

    // Build a linked list for the free list.
    for (int32 i = 0; i < nodeCapacity - 1; ++i)
    {
        nodes[i].next = i + 1;
        nodes[i].parent = i;
    }
    nodes[nodeCapacity - 1].next = nullNode;
    nodes[nodeCapacity - 1].parent = nodeCapacity - 1;

    freeList = 0;
}

AABBTree::~AABBTree() noexcept
{
    muli::Free(nodes);
    root = nullNode;
    nodeCount = 0;
}

AABBTree::AABBTree(AABBTree&& other) noexcept
{
    root = other.root;

    nodes = other.nodes;
    nodeCount = other.nodeCount;
    nodeCapacity = other.nodeCapacity;

    freeList = other.freeList;

    other.root = nullNode;

    other.nodes = nullptr;
    other.nodeCount = 0;
    other.nodeCapacity = 0;

    other.freeList = nullNode;
}

AABBTree& AABBTree::operator=(AABBTree&& other) noexcept
{
    muliAssert(this != &other);

    muli::Free(nodes);

    root = other.root;

    nodes = other.nodes;
    nodeCount = other.nodeCount;
    nodeCapacity = other.nodeCapacity;

    freeList = other.freeList;

    other.root = nullNode;

    other.nodes = nullptr;
    other.nodeCount = 0;
    other.nodeCapacity = 0;

    other.freeList = nullNode;

    return *this;
}

NodeProxy AABBTree::InsertLeaf(NodeProxy leaf)
{
    muliAssert(0 <= leaf && leaf < nodeCapacity);
    muliAssert(nodes[leaf].IsLeaf());

    if (root == nullNode)
    {
        root = leaf;
        return leaf;
    }

    AABB aabb = nodes[leaf].aabb;

    // Find the best sibling for the new leaf

#if 1
    NodeProxy bestSibling = root;
    float bestCost = SAH(AABB::Union(nodes[root].aabb, aabb));

    // Candidate node with inherited cost
    struct Candidate
    {
        NodeProxy node;
        float inheritedCost;
    };

    GrowableArray<Candidate, 256> stack;
    stack.EmplaceBack(root, 0.0f);

    while (stack.Count() != 0)
    {
        NodeProxy current = stack.Back().node;
        float inheritedCost = stack.Back().inheritedCost;
        stack.PopBack();

        AABB combined = AABB::Union(nodes[current].aabb, aabb);
        float directCost = SAH(combined);

        float cost = directCost + inheritedCost;
        if (cost < bestCost)
        {
            bestCost = cost;
            bestSibling = current;
        }

        inheritedCost += directCost - SAH(nodes[current].aabb);

        float lowerBoundCost = SAH(aabb) + inheritedCost;
        if (lowerBoundCost < bestCost)
        {
            if (nodes[current].IsLeaf() == false)
            {
                stack.EmplaceBack(nodes[current].child1, inheritedCost);
                stack.EmplaceBack(nodes[current].child2, inheritedCost);
            }
        }
    }
#else
    // O(log n)
    // This method is faster when inserting a new node, but builds a slightly bad quality tree.
    NodeProxy bestSibling = root;
    while (nodes[bestSibling].IsLeaf() == false)
    {
        NodeProxy child1 = nodes[bestSibling].child1;
        NodeProxy child2 = nodes[bestSibling].child2;

        float area = SAH(nodes[bestSibling].aabb);
        AABB combined = AABB::Union(nodes[bestSibling].aabb, aabb);
        float combinedArea = SAH(combined);

        float cost = combinedArea;
        float inheritanceCost = combinedArea - area;

        float cost1;
        if (nodes[child1].IsLeaf())
        {
            cost1 = SAH(AABB::Union(nodes[child1].aabb, aabb)) + inheritanceCost;
        }
        else
        {
            float newArea = SAH(AABB::Union(nodes[child1].aabb, aabb));
            float oldArea = SAH(nodes[child1].aabb);
            cost1 = (newArea - oldArea) + inheritanceCost; // Lower bound cost required when descending down to child1
        }

        float cost2;
        if (nodes[child2].IsLeaf())
        {
            cost2 = SAH(AABB::Union(nodes[child2].aabb, aabb)) + inheritanceCost;
        }
        else
        {
            float newArea = SAH(AABB::Union(nodes[child2].aabb, aabb));
            float oldArea = SAH(nodes[child2].aabb);
            cost2 = (newArea - oldArea) + inheritanceCost; // Lower bound cost required when descending down to child2
        }

        if (cost < cost1 && cost < cost2)
        {
            break;
        }

        if (cost1 < cost2)
        {
            bestSibling = child1;
        }
        else
        {
            bestSibling = child2;
        }
    }
#endif

    // Create a new parent
    NodeProxy oldParent = nodes[bestSibling].parent;
    NodeProxy newParent = AllocateNode();
    nodes[newParent].aabb = AABB::Union(aabb, nodes[bestSibling].aabb);
    nodes[newParent].data = nullptr;
    nodes[newParent].parent = oldParent;

    // Connect new leaf and sibling to new parent
    nodes[newParent].child1 = leaf;
    nodes[newParent].child2 = bestSibling;
    nodes[leaf].parent = newParent;
    nodes[bestSibling].parent = newParent;

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
    }
    else
    {
        root = newParent;
    }

    // Walk back up the tree refitting ancestors' AABB and applying rotations
    NodeProxy ancestor = newParent;
    while (ancestor != nullNode)
    {
        NodeProxy child1 = nodes[ancestor].child1;
        NodeProxy child2 = nodes[ancestor].child2;

        nodes[ancestor].aabb = AABB::Union(nodes[child1].aabb, nodes[child2].aabb);

        Rotate(ancestor);

        ancestor = nodes[ancestor].parent;
    }

    return leaf;
}

void AABBTree::RemoveLeaf(NodeProxy leaf)
{
    muliAssert(0 <= leaf && leaf < nodeCapacity);
    muliAssert(nodes[leaf].IsLeaf());

    NodeProxy parent = nodes[leaf].parent;
    if (parent == nullNode) // node is root
    {
        muliAssert(root == leaf);
        root = nullNode;
        return;
    }

    NodeProxy grandParent = nodes[parent].parent;
    NodeProxy sibling;
    if (nodes[parent].child1 == leaf)
    {
        sibling = nodes[parent].child2;
    }
    else
    {
        sibling = nodes[parent].child1;
    }

    FreeNode(parent);

    if (grandParent != nullNode) // node has grandparent
    {
        nodes[sibling].parent = grandParent;

        if (nodes[grandParent].child1 == parent)
        {
            nodes[grandParent].child1 = sibling;
        }
        else
        {
            nodes[grandParent].child2 = sibling;
        }

        NodeProxy ancestor = grandParent;
        while (ancestor != nullNode)
        {
            NodeProxy child1 = nodes[ancestor].child1;
            NodeProxy child2 = nodes[ancestor].child2;

            nodes[ancestor].aabb = AABB::Union(nodes[child1].aabb, nodes[child2].aabb);

            Rotate(ancestor);

            ancestor = nodes[ancestor].parent;
        }
    }
    else // node has no grandparent
    {
        root = sibling;
        nodes[sibling].parent = nullNode;
    }
}

NodeProxy AABBTree::CreateNode(Data* data, const AABB& aabb)
{
    NodeProxy newNode = AllocateNode();

    // Fatten the aabb
    nodes[newNode].aabb.max = aabb.max + aabb_margin;
    nodes[newNode].aabb.min = aabb.min - aabb_margin;
    nodes[newNode].data = data;
    nodes[newNode].parent = nullNode;
    nodes[newNode].moved = true;

    InsertLeaf(newNode);

    return newNode;
}

bool AABBTree::MoveNode(NodeProxy node, AABB aabb, const Vec2& displacement, bool forceMove)
{
    muliAssert(0 <= node && node < nodeCapacity);
    muliAssert(nodes[node].IsLeaf());

    const AABB& treeAABB = nodes[node].aabb;
    if (treeAABB.Contains(aabb) && forceMove == false)
    {
        return false;
    }

    Vec2 d = displacement * aabb_multiplier;

    if (d.x > 0.0f)
    {
        aabb.max.x += d.x;
    }
    else
    {
        aabb.min.x += d.x;
    }

    if (d.y > 0.0f)
    {
        aabb.max.y += d.y;
    }
    else
    {
        aabb.min.y += d.y;
    }

    // Fatten the aabb
    aabb.max += aabb_margin;
    aabb.min -= aabb_margin;

    RemoveLeaf(node);

    nodes[node].aabb = aabb;

    InsertLeaf(node);

    nodes[node].moved = true;

    return true;
}

void AABBTree::RemoveNode(NodeProxy node)
{
    muliAssert(0 <= node && node < nodeCapacity);
    muliAssert(nodes[node].IsLeaf());

    RemoveLeaf(node);
    FreeNode(node);
}

void AABBTree::Rotate(NodeProxy node)
{
    if (nodes[node].IsLeaf())
    {
        return;
    }

    NodeProxy child1 = nodes[node].child1;
    NodeProxy child2 = nodes[node].child2;

    float costDiffs[4] = { 0.0f };

    if (nodes[child1].IsLeaf() == false)
    {
        float area1 = SAH(nodes[child1].aabb);
        costDiffs[0] = SAH(AABB::Union(nodes[nodes[child1].child1].aabb, nodes[child2].aabb)) - area1;
        costDiffs[1] = SAH(AABB::Union(nodes[nodes[child1].child2].aabb, nodes[child2].aabb)) - area1;
    }

    if (nodes[child2].IsLeaf() == false)
    {
        float area2 = SAH(nodes[child2].aabb);
        costDiffs[2] = SAH(AABB::Union(nodes[nodes[child2].child1].aabb, nodes[child1].aabb)) - area2;
        costDiffs[3] = SAH(AABB::Union(nodes[nodes[child2].child2].aabb, nodes[child1].aabb)) - area2;
    }

    int32 bestDiffIndex = 0;
    for (int32 i = 1; i < 4; ++i)
    {
        if (costDiffs[i] < costDiffs[bestDiffIndex])
        {
            bestDiffIndex = i;
        }
    }

    // Rotate only if it reduce the suface area
    if (costDiffs[bestDiffIndex] >= 0.0f)
    {
        return;
    }

    // printf("Tree rotation occurred: %d\n", bestDiffIndex);
    switch (bestDiffIndex)
    {
    case 0:
    {
        // Swap(child2, nodes[child1].child2);
        nodes[nodes[child1].child2].parent = node;
        nodes[node].child2 = nodes[child1].child2;

        nodes[child1].child2 = child2;
        nodes[child2].parent = child1;

        nodes[child1].aabb = AABB::Union(nodes[nodes[child1].child1].aabb, nodes[nodes[child1].child2].aabb);
    }
    break;
    case 1:
    {
        // Swap(child2, nodes[child1].child1);
        nodes[nodes[child1].child1].parent = node;
        nodes[node].child2 = nodes[child1].child1;

        nodes[child1].child1 = child2;
        nodes[child2].parent = child1;

        nodes[child1].aabb = AABB::Union(nodes[nodes[child1].child1].aabb, nodes[nodes[child1].child2].aabb);
    }
    break;
    case 2:
    {
        // Swap(child1, nodes[child2].child2);
        nodes[nodes[child2].child2].parent = node;
        nodes[node].child1 = nodes[child2].child2;

        nodes[child2].child2 = child1;
        nodes[child1].parent = child2;

        nodes[child2].aabb = AABB::Union(nodes[nodes[child2].child1].aabb, nodes[nodes[child2].child2].aabb);
    }
    break;
    case 3:
    {
        // Swap(child1, nodes[child2].child1);
        nodes[nodes[child2].child1].parent = node;
        nodes[node].child1 = nodes[child2].child1;

        nodes[child2].child1 = child1;
        nodes[child1].parent = child2;

        nodes[child2].aabb = AABB::Union(nodes[nodes[child2].child1].aabb, nodes[nodes[child2].child2].aabb);
    }
    break;
    }
}

void AABBTree::Swap(NodeProxy node1, NodeProxy node2)
{
    NodeProxy parent1 = nodes[node1].parent;
    NodeProxy parent2 = nodes[node2].parent;

    if (parent1 == parent2)
    {
        nodes[parent1].child1 = node2;
        nodes[parent1].child2 = node1;
        return;
    }

    if (nodes[parent1].child1 == node1)
    {
        nodes[parent1].child1 = node2;
    }
    else
    {
        nodes[parent1].child2 = node2;
    }
    nodes[node2].parent = parent1;

    if (nodes[parent2].child1 == node2)
    {
        nodes[parent2].child1 = node1;
    }
    else
    {
        nodes[parent2].child2 = node1;
    }
    nodes[node1].parent = parent2;
}

void AABBTree::Traverse(const std::function<void(const Node*)>& callback) const
{
    if (root == nullNode)
    {
        return;
    }

    GrowableArray<NodeProxy, 256> stack;
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
        callback(node);
    }
}

void AABBTree::Query(const Vec2& point, const std::function<bool(NodeProxy, Data*)>& callback) const
{
    if (root == nullNode)
    {
        return;
    }

    GrowableArray<NodeProxy, 256> stack;
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
            bool proceed = callback(current, nodes[current].data);
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

void AABBTree::Query(const AABB& aabb, const std::function<bool(NodeProxy, Data*)>& callback) const
{
    if (root == nullNode)
    {
        return;
    }

    GrowableArray<NodeProxy, 256> stack;
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
            bool proceed = callback(current, nodes[current].data);
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

void AABBTree::AABBCast(const AABBCastInput& input,
                        const std::function<float(const AABBCastInput& input, Data* data)>& callback) const
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

    GrowableArray<NodeProxy, 256> stack;
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

            float newFraction = callback(subInput, node->data);
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

void AABBTree::Reset()
{
    root = nullNode;
    nodeCount = 0;
    memset(nodes, 0, nodeCapacity * sizeof(Node));

    // Build a linked list for the free list.
    for (int32 i = 0; i < nodeCapacity - 1; ++i)
    {
        nodes[i].next = i + 1;
        nodes[i].parent = i;
    }
    nodes[nodeCapacity - 1].next = nullNode;
    nodes[nodeCapacity - 1].parent = nodeCapacity - 1;

    freeList = 0;
}

NodeProxy AABBTree::AllocateNode()
{
    if (freeList == nullNode)
    {
        muliAssert(nodeCount == nodeCapacity);

        // Grow the node pool
        Node* oldNodes = nodes;
        nodeCapacity += nodeCapacity / 2;
        nodes = (Node*)muli::Alloc(nodeCapacity * sizeof(Node));
        memcpy(nodes, oldNodes, nodeCount * sizeof(Node));
        memset(nodes + nodeCount, 0, (nodeCapacity - nodeCount) * sizeof(Node));
        muli::Free(oldNodes);

        // Build a linked list for the free list.
        for (int32 i = nodeCount; i < nodeCapacity - 1; ++i)
        {
            nodes[i].next = i + 1;
            nodes[i].parent = i;
        }
        nodes[nodeCapacity - 1].next = nullNode;
        nodes[nodeCapacity - 1].parent = nodeCapacity - 1;

        freeList = nodeCount;
    }

    NodeProxy node = freeList;
    freeList = nodes[node].next;
    nodes[node].parent = nullNode;
    nodes[node].child1 = nullNode;
    nodes[node].child2 = nullNode;
    nodes[node].moved = false;
    ++nodeCount;

    return node;
}

void AABBTree::FreeNode(NodeProxy node)
{
    muliAssert(0 <= node && node <= nodeCapacity);
    muliAssert(0 < nodeCount);

    nodes[node].parent = node;
    nodes[node].next = freeList;
    freeList = node;

    --nodeCount;
}

void AABBTree::Rebuild()
{
    // Rebuild tree with bottom up approach

    NodeProxy* leaves = (NodeProxy*)muli::Alloc(nodeCount * sizeof(NodeProxy));
    int32 count = 0;

    // Collect all leaves
    for (int32 i = 0; i < nodeCapacity; ++i)
    {
        // Already in the free list
        if (nodes[i].parent == i)
        {
            continue;
        }

        // Clean the leaf
        if (nodes[i].IsLeaf())
        {
            nodes[i].parent = nullNode;

            leaves[count++] = i;
        }
        else
        {
            // Free the internal node
            FreeNode(i);
        }
    }

    while (count > 1)
    {
        float minCost = max_value;
        int32 minI = -1;
        int32 minJ = -1;

        // Find the best aabb pair
        for (int32 i = 0; i < count; ++i)
        {
            AABB aabbI = nodes[leaves[i]].aabb;

            for (int32 j = i + 1; j < count; ++j)
            {
                AABB aabbJ = nodes[leaves[j]].aabb;

                AABB combined = AABB::Union(aabbI, aabbJ);
                float cost = SAH(combined);

                if (cost < minCost)
                {
                    minCost = cost;
                    minI = i;
                    minJ = j;
                }
            }
        }

        NodeProxy index1 = leaves[minI];
        NodeProxy index2 = leaves[minJ];
        Node* child1 = nodes + index1;
        Node* child2 = nodes + index2;

        // Create a parent(internal) node
        NodeProxy parentIndex = AllocateNode();
        Node* parent = nodes + parentIndex;

        parent->child1 = index1;
        parent->child2 = index2;
        parent->aabb = AABB::Union(child1->aabb, child2->aabb);
        parent->parent = nullNode;

        child1->parent = parentIndex;
        child2->parent = parentIndex;

        leaves[minI] = parentIndex;

        leaves[minJ] = leaves[count - 1];
        --count;
    }

    root = leaves[0];
    muli::Free(leaves);
}

} // namespace muli