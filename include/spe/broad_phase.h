#pragma once

#include "common.h"
#include "aabb.h"
#include "aabbtree.h"

namespace spe
{
class BroadPhase
{
    friend class World;

public:
    BroadPhase(World& _world);
    void Update(float dt);
    void Reset();
    void Add(RigidBody* body);
    void Remove(RigidBody* body);

private:
    World& world;
    float margin = 0.1f;
    float velocityMultiplier = 3.0f;
    AABBTree tree;
    std::unordered_set<uint64_t> pairs{};
};

}