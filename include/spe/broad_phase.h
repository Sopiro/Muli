#pragma once

#include "aabb.h"
#include "aabbtree.h"
#include "common.h"

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
    std::set<uint64_t> pairs{};
};

} // namespace spe