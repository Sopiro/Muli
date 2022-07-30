#pragma once

#include "../common.h"
#include "world.h"

namespace spe
{
class Island
{
    friend class World;

private:
    World& world;
    bool sleeping{ false };
    std::vector<RigidBody*> bodies{};

    std::vector<ContactConstraint*> ccs{};

    Island(World& _world);
    void Solve(float dt);
    void Clear();
};
}