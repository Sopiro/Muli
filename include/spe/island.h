#pragma once

#include "common.h"
#include "world.h"

namespace spe
{

class Island
{
    friend class World;

private:
    World& world;
    bool sleeping = false;

    std::vector<RigidBody*> bodies{};

    std::vector<ContactConstraint*> ccs{};
    std::vector<Joint*> js{};

    Island(World& _world);
    void Solve();
    void Clear();
};

} // namespace spe