#pragma once

#include "world.h"

namespace muli
{

class Island
{
    friend class World;

private:
    World& world;
    bool sleeping;

    // Static body is not included
    std::vector<RigidBody*> bodies;

    std::vector<Contact*> contacts;
    std::vector<Joint*> joints;

    Island(World& _world);
    void Solve();
    void Clear();
};

inline Island::Island(World& _world)
    : world{ _world }
    , sleeping{ false }
{
}

inline void Island::Clear()
{
    bodies.clear();
    contacts.clear();
    joints.clear();

    sleeping = false;
}

} // namespace muli