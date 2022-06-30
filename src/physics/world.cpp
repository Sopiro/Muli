#include "world.h"

using namespace spe;

World::World()
{
    bodies.reserve(100);
}

World::~World() noexcept
{
}

void World::Update(float inv_dt)
{
    for (RigidBody* b : bodies)
    {
    }
}

void World::Reset()
{
    tree.Reset();
    bodies.clear();
}

void World::Register(RigidBody* body)
{
    bodies.push_back(body);
    tree.Add(body);
}

void World::Register(const std::vector<RigidBody*>& bodies)
{
    for (auto b : bodies)
    {
        Register(b);
    }
}
