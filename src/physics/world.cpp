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

void World::Unregister(RigidBody* body)
{
    auto it = std::find(bodies.begin(), bodies.end(), body);

    if (it != bodies.end())
    {
        bodies.erase(it);
        tree.Remove(body);
    }
}

void World::Unregister(const std::vector<RigidBody*>& bodies)
{
    for(size_t i = 0; i < bodies.size(); i++)
    {
        Unregister(bodies[i]);
    }
}

std::vector<RigidBody*> World::QueryPoint(const glm::vec2& point)
{
    std::vector<RigidBody*> res;
    std::vector<Node*> nodes = tree.QueryPoint(point);

    for (size_t i = 0; i < nodes.size(); i++)
    {
        RigidBody* body = nodes[i]->body;

        if (test_point_inside(body, point))
        {
            res.push_back(body);
        }
    }

    return res;
}
