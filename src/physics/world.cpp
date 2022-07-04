#include "world.h"

using namespace spe;

World::World()
{
    bodies.reserve(100);
}

World::~World() noexcept
{
    for (size_t i = 0; i < contactConstraints.size(); i++)
    {
        delete contactConstraints[i];
    }
}

void World::Update(float inv_dt)
{
    // spdlog::stopwatch sw;

    std::vector<ContactConstraint*> newContactConstraints{};
    std::unordered_map<int32_t, ContactConstraint*> newContactConstraintMap{};
    newContactConstraints.reserve(contactConstraints.size());
    newContactConstraintMap.reserve(newContactConstraintMap.size());

    for (size_t i = 0; i < bodies.size(); i++)
    {
        RigidBody* b = bodies[i];
        b->manifoldIDs.clear();
        if (b->type != Static)
            b->linearVelocity.y -= 10.0f * DT;

        if (b->sleeping) continue;

        Node* node = b->node;
        AABB tightAABB = create_AABB(b, 0.0f);

        if (contains_AABB(node->aabb, tightAABB)) continue;

        tree.Remove(b);
        tree.Add(b);
    }

    // Broad Phase
    // Retrieve a list of collider pairs that are potentially colliding
    std::vector<std::pair<RigidBody*, RigidBody*>> pairs = tree.GetCollisionPairs();
    // std::vector<std::pair<RigidBody*, RigidBody*>> pairs = get_collision_pair_n2(bodies);

    for (size_t i = 0; i < pairs.size(); i++)
    {
        std::pair<RigidBody*, RigidBody*> pair = pairs[i];
        RigidBody* a = pair.first;
        RigidBody* b = pair.second;

        // Improve coherence
        if (a->id > b->id)
        {
            a = pair.second;
            b = pair.first;
        }

        if (a->type == Static && b->type == Static)
            continue;

        uint32_t key = make_pair_natural(a->id, b->id);
        if (passTestSet.find(key) != passTestSet.end()) continue;

        // Narrow Phase
        // Execute more accurate and expensive collision detection
        std::optional<ContactManifold> newManifold = detect_collision(a, b);
        if (!newManifold.has_value()) continue;

        ContactConstraint* cc = new ContactConstraint(std::move(newManifold.value()));
        newContactConstraints.push_back(cc);

        a->manifoldIDs.push_back(key);
        b->manifoldIDs.push_back(key);

        auto it = contactConstraintMap.find(key);
        if (WARM_START && (it != contactConstraintMap.end()))
        {
            ContactConstraint* oldCC = it->second;
            cc->TryWarmStart(*oldCC);
        }

        newContactConstraintMap.insert({ key, cc });
    }

    contactConstraintMap = std::move(newContactConstraintMap);

    for (size_t i = 0; i < contactConstraints.size(); i++)
    {
        delete contactConstraints[i];
    }

    contactConstraints = std::move(newContactConstraints);

    for (size_t i = 0; i < contactConstraints.size(); i++)
    {
        contactConstraints[i]->Prepare();
    }

    for (size_t i = 0; i < 10; i++)
    {
        for (size_t j = 0; j < contactConstraints.size(); j++)
        {
            contactConstraints[j]->Solve();
        }
    }

    for (size_t i = 0; i < bodies.size(); i++)
    {
        RigidBody* b = bodies[i];

        if (b->type == Static)
        {
            glm::clear(b->linearVelocity);
            b->rotation = 0.0f;
            continue;
        }

        b->position += b->linearVelocity * DT;
        b->rotation += b->angularVelocity * DT;
    }

    // spdlog::info("Elapsed {}", sw);
}

void World::Reset()
{
    tree.Reset();
    bodies.clear();
}

void World::Register(RigidBody* body)
{
    body->id = uid++;
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
    for (size_t i = 0; i < bodies.size(); i++)
    {
        Unregister(bodies[i]);
    }
}

std::vector<RigidBody*> World::QueryPoint(const glm::vec2& point) const
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

std::vector<RigidBody*> World::QueryRegion(const AABB& region) const
{
    std::vector<RigidBody*> res;
    std::vector<Node*> nodes = tree.QueryRegion(region);

    for (size_t i = 0; i < nodes.size(); i++)
    {
        RigidBody* body = nodes[i]->body;

        float w = region.max.x - region.min.x;
        float h = region.max.y - region.min.y;

        Polygon t{ {region.min, {region.max.x, region.min.y}, region.max, {region.min.x, region.max.y}}, Dynamic, false };

        if (detect_collision(body, &t))
        {
            res.push_back(body);
        }
    }

    return res;
}

const std::vector<RigidBody*>& World::GetBodies() const
{
    return bodies;
}

const AABBTree& World::GetBVH() const
{
    return tree;
}

const std::vector<ContactConstraint*>& World::GetContactConstraints() const
{
    return contactConstraints;
}