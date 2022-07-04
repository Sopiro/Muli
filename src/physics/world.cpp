#include "world.h"

using namespace spe;

World::World(const Settings& simulationSettings) :
    settings{ simulationSettings }
{
    bodies.reserve(256);
}

World::~World() noexcept
{
}

void World::Update()
{
    std::vector<std::unique_ptr<ContactConstraint>> newContactConstraints{};
    std::unordered_map<int32_t, ContactConstraint*> newContactConstraintMap{};
    newContactConstraints.reserve(contactConstraints.size());
    newContactConstraintMap.reserve(newContactConstraintMap.size());

    for (size_t i = 0; i < bodies.size(); i++)
    {
        RigidBody* b = bodies[i];
        b->manifoldIDs.clear();
        if (b->type != Static)
            b->linearVelocity += settings.GRAVITY * settings.DT;

        if (b->sleeping) continue;

        Node* node = b->node;
        AABB tightAABB = create_AABB(b, 0.0f);

        if (contains_AABB(node->aabb, tightAABB)) continue;

        tree.Remove(b);
        tree.Add(b);
    }

    // Broad Phase
    // Retrieve a list of collider pairs that are potentially colliding
    // std::vector<std::pair<RigidBody*, RigidBody*>> pairs = get_collision_pair_n2(bodies);
    std::vector<std::pair<RigidBody*, RigidBody*>> pairs = tree.GetCollisionPairs();

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

        ContactConstraint* cc = new ContactConstraint(std::move(newManifold.value()), settings);
        newContactConstraints.emplace_back(cc);
        newContactConstraintMap.insert({ key, cc });

        a->manifoldIDs.push_back(key);
        b->manifoldIDs.push_back(key);

        auto it = contactConstraintMap.find(key);
        if (settings.WARM_STARTING && (it != contactConstraintMap.end()))
        {
            ContactConstraint* oldCC = it->second;
            cc->TryWarmStart(*oldCC);
        }
    }

    contactConstraintMap = std::move(newContactConstraintMap);
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

        b->position += b->linearVelocity * settings.DT;
        b->rotation += b->angularVelocity * settings.DT;
    }
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

const std::vector<std::unique_ptr<ContactConstraint>>& World::GetContactConstraints() const
{
    return contactConstraints;
}