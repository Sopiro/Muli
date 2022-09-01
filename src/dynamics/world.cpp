#include "spe/world.h"
#include "spe/island.h"
#include <iostream>

namespace spe
{

void World::Step(float dt)
{
    settings.DT = dt;
    settings.INV_DT = 1.0f / dt;

    contactManager.Update(dt);

    // Build the constraint island
    Island island{ *this };
    island.bodies.reserve(bodies.size());
    island.contacts.reserve(contactManager.contactCount);
    island.joints.reserve(joints.size());
    uint32_t restingBodies = 0;
    uint32_t islandID = 0;
    sleepingIslands = 0;
    sleepingBodies = 0;

    std::unordered_set<uint32_t> visited{};
    std::vector<RigidBody*> stack;
    visited.reserve(bodies.size());
    stack.reserve(bodies.size());

    // Perform a DFS(Depth First Search) on the constraint graph
    // After building island, each island can be solved in parallel because they are independent of each other
    for (uint32_t i = 0; i < bodies.size(); i++)
    {
        RigidBody* b = bodies[i];

        if (b->type == RigidBody::Type::Static || (visited.find(b->id) != visited.end())) continue;

        stack.clear();
        stack.push_back(b);

        islandID++;
        while (stack.size() > 0)
        {
            RigidBody* t = stack.back();
            stack.pop_back();

            if (t->type == RigidBody::Type::Static || (visited.find(t->id) != visited.end())) continue;

            visited.insert(t->id);
            t->islandID = islandID;
            island.bodies.push_back(t);

            for (ContactEdge* ce = t->contactList; ce; ce = ce->next)
            {
                if (visited.find(ce->other->id) != visited.end())
                {
                    continue;
                }

                if (ce->contact->touching)
                {
                    island.contacts.push_back(ce->contact);
                    stack.push_back(ce->other);
                }
            }

            for (uint32_t j = 0; j < t->joints.size(); j++)
            {
                Joint* joint = t->joints[j];

                RigidBody* other = joint->bodyB->id == t->id ? joint->bodyA : joint->bodyB;

                if (joint->bodyA == joint->bodyB)
                {
                    island.joints.push_back(joint);
                    t->Awake();
                }

                if (visited.find(other->id) != visited.end())
                {
                    continue;
                }

                island.joints.push_back(joint);
                stack.push_back(other);
            }

            if (t->resting > settings.SLEEPING_TRESHOLD) restingBodies++;
        }

        island.sleeping = settings.SLEEPING_ENABLED && (restingBodies == island.bodies.size());

        if (island.sleeping)
        {
            sleepingBodies += static_cast<uint32_t>(island.bodies.size());
            sleepingIslands++;
        }

        island.Solve();
        island.Clear();
        restingBodies = 0;
    }

    numIslands = islandID;
}

void World::Reset()
{
    contactManager.Reset();

    for (RigidBody* body : bodies)
        delete body;
    for (Joint* joint : joints)
        delete joint;

    bodies.clear();
    joints.clear();

    uid = 0;
}

void World::Add(RigidBody* body)
{
    if (body->world != nullptr) throw std::exception("This body is already registered.");

    body->world = this;
    body->id = ++uid;
    bodies.push_back(body);
    contactManager.Add(body);
}

void World::Add(const std::vector<RigidBody*>& bodies)
{
    for (auto b : bodies)
    {
        Add(b);
    }
}

void World::Destroy(RigidBody* body)
{
    auto it = std::find(bodies.begin(), bodies.end(), body);
    if (it == bodies.end()) throw std::exception("This body is not registered in this world.");

    contactManager.Remove(body);

    for (uint32_t i = 0; i < body->joints.size(); i++)
    {
        Joint* joint = body->joints[i];
        RigidBody* other = joint->bodyB->id == body->id ? joint->bodyA : joint->bodyB;

        other->Awake();

        auto it = std::find(other->joints.begin(), other->joints.end(), joint);
        if (it != other->joints.end())
        {
            std::iter_swap(it, other->joints.end() - 1);
            other->joints.pop_back();
        }

        delete joint;
    }

    joints.clear();

    contactManager.Remove(body);
    bodies.erase(it);

    delete body;
}

void World::Destroy(const std::vector<RigidBody*>& bodies)
{
    for (uint32_t i = 0; i < bodies.size(); i++)
    {
        Destroy(bodies[i]);
    }
}

void World::Destroy(Joint* joint)
{
    if (std::find(joints.begin(), joints.end(), joint) == joints.end())
        throw std::exception("This joint is not registered in this world.");

    auto it = std::find(joint->bodyA->joints.begin(), joint->bodyA->joints.end(), joint);
    if (it != joint->bodyA->joints.end())
    {
        std::iter_swap(it, joint->bodyA->joints.end() - 1);
        joint->bodyA->joints.pop_back();
    }
    joint->bodyA->Awake();

    it = std::find(joint->bodyB->joints.begin(), joint->bodyB->joints.end(), joint);
    if (it != joint->bodyB->joints.end())
    {
        std::iter_swap(it, joint->bodyB->joints.end() - 1);
        joint->bodyB->joints.pop_back();
    }
    joint->bodyB->Awake();

    auto jit = std::find(joints.begin(), joints.end(), joint);
    if (jit != joints.end())
    {
        std::iter_swap(jit, joints.end() - 1);
        joints.pop_back();
    }

    delete joint;
}

void World::Destroy(const std::vector<Joint*>& joints)
{
    for (uint32_t i = 0; i < joints.size(); i++)
    {
        Destroy(joints[i]);
    }
}

std::vector<RigidBody*> World::Query(const glm::vec2& point) const
{
    std::vector<RigidBody*> res;
    std::vector<Node*> nodes = contactManager.broadPhase.tree.Query(point);

    for (uint32_t i = 0; i < nodes.size(); i++)
    {
        RigidBody* body = nodes[i]->body;

        if (test_point_inside(body, point))
        {
            res.push_back(body);
        }
    }

    return res;
}

std::vector<RigidBody*> World::Query(const AABB& region) const
{
    std::vector<RigidBody*> res;
    std::vector<Node*> nodes = contactManager.broadPhase.tree.Query(region);

    for (uint32_t i = 0; i < nodes.size(); i++)
    {
        RigidBody* body = nodes[i]->body;

        float w = region.max.x - region.min.x;
        float h = region.max.y - region.min.y;

        Polygon t{ { region.min, { region.max.x, region.min.y }, region.max, { region.min.x, region.max.y } },
                   RigidBody::Type::Dynamic,
                   false };

        if (detect_collision(body, &t))
        {
            res.push_back(body);
        }
    }

    return res;
}

Box* World::CreateBox(float size, RigidBody::Type type, float density)
{
    return CreateBox(size, size, type, density);
}

Box* World::CreateBox(float width, float height, RigidBody::Type type, float density)
{
    Box* b = new Box(width, height, type, density);
    Add(b);
    return b;
}

Circle* World::CreateCircle(float radius, RigidBody::Type type, float density)
{
    Circle* c = new Circle(radius, type, density);
    Add(c);
    return c;
}

spe::Polygon* World::CreatePolygon(std::vector<glm::vec2> vertices, RigidBody::Type type, bool resetPosition, float density)
{
    Polygon* p = new Polygon(std::move(vertices), type, resetPosition, density);
    Add(p);
    return p;
}

Polygon* World::CreateRandomConvexPolygon(float radius, uint32_t num_vertices, float density)
{
    if (num_vertices < 3) num_vertices = glm::linearRand<uint32_t>(3, 8);

    std::vector<float> angles{};
    angles.reserve(num_vertices);

    for (uint32_t i = 0; i < num_vertices; i++)
    {
        angles.push_back(glm::linearRand<float>(0.0f, 1.0f) * glm::pi<float>() * 2.0f);
    }

    std::sort(angles.begin(), angles.end());

    std::vector<glm::vec2> vertices{};
    vertices.reserve(num_vertices);

    for (uint32_t i = 0; i < num_vertices; i++)
    {
        vertices.emplace_back(glm::cos(angles[i]) * radius, glm::sin(angles[i]) * radius);
    }

    Polygon* p = new Polygon(vertices, RigidBody::Type::Dynamic, true, density);
    Add(p);
    return p;
}

Polygon* World::CreateRegularPolygon(float radius, uint32_t num_vertices, float initial_angle, float density)
{
    if (num_vertices < 3) num_vertices = glm::linearRand<uint32_t>(3, 11);

    float angleStart = initial_angle;
    float angle = glm::pi<float>() * 2.0f / num_vertices;

    if (num_vertices % 2 == 0) angleStart += angle / 2.0f;

    std::vector<glm::vec2> vertices;
    vertices.reserve(num_vertices);

    for (uint32_t i = 0; i < num_vertices; i++)
    {
        float currentAngle = angleStart + angle * i;

        glm::vec2 corner = glm::vec2{ glm::cos(currentAngle), glm::sin(currentAngle) };
        corner *= radius * glm::sqrt(2);

        vertices.push_back(corner);
    }

    Polygon* p = new Polygon(vertices, RigidBody::Type::Dynamic, true, density);
    Add(p);
    return p;
}

GrabJoint* World::CreateGrabJoint(
    RigidBody* body, glm::vec2 anchor, glm::vec2 target, float frequency, float dampingRatio, float jointMass)
{
    if (body->world != this) throw std::exception("You should register the rigid bodies before registering the joint");

    GrabJoint* gj = new GrabJoint(body, anchor, target, settings, frequency, dampingRatio, jointMass);

    joints.push_back(gj);
    body->joints.push_back(gj);

    return gj;
}

RevoluteJoint* World::CreateRevoluteJoint(
    RigidBody* bodyA, RigidBody* bodyB, glm::vec2 anchor, float frequency, float dampingRatio, float jointMass)
{
    if (bodyA->world != this || bodyB->world != this)
        throw std::exception("You should register the rigid bodies before registering the joint");

    RevoluteJoint* rj = new RevoluteJoint(bodyA, bodyB, anchor, settings, frequency, dampingRatio, jointMass);

    joints.push_back(rj);
    bodyA->joints.push_back(rj);
    bodyB->joints.push_back(rj);

    return rj;
}

DistanceJoint* World::CreateDistanceJoint(RigidBody* bodyA,
                                          RigidBody* bodyB,
                                          glm::vec2 anchorA,
                                          glm::vec2 anchorB,
                                          float length,
                                          float frequency,
                                          float dampingRatio,
                                          float jointMass)
{
    if (bodyA->world != this || bodyB->world != this)
        throw std::exception("You should register the rigid bodies before registering the joint");

    DistanceJoint* dj = new DistanceJoint(bodyA, bodyB, anchorA, anchorB, length, settings, frequency, dampingRatio, jointMass);

    joints.push_back(dj);
    bodyA->joints.push_back(dj);
    bodyB->joints.push_back(dj);

    return dj;
}

DistanceJoint* World::CreateDistanceJoint(
    RigidBody* bodyA, RigidBody* bodyB, float length, float frequency, float dampingRatio, float jointMass)
{
    return CreateDistanceJoint(bodyA, bodyB, bodyA->position, bodyB->position, length, frequency, dampingRatio, jointMass);
}

} // namespace spe