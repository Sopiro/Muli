#include "muli/world.h"
#include "muli/island.h"
#include <iostream>

namespace muli
{

void World::Step(float dt)
{
    settings.DT = dt;
    settings.INV_DT = 1.0f / dt;

    contactManager.Update(dt);

    // Build the constraint island
    Island island{ *this };
    island.bodies.reserve(bodyCount);
    island.joints.reserve(jointCount);
    island.contacts.reserve(contactManager.contactCount);

    uint32 restingBodies = 0;
    uint32 islandID = 0;
    sleepingIslands = 0;
    sleepingBodies = 0;

    std::unordered_set<uint32> visited{};
    std::vector<RigidBody*> stack;
    visited.reserve(bodyCount);
    stack.reserve(bodyCount);

    // Perform a DFS(Depth First Search) on the constraint graph
    // After building island, each island can be solved in parallel because they are independent of each other
    for (RigidBody* b = bodyList; b; b = b->next)
    {
        if (b->type == RigidBody::Type::Static || (visited.find(b->id) != visited.end()))
        {
            continue;
        }

        stack.clear();
        stack.push_back(b);

        islandID++;
        while (stack.size() > 0)
        {
            RigidBody* t = stack.back();
            stack.pop_back();

            if (t->type == RigidBody::Type::Static || (visited.find(t->id) != visited.end()))
            {
                continue;
            }

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

            for (JointEdge* je = t->jointList; je; je = je->next)
            {
                if (je->other == t)
                {
                    island.joints.push_back(je->joint);
                    t->Awake();
                }

                if (visited.find(je->other->id) != visited.end())
                {
                    continue;
                }

                island.joints.push_back(je->joint);
                stack.push_back(je->other);
            }

            if (t->resting > settings.SLEEPING_TRESHOLD)
            {
                restingBodies++;
            }
        }

        island.sleeping = settings.SLEEPING && (restingBodies == island.bodies.size());

        if (island.sleeping)
        {
            sleepingBodies += static_cast<uint32>(island.bodies.size());
            sleepingIslands++;
        }

        island.Solve();
        island.Clear();
        restingBodies = 0;
    }

    numIslands = islandID;

    for (RigidBody* b : destroyBufferBody)
        Destroy(b);
    for (Joint* j : destroyBufferJoint)
        Destroy(j);
    destroyBufferBody.clear();
    destroyBufferJoint.clear();

    integrateForce = false;
}

void World::Reset()
{
    contactManager.Reset();

    RigidBody* b = bodyList;
    while (b)
    {
        RigidBody* b0 = b;
        b = b->next;
        delete b0;
    }

    Joint* j = jointList;
    while (j)
    {
        Joint* j0 = j;
        j = j->next;
        delete j0;
    }

    bodyList = nullptr;
    bodyListTail = nullptr;
    bodyCount = 0;

    jointList = nullptr;
    jointCount = 0;

    uid = 0;
}

void World::Add(RigidBody* body)
{
    if (body->world != nullptr)
    {
        throw std::runtime_error("This body is already registered.");
    }

    body->world = this;
    body->id = ++uid;

    if (bodyList == nullptr && bodyListTail == nullptr)
    {
        bodyList = body;
        bodyListTail = body;
    }
    else
    {
        bodyListTail->next = body;
        body->prev = bodyListTail;
        bodyListTail = body;
    }

    contactManager.Add(body);
    ++bodyCount;
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
    if (body->world != this)
    {
        throw std::runtime_error("This body is not registered in this world");
    }

    contactManager.Remove(body);

    JointEdge* je = body->jointList;
    while (je)
    {
        JointEdge* je0 = je;
        je = je->next;
        je0->other->Awake();

        Destroy(je0->joint);
    }

    contactManager.Remove(body);

    if (body->next) body->next->prev = body->prev;
    if (body->prev) body->prev->next = body->next;
    if (body == bodyList) bodyList = body->next;
    if (body == bodyListTail) bodyListTail = body->prev;

    delete body;
    --bodyCount;
}

void World::Destroy(const std::vector<RigidBody*>& bodies)
{
    for (uint32 i = 0; i < bodies.size(); i++)
    {
        Destroy(bodies[i]);
    }
}

void World::BufferDestroy(RigidBody* body)
{
    destroyBufferBody.push_back(body);
}

void World::BufferDestroy(const std::vector<RigidBody*>& bodies)
{
    for (uint32 i = 0; i < bodies.size(); i++)
    {
        BufferDestroy(bodies[i]);
    }
}

void World::Destroy(Joint* joint)
{
    RigidBody* bodyA = joint->bodyA;
    RigidBody* bodyB = joint->bodyB;

    // Remove from the world
    if (joint->prev) joint->prev->next = joint->next;
    if (joint->next) joint->next->prev = joint->prev;
    if (joint == jointList) jointList = joint->next;

    // Remove from bodyA
    if (joint->nodeA.prev) joint->nodeA.prev->next = joint->nodeA.next;
    if (joint->nodeA.next) joint->nodeA.next->prev = joint->nodeA.prev;
    if (&joint->nodeA == bodyA->jointList) bodyA->jointList = joint->nodeA.next;

    // Remove from bodyB
    if (joint->bodyA != joint->bodyB)
    {
        if (joint->nodeB.prev) joint->nodeB.prev->next = joint->nodeB.next;
        if (joint->nodeB.next) joint->nodeB.next->prev = joint->nodeB.prev;
        if (&joint->nodeB == bodyB->jointList) bodyB->jointList = joint->nodeB.next;
    }

    --jointCount;
    delete joint;
}

void World::Destroy(const std::vector<Joint*>& joints)
{
    for (uint32 i = 0; i < joints.size(); i++)
    {
        Destroy(joints[i]);
    }
}

void World::BufferDestroy(Joint* joint)
{
    destroyBufferJoint.push_back(joint);
}

void World::BufferDestroy(const std::vector<Joint*>& joints)
{
    for (uint32 i = 0; i < joints.size(); i++)
    {
        BufferDestroy(joints[i]);
    }
}

std::vector<RigidBody*> World::Query(const Vec2& point) const
{
    std::vector<RigidBody*> res;
    std::vector<Node*> nodes = contactManager.broadPhase.tree.Query(point);

    for (uint32 i = 0; i < nodes.size(); i++)
    {
        RigidBody* body = nodes[i]->body;

        if (TestPointInside(body, point))
        {
            res.push_back(body);
        }
    }

    return res;
}

std::vector<RigidBody*> World::Query(const AABB& aabb) const
{
    std::vector<RigidBody*> res;
    std::vector<Node*> nodes = contactManager.broadPhase.tree.Query(aabb);

    Polygon t{
        { aabb.min, { aabb.max.x, aabb.min.y }, aabb.max, { aabb.min.x, aabb.max.y } }, RigidBody::Type::Dynamic, false, 0.0f
    };

    for (uint32 i = 0; i < nodes.size(); i++)
    {
        RigidBody* body = nodes[i]->body;

        if (DetectCollision(body, &t))
        {
            res.push_back(body);
        }
    }

    return res;
}

Box* World::CreateBox(float width, float height, RigidBody::Type type, float radius, float density)
{
    Box* b = new Box(width, height, type, radius, density);
    Add(b);
    return b;
}

Box* World::CreateBox(float size, RigidBody::Type type, float radius, float density)
{
    return CreateBox(size, size, type, radius, density);
}

Circle* World::CreateCircle(float radius, RigidBody::Type type, float density)
{
    Circle* c = new Circle(radius, type, density);
    Add(c);
    return c;
}

Polygon* World::CreatePolygon(
    const std::vector<Vec2>& vertices, RigidBody::Type type, bool resetPosition, float radius, float density)
{
    Polygon* p = new Polygon(vertices, type, resetPosition, radius, density);
    Add(p);
    return p;
}

Polygon* World::CreateRandomConvexPolygon(float length, uint32 vertexCount, float radius, float density)
{
    if (vertexCount < 3)
    {
        vertexCount = LinearRand(3, 8);
    }

    std::vector<float> angles{};
    angles.reserve(vertexCount);

    for (uint32 i = 0; i < vertexCount; i++)
    {
        angles.push_back(LinearRand(0.0f, 1.0f) * (MULI_PI * 2.0f - FLT_EPSILON));
    }

    std::sort(angles.begin(), angles.end());

    std::vector<Vec2> vertices;
    vertices.reserve(vertexCount);

    for (uint32 i = 0; i < vertexCount; i++)
    {
        vertices.emplace_back(Cos(angles[i]) * length, Sin(angles[i]) * length);
    }

    Polygon* p = new Polygon(vertices, RigidBody::Type::Dynamic, true, radius, density);
    Add(p);
    return p;
}

Polygon* World::CreateRegularPolygon(float length, uint32 vertexCount, float initial_angle, float radius, float density)
{
    if (vertexCount < 3)
    {
        vertexCount = LinearRand(3, 11);
    }

    float angleStart = initial_angle;
    float angle = MULI_PI * 2.0f / vertexCount;

    if (vertexCount % 2 == 0)
    {
        angleStart += angle / 2.0f;
    }

    std::vector<Vec2> vertices;
    vertices.reserve(vertexCount);

    for (uint32 i = 0; i < vertexCount; i++)
    {
        float currentAngle = angleStart + angle * i;

        Vec2 corner = Vec2{ Cos(currentAngle), Sin(currentAngle) };
        corner *= length * Sqrt(2.0f);

        vertices.push_back(corner);
    }

    Polygon* p = new Polygon(vertices, RigidBody::Type::Dynamic, true, radius, density);
    Add(p);
    return p;
}

Capsule* World::CreateCapsule(float length, float radius, bool horizontal, RigidBody::Type type, float density)
{
    Capsule* c = new Capsule(length, radius, horizontal, type, density);
    Add(c);
    return c;
}
Capsule* World::CreateCapsule(
    const Vec2& p1, const Vec2& p2, float radius, bool resetPosition, RigidBody::Type type, float density)
{
    Capsule* c = new Capsule(p1, p2, radius, resetPosition, type, density);
    Add(c);
    return c;
}

GrabJoint* World::CreateGrabJoint(RigidBody* body, Vec2 anchor, Vec2 target, float frequency, float dampingRatio, float jointMass)
{
    if (body->world != this)
    {
        throw std::runtime_error("You should register the rigid bodies before registering the joint");
    }

    GrabJoint* gj = new GrabJoint(body, anchor, target, settings, frequency, dampingRatio, jointMass);

    Add(gj);
    return gj;
}

RevoluteJoint* World::CreateRevoluteJoint(
    RigidBody* bodyA, RigidBody* bodyB, Vec2 anchor, float frequency, float dampingRatio, float jointMass)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        throw std::runtime_error("You should register the rigid bodies before registering the joint");
    }

    RevoluteJoint* rj = new RevoluteJoint(bodyA, bodyB, anchor, settings, frequency, dampingRatio, jointMass);

    Add(rj);
    return rj;
}

DistanceJoint* World::CreateDistanceJoint(RigidBody* bodyA,
                                          RigidBody* bodyB,
                                          Vec2 anchorA,
                                          Vec2 anchorB,
                                          float length,
                                          float frequency,
                                          float dampingRatio,
                                          float jointMass)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        throw std::runtime_error("You should register the rigid bodies before registering the joint");
    }

    DistanceJoint* dj = new DistanceJoint(bodyA, bodyB, anchorA, anchorB, length, settings, frequency, dampingRatio, jointMass);

    Add(dj);
    return dj;
}

DistanceJoint* World::CreateDistanceJoint(
    RigidBody* bodyA, RigidBody* bodyB, float length, float frequency, float dampingRatio, float jointMass)
{
    return CreateDistanceJoint(bodyA, bodyB, bodyA->GetPosition(), bodyB->GetPosition(), length, frequency, dampingRatio,
                               jointMass);
}

AngleJoint* World::CreateAngleJoint(RigidBody* _bodyA, RigidBody* _bodyB, float _frequency, float _dampingRatio, float _jointMass)
{
    AngleJoint* aj = new AngleJoint(_bodyA, _bodyB, settings, _frequency, _dampingRatio, _jointMass);

    Add(aj);
    return aj;
}

void World::Add(Joint* joint)
{
    // Insert into the world
    joint->prev = nullptr;
    joint->next = jointList;
    if (jointList != nullptr)
    {
        jointList->prev = joint;
    }
    jointList = joint;

    // Connect to island graph

    // Connect joint edge to body A
    joint->nodeA.joint = joint;
    joint->nodeA.other = joint->bodyB;

    joint->nodeA.prev = nullptr;
    joint->nodeA.next = joint->bodyA->jointList;
    if (joint->bodyA->jointList != nullptr)
    {
        joint->bodyA->jointList->prev = &joint->nodeA;
    }
    joint->bodyA->jointList = &joint->nodeA;

    // Connect joint edge to body B
    if (joint->bodyA != joint->bodyB)
    {
        joint->nodeB.joint = joint;
        joint->nodeB.other = joint->bodyA;

        joint->nodeB.prev = nullptr;
        joint->nodeB.next = joint->bodyB->jointList;
        if (joint->bodyB->jointList != nullptr)
        {
            joint->bodyB->jointList->prev = &joint->nodeB;
        }
        joint->bodyB->jointList = &joint->nodeB;
    }

    ++jointCount;
}

} // namespace muli