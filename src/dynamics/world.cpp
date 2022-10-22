#include "muli/world.h"
#include "muli/island.h"
#include <iostream>

namespace muli
{

World::World(const WorldSettings& simulationSettings)
    : settings{ simulationSettings }
    , contactManager{ *this }
{
}

World::~World() noexcept
{
    Reset();
}

void World::Step(float dt)
{
    settings.DT = dt;
    settings.INV_DT = 1.0f / dt;

    contactManager.Update(dt);

    // Build the constraint island
    Island island{ *this, bodyCount, contactManager.contactCount, jointCount };

    uint32 restingBodies = 0;
    uint32 islandID = 0;
    sleepingIslands = 0;
    sleepingBodies = 0;

    // Use stack allocator to avoid per-frame allocation
    RigidBody** stack = (RigidBody**)stackAllocator.Allocate(bodyCount * sizeof(RigidBody*));
    uint32 stackPointer;

    // Perform a DFS(Depth First Search) on the constraint graph
    // After building island, each island can be solved in parallel because they are independent of each other
    for (RigidBody* b = bodyList; b; b = b->next)
    {
        if (b->type == RigidBody::Type::Static)
        {
            continue;
        }

        if (b->flag & RigidBody::Flag::FlagSleeping || b->flag & RigidBody::Flag::FlagIsland)
        {
            continue;
        }

        stackPointer = 0;
        stack[stackPointer++] = b;

        ++islandID;
        while (stackPointer > 0)
        {
            RigidBody* t = stack[--stackPointer];

            if (t->type == RigidBody::Type::Static || (t->flag & RigidBody::Flag::FlagIsland))
            {
                continue;
            }

            t->flag |= RigidBody::Flag::FlagIsland;
            t->islandID = islandID;
            island.Add(t);

            for (ContactEdge* ce = t->contactList; ce; ce = ce->next)
            {
                if (ce->other->flag & RigidBody::Flag::FlagIsland)
                {
                    continue;
                }

                if (ce->contact->touching)
                {
                    island.Add(ce->contact);
                    stack[stackPointer++] = ce->other;
                }
            }

            for (JointEdge* je = t->jointList; je; je = je->next)
            {
                if (je->other == t)
                {
                    island.Add(je->joint);
                    t->Awake();
                }

                if (je->other->flag & RigidBody::Flag::FlagIsland)
                {
                    continue;
                }

                island.Add(je->joint);
                stack[stackPointer++] = je->other;
            }

            if (t->resting > settings.SLEEPING_TRESHOLD)
            {
                restingBodies++;
            }
        }

        island.sleeping = settings.SLEEPING && (restingBodies == island.bodyCount);

        if (island.sleeping)
        {
            sleepingBodies += island.bodyCount;
            sleepingIslands++;
        }

        island.Solve();
        island.Clear();
        restingBodies = 0;
    }

    stackAllocator.Free(stack);

    numIslands = islandID;

    for (RigidBody* b : destroyBufferBody)
    {
        Destroy(b);
    }
    for (Joint* j : destroyBufferJoint)
    {
        Destroy(j);
    }

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
        FreeBody(b0);
    }

    Joint* j = jointList;
    while (j)
    {
        Joint* j0 = j;
        j = j->next;
        FreeJoint(j0);
    }

    bodyList = nullptr;
    bodyListTail = nullptr;
    bodyCount = 0;

    jointList = nullptr;
    jointCount = 0;

    uid = 0;

    muliAssert(blockAllocator.GetBlockCount() == 0);
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

    FreeBody(body);
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

    FreeJoint(joint);
    --jointCount;
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
    std::vector<RigidBody*> bodies = contactManager.broadPhase.tree.Query(point);

    for (uint32 i = 0; i < bodies.size(); i++)
    {
        RigidBody* body = bodies[i];

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
    std::vector<RigidBody*> bodies = contactManager.broadPhase.tree.Query(aabb);

    Vec2 box[4] = { aabb.min, { aabb.max.x, aabb.min.y }, aabb.max, { aabb.min.x, aabb.max.y } };
    Polygon t{ box, 4, RigidBody::Type::Dynamic, false, 0.0f };

    for (uint32 i = 0; i < bodies.size(); i++)
    {
        RigidBody* body = bodies[i];

        if (DetectCollision(body, &t))
        {
            res.push_back(body);
        }
    }

    return res;
}

Polygon* World::CreateBox(float width, float height, RigidBody::Type type, float radius, float density)
{
    Vec2 box[4] = { Vec2{ 0, 0 }, Vec2{ width, 0 }, Vec2{ width, height }, Vec2{ 0, height } };
    void* mem = blockAllocator.Allocate(sizeof(Polygon));
    Polygon* b = new (mem) Polygon(box, 4, type, true, radius, density);
    Add(b);
    return b;
}

Polygon* World::CreateBox(float size, RigidBody::Type type, float radius, float density)
{
    return CreateBox(size, size, type, radius, density);
}

Circle* World::CreateCircle(float radius, RigidBody::Type type, float density)
{
    void* mem = blockAllocator.Allocate(sizeof(Circle));
    Circle* c = new (mem) Circle(radius, type, density);
    Add(c);
    return c;
}

Polygon* World::CreatePolygon(
    const std::vector<Vec2>& vertices, RigidBody::Type type, bool resetPosition, float radius, float density)
{
    void* mem = blockAllocator.Allocate(sizeof(Polygon));
    Polygon* p = new Polygon(vertices.data(), static_cast<int32>(vertices.size()), type, resetPosition, radius, density);
    Add(p);
    return p;
}

Polygon* World::CreateRandomConvexPolygon(float length, int32 vertexCount, float radius, float density)
{
    if (vertexCount < 3)
    {
        vertexCount = LinearRand(6, 12);
    }

    std::vector<float> angles;
    angles.reserve(vertexCount);

    for (int32 i = 0; i < vertexCount; i++)
    {
        angles.push_back(LinearRand(0.0f, 1.0f) * (MULI_PI * 2.0f - FLT_EPSILON));
    }

    std::sort(angles.begin(), angles.end());

    std::vector<Vec2> vertices;
    vertices.reserve(vertexCount);

    for (int32 i = 0; i < vertexCount; i++)
    {
        vertices.emplace_back(Cos(angles[i]) * length, Sin(angles[i]) * length);
    }

    void* mem = blockAllocator.Allocate(sizeof(Polygon));
    Polygon* p = new (mem) Polygon(vertices.data(), vertexCount, RigidBody::Type::Dynamic, true, radius, density);
    Add(p);
    return p;
}

Polygon* World::CreateRegularPolygon(float length, int32 vertexCount, float initial_angle, float radius, float density)
{
    if (vertexCount < 3)
    {
        vertexCount = LinearRand(3, 12);
    }

    float angleStart = initial_angle;
    float angle = MULI_PI * 2.0f / vertexCount;

    if (vertexCount % 2 == 0)
    {
        angleStart += angle / 2.0f;
    }

    std::vector<Vec2> vertices;
    vertices.reserve(vertexCount);

    for (int32 i = 0; i < vertexCount; i++)
    {
        float currentAngle = angleStart + angle * i;

        Vec2 corner = Vec2{ Cos(currentAngle), Sin(currentAngle) };
        corner *= length * Sqrt(2.0f);

        vertices.push_back(corner);
    }

    void* mem = blockAllocator.Allocate(sizeof(Polygon));
    Polygon* p = new (mem) Polygon(vertices.data(), vertexCount, RigidBody::Type::Dynamic, true, radius, density);
    Add(p);
    return p;
}

Capsule* World::CreateCapsule(float length, float radius, bool horizontal, RigidBody::Type type, float density)
{
    void* mem = blockAllocator.Allocate(sizeof(Capsule));
    Capsule* c = new (mem) Capsule(length, radius, horizontal, type, density);
    Add(c);
    return c;
}
Capsule* World::CreateCapsule(
    const Vec2& p1, const Vec2& p2, float radius, bool resetPosition, RigidBody::Type type, float density)
{
    void* mem = blockAllocator.Allocate(sizeof(Capsule));
    Capsule* c = new (mem) Capsule(p1, p2, radius, resetPosition, type, density);
    Add(c);
    return c;
}

GrabJoint* World::CreateGrabJoint(
    RigidBody* body, const Vec2& anchor, const Vec2& target, float frequency, float dampingRatio, float jointMass)
{
    if (body->world != this)
    {
        throw std::runtime_error("You should register the rigid bodies before registering the joint");
    }

    void* mem = blockAllocator.Allocate(sizeof(GrabJoint));
    GrabJoint* gj = new (mem) GrabJoint(body, anchor, target, settings, frequency, dampingRatio, jointMass);

    Add(gj);
    return gj;
}

RevoluteJoint* World::CreateRevoluteJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec2& anchor, float frequency, float dampingRatio, float jointMass)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        throw std::runtime_error("You should register the rigid bodies before registering the joint");
    }

    void* mem = blockAllocator.Allocate(sizeof(RevoluteJoint));
    RevoluteJoint* rj = new (mem) RevoluteJoint(bodyA, bodyB, anchor, settings, frequency, dampingRatio, jointMass);

    Add(rj);
    return rj;
}

DistanceJoint* World::CreateDistanceJoint(RigidBody* bodyA,
                                          RigidBody* bodyB,
                                          const Vec2& anchorA,
                                          const Vec2& anchorB,
                                          float length,
                                          float frequency,
                                          float dampingRatio,
                                          float jointMass)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        throw std::runtime_error("You should register the rigid bodies before registering the joint");
    }

    void* mem = blockAllocator.Allocate(sizeof(DistanceJoint));
    DistanceJoint* dj =
        new (mem) DistanceJoint(bodyA, bodyB, anchorA, anchorB, length, settings, frequency, dampingRatio, jointMass);

    Add(dj);
    return dj;
}

DistanceJoint* World::CreateDistanceJoint(
    RigidBody* bodyA, RigidBody* bodyB, float length, float frequency, float dampingRatio, float jointMass)
{
    return CreateDistanceJoint(bodyA, bodyB, bodyA->GetPosition(), bodyB->GetPosition(), length, frequency, dampingRatio,
                               jointMass);
}

AngleJoint* World::CreateAngleJoint(RigidBody* bodyA, RigidBody* bodyB, float frequency, float dampingRatio, float jointMass)
{
    void* mem = blockAllocator.Allocate(sizeof(AngleJoint));
    AngleJoint* aj = new (mem) AngleJoint(bodyA, bodyB, settings, frequency, dampingRatio, jointMass);

    Add(aj);
    return aj;
}

WeldJoint* World::CreateWeldJoint(RigidBody* bodyA, RigidBody* bodyB, float frequency, float dampingRatio, float jointMass)
{
    void* mem = blockAllocator.Allocate(sizeof(WeldJoint));
    WeldJoint* wj = new (mem) WeldJoint(bodyA, bodyB, (bodyA->GetPosition() + bodyB->GetPosition()) * 0.5f, settings, frequency,
                                        dampingRatio, jointMass);

    Add(wj);
    return wj;
}

PrismaticJoint* World::CreatePrismaticJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec2& anchor, const Vec2& dir, float frequency, float dampingRatio, float jointMass)
{
    void* mem = blockAllocator.Allocate(sizeof(PrismaticJoint));
    PrismaticJoint* pj = new (mem) PrismaticJoint(bodyA, bodyB, anchor, dir, settings, frequency, dampingRatio, jointMass);

    Add(pj);
    return pj;
}

PrismaticJoint* World::CreatePrismaticJoint(
    RigidBody* bodyA, RigidBody* bodyB, float frequency, float dampingRatio, float jointMass)
{
    return CreatePrismaticJoint(bodyA, bodyB, bodyB->GetPosition(), (bodyB->GetPosition() - bodyA->GetPosition()).Normalized(),
                                frequency, dampingRatio, jointMass);
}

PulleyJoint* World::CreatePulleyJoint(RigidBody* bodyA,
                                      RigidBody* bodyB,
                                      const Vec2& anchorA,
                                      const Vec2& anchorB,
                                      const Vec2& groundAnchorA,
                                      const Vec2& groundAnchorB,
                                      float ratio,
                                      float frequency,
                                      float dampingRatio,
                                      float jointMass)
{
    void* mem = blockAllocator.Allocate(sizeof(PulleyJoint));
    PulleyJoint* pj = new (mem) PulleyJoint(bodyA, bodyB, anchorA, anchorB, groundAnchorA, groundAnchorB, settings, ratio,
                                            frequency, dampingRatio, jointMass);

    Add(pj);
    return pj;
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

void World::FreeBody(RigidBody* body)
{
    body->~RigidBody();

    switch (body->shape)
    {
    case RigidBody::Shape::ShapePolygon:
        blockAllocator.Free(body, sizeof(Polygon));
        break;
    case RigidBody::Shape::ShapeCircle:
        blockAllocator.Free(body, sizeof(Circle));
        break;
    case RigidBody::Shape::ShapeCapsule:
        blockAllocator.Free(body, sizeof(Capsule));
        break;
    default:
        muliAssert(false);
        break;
    }
}

void World::FreeJoint(Joint* joint)
{
    joint->~Joint();

    switch (joint->type)
    {
    case Joint::Type::JointGrab:
        blockAllocator.Free(joint, sizeof(GrabJoint));
        break;
    case Joint::Type::JointRevolute:
        blockAllocator.Free(joint, sizeof(RevoluteJoint));
        break;
    case Joint::Type::JointDistance:
        blockAllocator.Free(joint, sizeof(DistanceJoint));
        break;
    case Joint::Type::JointAngle:
        blockAllocator.Free(joint, sizeof(AngleJoint));
        break;
    case Joint::Type::JointWeld:
        blockAllocator.Free(joint, sizeof(WeldJoint));
        break;
    case Joint::Type::JointPrismatic:
        blockAllocator.Free(joint, sizeof(PrismaticJoint));
        break;
    case Joint::Type::JointPulley:
        blockAllocator.Free(joint, sizeof(PulleyJoint));
        break;
    default:
        muliAssert(false);
        break;
    }
}

} // namespace muli