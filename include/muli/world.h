#pragma once

#include "angle_joint.h"
#include "box.h"
#include "capsule.h"
#include "circle.h"
#include "collision.h"
#include "common.h"
#include "contact_manager.h"
#include "distance_joint.h"
#include "grab_joint.h"
#include "joint.h"
#include "polygon.h"
#include "prismatic_joint.h"
#include "revolute_joint.h"
#include "rigidbody.h"
#include "util.h"
#include "weld_joint.h"

namespace muli
{

class World final
{
    friend class RigidBody;
    friend class Island;
    friend class BroadPhase;
    friend class ContactManager;

public:
    World(const WorldSettings& simulationSettings);
    ~World() noexcept;

    World(const World&) noexcept = delete;
    World& operator=(const World&) noexcept = delete;

    World(World&&) noexcept = delete;
    World& operator=(World&&) noexcept = delete;

    void Step(float dt);
    void Reset();

    void Add(RigidBody* body);
    void Add(const std::vector<RigidBody*>& bodies);
    void Destroy(RigidBody* body);
    void Destroy(const std::vector<RigidBody*>& bodies);
    void Destroy(Joint* joint);
    void Destroy(const std::vector<Joint*>& joints);

    // Buffered body will be destroy at the end of the step
    void BufferDestroy(RigidBody* body);
    void BufferDestroy(const std::vector<RigidBody*>& bodies);
    void BufferDestroy(Joint* joint);
    void BufferDestroy(const std::vector<Joint*>& joints);

    // Factory functions

    Box* CreateBox(float size,
                   RigidBody::Type type = RigidBody::Type::Dynamic,
                   float radius = DEFAULT_RADIUS,
                   float density = DEFAULT_DENSITY);
    Box* CreateBox(float width,
                   float height,
                   RigidBody::Type type = RigidBody::Type::Dynamic,
                   float radius = DEFAULT_RADIUS,
                   float density = DEFAULT_DENSITY);
    Circle* CreateCircle(float radius, RigidBody::Type type = RigidBody::Type::Dynamic, float density = DEFAULT_DENSITY);
    Polygon* CreatePolygon(const std::vector<Vec2>& vertices,
                           RigidBody::Type type = RigidBody::Type::Dynamic,
                           bool resetCenter = true,
                           float radius = DEFAULT_RADIUS,
                           float density = DEFAULT_DENSITY);
    Capsule* CreateCapsule(float length,
                           float radius,
                           bool horizontal = false,
                           RigidBody::Type type = RigidBody::Type::Dynamic,
                           float density = DEFAULT_DENSITY);
    Capsule* CreateCapsule(const Vec2& p1,
                           const Vec2& p2,
                           float radius,
                           bool resetPosition = true,
                           RigidBody::Type type = RigidBody::Type::Dynamic,
                           float density = DEFAULT_DENSITY);
    Polygon* CreateRandomConvexPolygon(float length,
                                       uint32 vertexCount = 0,
                                       float radius = DEFAULT_RADIUS,
                                       float density = DEFAULT_DENSITY);
    Polygon* CreateRegularPolygon(float length,
                                  uint32 vertexCount = 0,
                                  float initial_angle = 0,
                                  float radius = DEFAULT_RADIUS,
                                  float density = DEFAULT_DENSITY);

    GrabJoint* CreateGrabJoint(
        RigidBody* body, Vec2 anchor, Vec2 target, float frequency = 1.0f, float dampingRatio = 0.5f, float jointMass = 1.0f);
    RevoluteJoint* CreateRevoluteJoint(RigidBody* bodyA,
                                       RigidBody* bodyB,
                                       Vec2 anchor,
                                       float frequency = 10.0f,
                                       float dampingRatio = 1.0f,
                                       float jointMass = 1.0f);
    DistanceJoint* CreateDistanceJoint(RigidBody* bodyA,
                                       RigidBody* bodyB,
                                       Vec2 anchorA,
                                       Vec2 anchorB,
                                       float length = -1.0f,
                                       float frequency = 10.0f,
                                       float dampingRatio = 1.0f,
                                       float jointMass = 1.0f);
    DistanceJoint* CreateDistanceJoint(RigidBody* bodyA,
                                       RigidBody* bodyB,
                                       float length = -1.0f,
                                       float frequency = 10.0f,
                                       float dampingRatio = 1.0f,
                                       float jointMass = 1.0f);
    AngleJoint* CreateAngleJoint(
        RigidBody* bodyA, RigidBody* bodyB, float frequency = 10.0f, float dampingRatio = 1.0f, float jointMass = 1.0f);
    WeldJoint* CreateWeldJoint(
        RigidBody* bodyA, RigidBody* bodyB, float frequency = -1.0f, float dampingRatio = 1.0f, float jointMass = 1.0f);
    PrismaticJoint* CreatePrismaticJoint(RigidBody* bodyA,
                                         RigidBody* bodyB,
                                         Vec2 anchor,
                                         Vec2 dir,
                                         float frequency = 10.0f,
                                         float dampingRatio = 1.0f,
                                         float jointMass = 1.0f);
    PrismaticJoint* CreatePrismaticJoint(
        RigidBody* bodyA, RigidBody* bodyB, float frequency = 10.0f, float dampingRatio = 1.0f, float jointMass = 1.0f);

    std::vector<RigidBody*> Query(const Vec2& point) const;
    std::vector<RigidBody*> Query(const AABB& aabb) const;

    RigidBody* GetBodyList();
    RigidBody* GetBodyListTail();
    uint32 GetBodyCount() const;
    uint32 GetSleepingBodyCount() const;
    uint32 GetSleepingIslandCount() const;
    const AABBTree& GetBVH() const;
    Contact* GetContacts() const;
    Joint* GetJoints() const;
    uint32 GetContactCount() const;
    uint32 GetJointCount() const;

    void Awake();

private:
    const WorldSettings& settings;
    uint32 uid{ 0 };

    ContactManager contactManager;

    // Doubly linked list of all registered rigid bodies
    RigidBody* bodyList = nullptr;
    RigidBody* bodyListTail = nullptr;
    Joint* jointList = nullptr;

    uint32 bodyCount = 0;
    uint32 jointCount = 0;

    uint32 numIslands = 0;
    uint32 sleepingIslands = 0;
    uint32 sleepingBodies = 0;

    std::vector<RigidBody*> destroyBufferBody;
    std::vector<Joint*> destroyBufferJoint;

    bool integrateForce = false;

    void Add(Joint* joint);
};

inline World::World(const WorldSettings& simulationSettings)
    : settings{ simulationSettings }
    , contactManager{ *this }
{
}

inline World::~World() noexcept
{
    Reset();
}

inline void World::Awake()
{
    for (RigidBody* b = bodyList; b; b = b->next)
    {
        b->Awake();
    }
    integrateForce = true;
}

inline RigidBody* World::GetBodyList()
{
    return bodyList;
}

inline RigidBody* World::GetBodyListTail()
{
    return bodyListTail;
}

inline uint32 World::GetSleepingBodyCount() const
{
    return sleepingBodies;
}

inline uint32 World::GetSleepingIslandCount() const
{
    return sleepingIslands;
}

inline const AABBTree& World::GetBVH() const
{
    return contactManager.broadPhase.tree;
}

inline Contact* World::GetContacts() const
{
    return contactManager.contactList;
}

inline Joint* World::GetJoints() const
{
    return jointList;
}

inline uint32 World::GetBodyCount() const
{
    return bodyCount;
}

inline uint32 World::GetJointCount() const
{
    return jointCount;
}

inline uint32 World::GetContactCount() const
{
    return contactManager.contactCount;
}

} // namespace muli