#pragma once

#include "box.h"
#include "circle.h"
#include "collision.h"
#include "common.h"
#include "contact_manager.h"
#include "distance_joint.h"
#include "grab_joint.h"
#include "joint.h"
#include "polygon.h"
#include "revolute_joint.h"
#include "rigidbody.h"
#include "util.h"

namespace spe
{

// Simulation settings
struct Settings
{
    mutable float DT = 1.0f / 60.0f;
    mutable float INV_DT = 60.0f;

    bool APPLY_GRAVITY = true;
    Vec2 GRAVITY{ 0.0f, -10.0f };

    bool WARM_STARTING = true;
    bool APPLY_WARM_STARTING_THRESHOLD = false;
    float WARM_STARTING_THRESHOLD = 0.005f * 0.005f - FLT_EPSILON;

    bool POSITION_CORRECTION = true;
    float POSITION_CORRECTION_BETA = 0.2f;

    float PENETRATION_SLOP = 0.005f;
    float RESTITUTION_SLOP = 0.1f;

    bool BLOCK_SOLVE = true;
    uint32_t VELOCITY_SOLVE_ITERATION = 10;
    uint32_t POSITION_SOLVE_ITERATION = 3;

    float REST_LINEAR_TOLERANCE = 0.01f * 0.01f;
    float REST_ANGULAR_TOLERANCE = (0.5f * SPE_PI / 180.0f) * (0.5f * SPE_PI / 180.0f);

    bool SLEEPING_ENABLED = true;
    float SLEEPING_TRESHOLD = 0.5f;

    AABB VALID_REGION{ Vec2{ -FLT_MAX, -FLT_MAX }, Vec2{ FLT_MAX, FLT_MAX } };
};

class World final
{
    friend class Island;
    friend class BroadPhase;
    friend class ContactManager;

public:
    World(const Settings& simulationSettings);
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
    void BufferDestroy(Joint* joint);

    Box* CreateBox(float size, RigidBody::Type type = RigidBody::Type::Dynamic, float density = DEFAULT_DENSITY);
    Box* CreateBox(float width, float height, RigidBody::Type type = RigidBody::Type::Dynamic, float density = DEFAULT_DENSITY);
    Circle* CreateCircle(float radius, RigidBody::Type type = RigidBody::Type::Dynamic, float density = DEFAULT_DENSITY);
    Polygon* CreatePolygon(std::vector<Vec2> vertices,
                           RigidBody::Type type = RigidBody::Type::Dynamic,
                           bool resetPosition = true,
                           float density = DEFAULT_DENSITY);
    Polygon* CreateRandomConvexPolygon(float radius, uint32_t num_vertices = 0, float density = DEFAULT_DENSITY);
    Polygon* CreateRegularPolygon(float radius,
                                  uint32_t num_vertices = 0,
                                  float initial_angle = 0,
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

    std::vector<RigidBody*> Query(const Vec2& point) const;
    std::vector<RigidBody*> Query(const AABB& aabb) const;

    RigidBody* GetBodyList();
    RigidBody* GetBodyListTail();
    uint32_t GetBodyCount() const;
    uint32_t GetSleepingBodyCount() const;
    uint32_t GetSleepingIslandCount() const;
    const AABBTree& GetBVH() const;
    Contact* GetContacts() const;
    Joint* GetJoints() const;
    uint32_t GetContactCount() const;
    uint32_t GetJointCount() const;

    void Awake();

private:
    const Settings& settings;
    uint32_t uid{ 0 };

    ContactManager contactManager;

    // Doubly linked list of all registered rigid bodies
    RigidBody* bodyList = nullptr;
    RigidBody* bodyListTail = nullptr;
    Joint* jointList = nullptr;

    uint32_t bodyCount = 0;
    uint32_t jointCount = 0;

    uint32_t numIslands = 0;
    uint32_t sleepingIslands = 0;
    uint32_t sleepingBodies = 0;

    std::vector<RigidBody*> destroyBufferBody;
    std::vector<Joint*> destroyBufferJoint;

    bool forceIntegration = false;

    void Add(Joint* joint);
};

inline World::World(const Settings& simulationSettings)
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
}

inline RigidBody* World::GetBodyList()
{
    return bodyList;
}

inline RigidBody* World::GetBodyListTail()
{
    return bodyListTail;
}

inline uint32_t World::GetSleepingBodyCount() const
{
    return sleepingBodies;
}

inline uint32_t World::GetSleepingIslandCount() const
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

inline uint32_t World::GetBodyCount() const
{
    return bodyCount;
}

inline uint32_t World::GetJointCount() const
{
    return jointCount;
}

inline uint32_t World::GetContactCount() const
{
    return contactManager.contactCount;
}

} // namespace spe