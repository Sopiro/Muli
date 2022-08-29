#pragma once

#include "box.h"
#include "broad_phase.h"
#include "circle.h"
#include "common.h"
#include "contact_constraint.h"
#include "detection.h"
#include "distance_joint.h"
#include "grab_joint.h"
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
    glm::vec2 GRAVITY{ 0.0f, -10.0f };

    bool IMPULSE_ACCUMULATION = true;
    bool WARM_STARTING = true;
    bool APPLY_WARM_STARTING_THRESHOLD = true;
    float WARM_STARTING_THRESHOLD = 0.005f * 0.005f - glm::epsilon<float>();

    bool POSITION_CORRECTION = true;
    float POSITION_CORRECTION_BETA = 0.2f;

    float PENETRATION_SLOP = 0.005f;
    float RESTITUTION_SLOP = 0.5f;

    bool BLOCK_SOLVE = true;
    uint32_t SOLVE_ITERATION = 10;

    float REST_LINEAR_TOLERANCE = 0.01f * 0.01f;
    float REST_ANGULAR_TOLERANCE = (0.5f * glm::pi<float>() / 180.0f) * (0.5f * glm::pi<float>() / 180.0f);

    bool SLEEPING_ENABLED = true;
    float SLEEPING_TRESHOLD = 0.5f;

    AABB VALID_REGION{ glm::vec2{ -FLT_MAX, -FLT_MAX }, glm::vec2{ FLT_MAX, FLT_MAX } };
};

class World final
{
    friend class Island;
    friend class BroadPhase;

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

    Box* CreateBox(float size, BodyType type = Dynamic, float density = DEFAULT_DENSITY);
    Box* CreateBox(float width, float height, BodyType type = Dynamic, float density = DEFAULT_DENSITY);
    Circle* CreateCircle(float radius, BodyType type = Dynamic, float density = DEFAULT_DENSITY);
    Polygon* CreatePolygon(std::vector<glm::vec2> vertices,
                           BodyType type = Dynamic,
                           bool resetPosition = true,
                           float density = DEFAULT_DENSITY);
    Polygon* CreateRandomConvexPolygon(float radius, uint32_t num_vertices = 0, float density = DEFAULT_DENSITY);
    Polygon* CreateRegularPolygon(float radius,
                                  uint32_t num_vertices = 0,
                                  float initial_angle = 0,
                                  float density = DEFAULT_DENSITY);

    GrabJoint* CreateGrabJoint(RigidBody* body,
                               glm::vec2 anchor,
                               glm::vec2 target,
                               float frequency = 1.0f,
                               float dampingRatio = 0.5f,
                               float jointMass = 1.0f);
    RevoluteJoint* CreateRevoluteJoint(RigidBody* bodyA,
                                       RigidBody* bodyB,
                                       glm::vec2 anchor,
                                       float frequency = 10.0f,
                                       float dampingRatio = 1.0f,
                                       float jointMass = 1.0f);
    DistanceJoint* CreateDistanceJoint(RigidBody* bodyA,
                                       RigidBody* bodyB,
                                       glm::vec2 anchorA,
                                       glm::vec2 anchorB,
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

    std::vector<RigidBody*> Query(const glm::vec2& point) const;
    std::vector<RigidBody*> Query(const AABB& region) const;

    const std::vector<RigidBody*>& GetBodies() const;
    const uint32_t GetSleepingBodyCount() const;
    const uint32_t GetSleepingIslandCount() const;
    const AABBTree& GetBVH() const;
    const std::vector<ContactConstraint>& GetContactConstraints() const;
    const std::vector<Joint*>& GetJoints() const;

    void Awake();

private:
    const Settings& settings;
    uint32_t uid{ 0 };

    BroadPhase broadphase;

    // All registered rigid bodies
    std::unordered_map<uint32_t, RigidBody*> bodyMap;
    std::vector<RigidBody*> bodies{};

    // Constraints to be solved
    std::vector<ContactConstraint> contactConstraints{};
    std::unordered_map<uint64_t, ContactConstraint*> contactConstraintMap{};
    std::vector<ContactConstraint> newContactConstraints{};
    std::unordered_map<uint64_t, ContactConstraint*> newContactConstraintMap{};

    std::vector<Joint*> joints{};
    std::unordered_map<uint32_t, Joint*> jointMap{};

    bool forceIntegration = false;
    uint32_t numIslands = 0;
    uint32_t sleepingIslands = 0;
    uint32_t sleepingBodies = 0;
};

inline const std::vector<RigidBody*>& World::GetBodies() const
{
    return bodies;
}

inline const uint32_t World::GetSleepingBodyCount() const
{
    return sleepingBodies;
}

inline const uint32_t World::GetSleepingIslandCount() const
{
    return sleepingIslands;
}

inline const AABBTree& World::GetBVH() const
{
    return broadphase.tree;
}

inline const std::vector<ContactConstraint>& World::GetContactConstraints() const
{
    return contactConstraints;
}

inline const std::vector<Joint*>& World::GetJoints() const
{
    return joints;
}

inline void World::Awake()
{
    for (RigidBody* b : bodies)
    {
        b->Awake();
    }
}

} // namespace spe