#pragma once

#include "collision.h"
#include "common.h"
#include "contact_manager.h"
#include "predefined_block_allocator.h"
#include "stack_allocator.h"
#include "util.h"

#include "collider.h"
#include "rigidbody.h"

#include "angle_joint.h"
#include "distance_joint.h"
#include "grab_joint.h"
#include "joint.h"
#include "line_joint.h"
#include "motor_joint.h"
#include "prismatic_joint.h"
#include "pulley_joint.h"
#include "revolute_joint.h"
#include "weld_joint.h"

namespace muli
{

class World final
{
public:
    World(const WorldSettings& simulationSettings);
    ~World() noexcept;

    World(const World&) noexcept = delete;
    World& operator=(const World&) noexcept = delete;

    World(World&&) noexcept = delete;
    World& operator=(World&&) noexcept = delete;

    void Step(float dt);
    void Reset();

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

    RigidBody* CreateEmptyBody(RigidBody::Type type = RigidBody::Type::dynamic_body);
    RigidBody* CreateCircle(float radius, RigidBody::Type type = RigidBody::Type::dynamic_body, float density = DEFAULT_DENSITY);
    RigidBody* CreateCapsule(float length,
                             float radius,
                             bool horizontal = false,
                             RigidBody::Type type = RigidBody::Type::dynamic_body,
                             float density = DEFAULT_DENSITY);
    RigidBody* CreateCapsule(const Vec2& p1,
                             const Vec2& p2,
                             float radius,
                             RigidBody::Type type = RigidBody::Type::dynamic_body,
                             bool resetPosition = false,
                             float density = DEFAULT_DENSITY);
    RigidBody* CreatePolygon(const std::vector<Vec2>& vertices,
                             RigidBody::Type type = RigidBody::Type::dynamic_body,
                             bool resetCenter = true,
                             float radius = DEFAULT_RADIUS,
                             float density = DEFAULT_DENSITY);
    RigidBody* CreateBox(float size,
                         RigidBody::Type type = RigidBody::Type::dynamic_body,
                         float radius = DEFAULT_RADIUS,
                         float density = DEFAULT_DENSITY);
    RigidBody* CreateBox(float width,
                         float height,
                         RigidBody::Type type = RigidBody::Type::dynamic_body,
                         float radius = DEFAULT_RADIUS,
                         float density = DEFAULT_DENSITY);
    RigidBody* CreateRandomConvexPolygon(float length,
                                         int32 vertexCount = 0,
                                         RigidBody::Type type = RigidBody::Type::dynamic_body,
                                         float radius = DEFAULT_RADIUS,
                                         float density = DEFAULT_DENSITY);
    RigidBody* CreateRegularPolygon(float length,
                                    int32 vertexCount = 0,
                                    float initial_angle = 0,
                                    RigidBody::Type type = RigidBody::Type::dynamic_body,
                                    float radius = DEFAULT_RADIUS,
                                    float density = DEFAULT_DENSITY);

    GrabJoint* CreateGrabJoint(RigidBody* body,
                               const Vec2& anchor,
                               const Vec2& target,
                               float frequency = 1.0f,
                               float dampingRatio = 0.5f,
                               float jointMass = 1.0f);
    RevoluteJoint* CreateRevoluteJoint(RigidBody* bodyA,
                                       RigidBody* bodyB,
                                       const Vec2& anchor,
                                       float frequency = 10.0f,
                                       float dampingRatio = 1.0f,
                                       float jointMass = 1.0f);
    DistanceJoint* CreateDistanceJoint(RigidBody* bodyA,
                                       RigidBody* bodyB,
                                       const Vec2& anchorA,
                                       const Vec2& anchorB,
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
    LineJoint* CreateLineJoint(RigidBody* bodyA,
                               RigidBody* bodyB,
                               Vec2 anchorA,
                               Vec2 dir,
                               float frequency = 10.0f,
                               float dampingRatio = 1.0f,
                               float jointMass = 1.0f);
    LineJoint* CreateLineJoint(
        RigidBody* bodyA, RigidBody* bodyB, float frequency = 10.0f, float dampingRatio = 1.0f, float jointMass = 1.0f);
    PrismaticJoint* CreatePrismaticJoint(RigidBody* bodyA,
                                         RigidBody* bodyB,
                                         const Vec2& anchor,
                                         const Vec2& dir,
                                         float frequency = -1.0f,
                                         float dampingRatio = 1.0f,
                                         float jointMass = 1.0f);
    PrismaticJoint* CreatePrismaticJoint(
        RigidBody* bodyA, RigidBody* bodyB, float frequency = -1.0f, float dampingRatio = 1.0f, float jointMass = 1.0f);
    PulleyJoint* CreatePulleyJoint(RigidBody* bodyA,
                                   RigidBody* bodyB,
                                   const Vec2& anchorA,
                                   const Vec2& anchorB,
                                   const Vec2& groundAnchorA,
                                   const Vec2& groundAnchorB,
                                   float ratio = 1.0f,
                                   float frequency = -1.0f,
                                   float dampingRatio = 1.0f,
                                   float jointMass = 1.0f);
    MotorJoint* CreateMotorJoint(RigidBody* bodyA,
                                 RigidBody* bodyB,
                                 const Vec2& anchor,
                                 float maxForce = 1000.0f,
                                 float maxTorque = 1000.0f,
                                 float frequency = -1.0f,
                                 float dampingRatio = 1.0f,
                                 float jointMass = 1.0f);

    std::vector<Collider*> Query(const Vec2& point) const;
    std::vector<Collider*> Query(const AABB& aabb) const;

    void RayCastAny(
        const Vec2& from,
        const Vec2& to,
        const std::function<float(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)>& callback);
    bool RayCastClosest(
        const Vec2& from,
        const Vec2& to,
        const std::function<void(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)>& callback);

    RigidBody* GetBodyList();
    RigidBody* GetBodyListTail();
    uint32 GetBodyCount() const;
    uint32 GetSleepingBodyCount() const;
    uint32 GetSleepingIslandCount() const;
    Contact* GetContacts() const;
    uint32 GetContactCount() const;
    Joint* GetJoints() const;
    uint32 GetJointCount() const;

    const AABBTree& GetBVH() const;
    void RebuildBVH();

    void Awake();

private:
    friend class RigidBody;
    friend class Island;
    friend class BroadPhase;
    friend class ContactManager;

    const WorldSettings& settings;

    ContactManager contactManager;

    // Doubly linked list of all registered rigid bodies
    RigidBody* bodyList;
    RigidBody* bodyListTail;
    Joint* jointList;

    uint32 bodyCount;
    uint32 jointCount;

    uint32 numIslands;
    uint32 sleepingIslands;
    uint32 sleepingBodies;

    std::vector<RigidBody*> destroyBufferBody;
    std::vector<Joint*> destroyBufferJoint;

    bool integrateForce;

    void Add(Joint* joint);
    void FreeBody(RigidBody* body);
    void FreeJoint(Joint* joint);

    StackAllocator stackAllocator;
    PredefinedBlockAllocator blockAllocator;
};

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

inline uint32 World::GetBodyCount() const
{
    return bodyCount;
}

inline uint32 World::GetSleepingBodyCount() const
{
    return sleepingBodies;
}

inline uint32 World::GetSleepingIslandCount() const
{
    return sleepingIslands;
}

inline Contact* World::GetContacts() const
{
    return contactManager.contactList;
}

inline uint32 World::GetContactCount() const
{
    return contactManager.contactCount;
}

inline Joint* World::GetJoints() const
{
    return jointList;
}

inline uint32 World::GetJointCount() const
{
    return jointCount;
}

inline const AABBTree& World::GetBVH() const
{
    return contactManager.broadPhase.tree;
}

inline void World::RebuildBVH()
{
    contactManager.broadPhase.tree.Rebuild();
}

} // namespace muli