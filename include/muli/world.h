#pragma once

#include "block_allocator.h"
#include "callbacks.h"
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
    World(const WorldSettings& settings);
    ~World() noexcept;

    World(const World&) noexcept = delete;
    World& operator=(const World&) noexcept = delete;

    World(World&&) noexcept = delete;
    World& operator=(World&&) noexcept = delete;

    float Step(float dt);
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
    RigidBody* CreateCircle(float radius, RigidBody::Type type = RigidBody::Type::dynamic_body, float density = default_density);
    RigidBody* CreateCapsule(float length,
                             float radius,
                             bool horizontal = false,
                             RigidBody::Type type = RigidBody::Type::dynamic_body,
                             float density = default_density);
    RigidBody* CreateCapsule(const Vec2& point1,
                             const Vec2& point2,
                             float radius,
                             RigidBody::Type type = RigidBody::Type::dynamic_body,
                             bool resetPosition = false,
                             float density = default_density);
    RigidBody* CreatePolygon(const std::vector<Vec2>& vertices,
                             RigidBody::Type type = RigidBody::Type::dynamic_body,
                             bool resetCenter = true,
                             float radius = default_radius,
                             float density = default_density);
    RigidBody* CreateBox(float size,
                         RigidBody::Type type = RigidBody::Type::dynamic_body,
                         float radius = default_radius,
                         float density = default_density);
    RigidBody* CreateBox(float width,
                         float height,
                         RigidBody::Type type = RigidBody::Type::dynamic_body,
                         float radius = default_radius,
                         float density = default_density);
    RigidBody* CreateRandomConvexPolygon(float length,
                                         int32 vertexCount = 0,
                                         RigidBody::Type type = RigidBody::Type::dynamic_body,
                                         float radius = default_radius,
                                         float density = default_density);
    RigidBody* CreateRegularPolygon(float length,
                                    int32 vertexCount = 0,
                                    float initialAngle = 0,
                                    RigidBody::Type type = RigidBody::Type::dynamic_body,
                                    float radius = default_radius,
                                    float density = default_density);

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
    WeldJoint* CreateWeldJoint(RigidBody* bodyA,
                               RigidBody* bodyB,
                               const Vec2& anchor,
                               float frequency = -1.0f,
                               float dampingRatio = 1.0f,
                               float jointMass = 1.0f);
    LineJoint* CreateLineJoint(RigidBody* bodyA,
                               RigidBody* bodyB,
                               const Vec2& anchorA,
                               const Vec2& dir,
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
    void Query(const Vec2& point, WorldQueryCallback* callback);
    void Query(const AABB& aabb, WorldQueryCallback* callback);

    void RayCastAny(
        const Vec2& from,
        const Vec2& to,
        const std::function<float(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)>& callback);
    bool RayCastClosest(
        const Vec2& from,
        const Vec2& to,
        const std::function<void(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)>& callback);
    void RayCastAny(const Vec2& from, const Vec2& to, RayCastAnyCallback* callback);
    bool RayCastClosest(const Vec2& from, const Vec2& to, RayCastClosestCallback* callback);

    RigidBody* GetBodyList() const;
    RigidBody* GetBodyListTail() const;
    int32 GetBodyCount() const;
    Joint* GetJoints() const;
    int32 GetJointCount() const;

    const Contact* GetContacts() const;
    int32 GetContactCount() const;

    int32 GetSleepingBodyCount() const;
    int32 GetAwakeIslandCount() const;

    const AABBTree& GetDynamicTree() const;
    void RebuildDynamicTree();

    void Awake();

private:
    friend class RigidBody;
    friend class Island;
    friend class ContactManager;
    friend class BroadPhase;

    void Solve();
    float SolveTOI();

    void FreeBody(RigidBody* body);
    void AddJoint(Joint* joint);
    void FreeJoint(Joint* joint);

    const WorldSettings& settings;
    ContactManager contactManager;

    // Doubly linked list of all registered rigid bodies
    RigidBody* bodyList;
    RigidBody* bodyListTail;
    int32 bodyCount;

    Joint* jointList;
    int32 jointCount;

    int32 islandCount;
    int32 sleepingBodyCount;

    bool stepComplete;

    std::vector<RigidBody*> destroyBodyBuffer;
    std::vector<Joint*> destroyJointBuffer;

    StackAllocator stackAllocator;
    BlockAllocator blockAllocator;
};

inline void World::Awake()
{
    for (RigidBody* b = bodyList; b; b = b->next)
    {
        b->Awake();
    }
}

inline RigidBody* World::GetBodyList() const
{
    return bodyList;
}

inline RigidBody* World::GetBodyListTail() const
{
    return bodyListTail;
}

inline int32 World::GetBodyCount() const
{
    return bodyCount;
}

inline int32 World::GetSleepingBodyCount() const
{
    return sleepingBodyCount;
}

inline int32 World::GetAwakeIslandCount() const
{
    return islandCount;
}

inline const Contact* World::GetContacts() const
{
    return contactManager.contactList;
}

inline int32 World::GetContactCount() const
{
    return contactManager.contactCount;
}

inline Joint* World::GetJoints() const
{
    return jointList;
}

inline int32 World::GetJointCount() const
{
    return jointCount;
}

inline const AABBTree& World::GetDynamicTree() const
{
    return contactManager.broadPhase.tree;
}

inline void World::RebuildDynamicTree()
{
    contactManager.broadPhase.tree.Rebuild();
}

} // namespace muli