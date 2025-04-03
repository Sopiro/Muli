#include "muli/world.h"
#include "muli/capsule.h"
#include "muli/circle.h"
#include "muli/island.h"
#include "muli/polygon.h"
#include "muli/random.h"
#include "muli/time_of_impact.h"

namespace muli
{

World::World(const WorldSettings& settings)
    : settings{ settings }
    , contactManager{ this }
    , bodyList{ nullptr }
    , bodyListTail{ nullptr }
    , bodyCount{ 0 }
    , jointList{ nullptr }
    , jointCount{ 0 }
    , islandCount{ 0 }
    , sleepingBodyCount{ 0 }
    , stepComplete{ true }
{
    // Assertions for stable CCD
    MuliAssert(toi_position_solver_threshold < linear_slop * 2.0f);
    MuliAssert(default_radius >= toi_position_solver_threshold);
    MuliAssert(position_solver_threshold > toi_position_solver_threshold);
}

World::~World() noexcept
{
    Reset();
}

void World::Reset()
{
    RigidBody* b = bodyList;
    while (b)
    {
        RigidBody* b0 = b;
        b = b->next;
        Destroy(b0);
    }

    MuliAssert(bodyList == nullptr);
    MuliAssert(bodyListTail == nullptr);
    MuliAssert(jointList == nullptr);
    MuliAssert(bodyCount == 0);
    MuliAssert(jointCount == 0);
    MuliAssert(blockAllocator.GetBlockCount() == 0);

    destroyBodyBuffer.clear();
    destroyJointBuffer.clear();
}

void World::Solve()
{
    // Build the constraint island
    Island island{ this, bodyCount, contactManager.contactCount, jointCount };

    int32 restingBodies = 0;
    int32 islandID = 0;
    sleepingBodyCount = 0;

    // Use arena allocator to avoid per-frame allocation
    RigidBody** stack = (RigidBody**)linearAllocator.Allocate(bodyCount * sizeof(RigidBody*));
    int32 stackPointer;

    // Perform a DFS(Depth First Search) on the constraint graph
    // After building island, each island can be solved in parallel because they are independent of each other
    for (RigidBody* b = bodyList; b; b = b->next)
    {
        if (b->flag & RigidBody::flag_island)
        {
            continue;
        }

        if (b->IsSleeping() == true)
        {
            ++sleepingBodyCount;
            continue;
        }

        if (b->type == RigidBody::Type::static_body)
        {
            continue;
        }

        if (b->IsEnabled() == false)
        {
            continue;
        }

        stackPointer = 0;
        stack[stackPointer++] = b;
        b->flag |= RigidBody::flag_island;

        ++islandID;
        while (stackPointer > 0)
        {
            RigidBody* t = stack[--stackPointer];

            island.Add(t);
            t->islandID = islandID;

            for (ContactEdge* ce = t->contactList; ce; ce = ce->next)
            {
                Contact* c = ce->contact;

                if (c->flag & Contact::flag_island)
                {
                    continue;
                }

                if ((c->flag & Contact::flag_touching) == 0)
                {
                    continue;
                }

                if ((c->flag & Contact::flag_enabled) == 0)
                {
                    continue;
                }

                island.Add(c);
                c->flag |= Contact::flag_island;

                RigidBody* other = ce->other;

                if (other->flag & RigidBody::flag_island)
                {
                    continue;
                }

                if (other->type == RigidBody::Type::static_body)
                {
                    continue;
                }

                MuliAssert(stackPointer < bodyCount);
                stack[stackPointer++] = other;
                other->flag |= RigidBody::flag_island;
            }

            for (JointEdge* je = t->jointList; je; je = je->next)
            {
                Joint* j = je->joint;

                if (j->flagIsland == true)
                {
                    continue;
                }

                RigidBody* other = je->other;

                if (other->IsEnabled() == false)
                {
                    continue;
                }

                island.Add(j);
                j->flagIsland = true;

                if (other->flag & RigidBody::flag_island)
                {
                    continue;
                }

                if (other->type == RigidBody::Type::static_body)
                {
                    continue;
                }

                MuliAssert(stackPointer < bodyCount);
                stack[stackPointer++] = other;
                other->flag |= RigidBody::flag_island;
            }

            if (t->resting > settings.sleeping_time)
            {
                restingBodies++;
            }
        }

        island.sleeping = settings.sleeping && (restingBodies == island.bodyCount);
        island.Solve();
        island.Clear();
        restingBodies = 0;
    }

    linearAllocator.Free(stack, bodyCount * sizeof(RigidBody*));

    islandCount = islandID;

    for (RigidBody* body = bodyList; body; body = body->next)
    {
        MuliAssert(body->motion.alpha0 == 0.0f);

        if ((body->flag & RigidBody::flag_island) == 0)
        {
            continue;
        }

        MuliAssert(body->type != RigidBody::Type::static_body);

        // Clear island flag
        body->flag &= ~RigidBody::flag_island;

        // Synchronize transform and broad-phase collider node
        body->SynchronizeTransform();
        body->SynchronizeColliders();
    }

    for (Contact* contact = contactManager.contactList; contact; contact = contact->next)
    {
        contact->flag &= ~Contact::flag_island;
    }

    for (Joint* joint = jointList; joint; joint = joint->next)
    {
        joint->flagIsland = false;
    }
}

// Find TOI contacts and solve them
float World::SolveTOI()
{
    Island island{ this, 2 * max_toi_contacts, max_toi_contacts, 0 };

    while (true)
    {
        contactManager.UpdateContactGraph();

        // Find the first TOI
        Contact* minContact = nullptr;
        float minAlpha = 1.0f;

        for (Contact* c = contactManager.contactList; c; c = c->next)
        {
            if (c->IsEnabled() == false)
            {
                continue;
            }

            if (c->toiCount > max_sub_steps)
            {
                continue;
            }

            float alpha = 1.0f;

            if (c->flag & Contact::flag_toi)
            {
                // This contact has a cached TOI
                alpha = c->toi;
            }
            else
            {
                Collider* colliderA = c->colliderA;
                Collider* colliderB = c->colliderB;

                if (colliderA->IsEnabled() == false || colliderB->IsEnabled() == false)
                {
                    continue;
                }

                RigidBody* bodyA = colliderA->body;
                RigidBody* bodyB = colliderB->body;

                RigidBody::Type typeA = bodyA->type;
                RigidBody::Type typeB = bodyB->type;
                MuliAssert(typeA == RigidBody::Type::dynamic_body || typeB == RigidBody::Type::dynamic_body);

                bool activeA = bodyA->IsSleeping() == false && typeA != RigidBody::Type::static_body;
                bool activeB = bodyB->IsSleeping() == false && typeB != RigidBody::Type::static_body;

                // Is at least one body active (awake and dynamic or kinematic)?
                if (activeA == false && activeB == false)
                {
                    continue;
                }

                bool collideA = bodyA->IsContinuous() || typeA == RigidBody::Type::static_body;
                bool collideB = bodyB->IsContinuous() || typeB == RigidBody::Type::static_body;

                // Discard non-continuous dynamic|kinematic vs. non-continuous dynamic|kinematic case
                if (collideA == false && collideB == false)
                {
                    continue;
                }

                // Compute the TOI for this contact

                // Put the motions onto the same time interval
                float alpha0 = bodyA->motion.alpha0;
                if (bodyA->motion.alpha0 < bodyB->motion.alpha0)
                {
                    alpha0 = bodyB->motion.alpha0;
                    bodyA->motion.Advance(alpha0);
                }
                else if (bodyA->motion.alpha0 > bodyB->motion.alpha0)
                {
                    alpha0 = bodyA->motion.alpha0;
                    bodyB->motion.Advance(alpha0);
                }

                MuliAssert(alpha0 < 1.0f);

                TOIOutput output;
                ComputeTimeOfImpact(colliderA->shape, bodyA->motion, colliderB->shape, bodyB->motion, 1.0f, &output);

#if 0
                switch (output.state)
                {
                case TOIOutput::unknown:
                    std::cout << "unknown" << std::endl;
                    break;
                case TOIOutput::failed:
                    std::cout << "failed" << std::endl;
                    break;
                case TOIOutput::overlapped:
                    std::cout << "overlapped" << std::endl;
                    break;
                case TOIOutput::touching:
                    std::cout << "touching: " << output.t << std::endl;
                    break;
                case TOIOutput::separated:
                    std::cout << "separated" << std::endl;
                    break;

                default:
                    MuliAssert(false);
                    break;
                }
#endif

                if (output.state == TOIOutput::touching)
                {
                    // TOI is the fraction in [alpha0, 1.0]
                    alpha = Min(alpha0 + (1.0f - alpha0) * output.t, 1.0f);
                }
                else
                {
                    alpha = 1.0f;
                }

                // Save the TOI
                c->toi = alpha;
                c->flag |= Contact::flag_toi;
            }

            if (alpha < minAlpha)
            {
                // Update the minimum TOI
                minContact = c;
                minAlpha = alpha;
            }
        }

        if (minContact == nullptr || 1.0f - 10.0f * epsilon < minAlpha)
        {
            // Done! No more TOI events
            stepComplete = true;
            break;
        }

        // Advance the bodies to the TOI
        Collider* colliderA = minContact->colliderA;
        Collider* colliderB = minContact->colliderB;
        RigidBody* bodyA = colliderA->body;
        RigidBody* bodyB = colliderB->body;

        Motion save1 = bodyA->motion;
        Motion save2 = bodyB->motion;

        bodyA->Advance(minAlpha);
        bodyB->Advance(minAlpha);

        // Find the TOI contact points
        minContact->Update();
        minContact->flag &= ~Contact::flag_toi;
        ++minContact->toiCount;

        // Contact disabled by the user or no contact points found
        if (minContact->IsEnabled() == false || minContact->IsTouching() == false)
        {
            // Restore the motions
            minContact->SetEnabled(false); // Prevent duplicate
            bodyA->motion = save1;
            bodyB->motion = save2;
            bodyA->SynchronizeTransform();
            bodyB->SynchronizeTransform();
            continue;
        }

        bodyA->Awake();
        bodyB->Awake();

        // Build the island
        island.Clear();
        island.Add(bodyA);
        island.Add(bodyB);
        island.Add(minContact);

        bodyA->flag |= RigidBody::flag_island;
        bodyB->flag |= RigidBody::flag_island;
        minContact->flag |= Contact::flag_island;

        // Find contacts for TOI contact bodies
        RigidBody* bodies[2] = { bodyA, bodyB };
        for (int32 i = 0; i < 2; ++i)
        {
            RigidBody* body = bodies[i];

            if (body->type != RigidBody::Type::dynamic_body)
            {
                continue;
            }

            for (ContactEdge* ce = body->contactList; ce; ce = ce->next)
            {
                if (island.bodyCount == island.bodyCapacity)
                {
                    break;
                }

                if (island.contactCount == island.contactCapacity)
                {
                    break;
                }

                Contact* contact = ce->contact;

                if (contact->flag & Contact::flag_island)
                {
                    continue;
                }

                RigidBody* other = ce->other;

                // Awake linked bodies
                other->Awake();

                // Discard non-continuous dynamic vs. non-continuous dynamic case
                if (body->IsContinuous() == false && other->IsContinuous() == false &&
                    other->type == RigidBody::Type::dynamic_body)
                {
                    continue;
                }

                Motion save = other->motion;

                // Tentatively advance the body to the TOI
                if ((other->flag & RigidBody::flag_island) == 0)
                {
                    other->Advance(minAlpha);
                }

                // Find the contact points
                contact->Update();

                // Contact disabled by the user or no contact points found
                if (contact->IsEnabled() == false || contact->IsTouching() == false)
                {
                    other->motion = save;
                    other->SynchronizeTransform();
                    continue;
                }

                // Add the contact to the island
                contact->flag |= Contact::flag_island;
                island.Add(contact);

                // Has the other body already been added to the island?
                if (other->flag & RigidBody::flag_island)
                {
                    continue;
                }

                if (other->type == RigidBody::Type::static_body)
                {
                    continue;
                }

                island.Add(other);

                // Awake linked bodies
                for (ContactEdge* oce = other->contactList; oce; oce = oce->next)
                {
                    oce->other->Awake();
                }
            }
        }

        // step the rest time
        float dt = (1.0f - minAlpha) * settings.step.dt;
        island.SolveTOI(dt);

        // Reset island flags and synchronize broad-phase collider node
        for (int32 i = 0; i < island.bodyCount; ++i)
        {
            RigidBody* body = island.bodies[i];
            body->flag &= ~RigidBody::flag_island;

            if (body->type != RigidBody::Type::dynamic_body)
            {
                continue;
            }

            body->SynchronizeColliders();

            // Invalidate all contact TOIs on this displaced body
            for (ContactEdge* ce = body->contactList; ce; ce = ce->next)
            {
                ce->contact->flag &= ~(Contact::flag_toi | Contact::flag_island);
            }
        }

        if (settings.sub_stepping)
        {
            // Solve only one TOI event and passed the remaining computation to the next Step() call
            stepComplete = false;
            return minAlpha;
        }
    }

    MuliAssert(stepComplete == true);

    for (RigidBody* body = bodyList; body; body = body->next)
    {
        body->motion.alpha0 = 0.0f;
        body->flag &= ~RigidBody::flag_island;
    }

    for (Contact* contact = contactManager.contactList; contact; contact = contact->next)
    {
        contact->flag &= ~(Contact::flag_toi | Contact::flag_island);
        contact->toiCount = 0;
        contact->toi = 1.0f;
    }

    return 1.0f;
}

float World::Step(float dt)
{
    settings.step.dt = dt;
    settings.step.inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

    if (settings.step.inv_dt == 0.0f)
    {
        return 0.0f;
    }

    // Grow the allocator buffer size if needed
    linearAllocator.GrowMemory();

    if (stepComplete)
    {
        // Update broad-phase contact graph
        contactManager.UpdateContactGraph();

        // Narrow-phase
        contactManager.EvaluateContacts();

        Solve();
    }

    float progress = 1.0f;
    if (settings.continuous)
    {
        progress = SolveTOI();
    }

    for (RigidBody* b : destroyBodyBuffer)
    {
        Destroy(b);
    }
    for (Joint* j : destroyJointBuffer)
    {
        Destroy(j);
    }

    destroyBodyBuffer.clear();
    destroyJointBuffer.clear();

    return progress;
}

void World::Destroy(RigidBody* body)
{
    MuliAssert(body->world == this);

    Collider* c = body->colliderList;
    while (c)
    {
        Collider* c0 = c;
        c = c->next;
        body->DestroyCollider(c0);
    }

    JointEdge* je = body->jointList;
    while (je)
    {
        JointEdge* je0 = je;
        je = je->next;
        je0->other->Awake();

        Destroy(je0->joint);
    }

    if (body->next) body->next->prev = body->prev;
    if (body->prev) body->prev->next = body->next;
    if (body == bodyList) bodyList = body->next;
    if (body == bodyListTail) bodyListTail = body->prev;

    FreeBody(body);
    --bodyCount;
}

void World::Destroy(std::span<RigidBody*> bodies)
{
    std::unordered_set<RigidBody*> destroyed;

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        RigidBody* b = bodies[i];

        if (destroyed.find(b) != destroyed.end())
        {
            destroyed.insert(b);
            Destroy(b);
        }
    }
}

void World::BufferDestroy(RigidBody* body)
{
    destroyBodyBuffer.push_back(body);
}

void World::BufferDestroy(std::span<RigidBody*> bodies)
{
    for (size_t i = 0; i < bodies.size(); ++i)
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

void World::Destroy(std::span<Joint*> joints)
{
    std::unordered_set<Joint*> destroyed;

    for (size_t i = 0; i < joints.size(); ++i)
    {
        Joint* j = joints[i];

        if (destroyed.find(j) != destroyed.end())
        {
            destroyed.insert(j);
            Destroy(j);
        }
    }
}

void World::BufferDestroy(Joint* joint)
{
    destroyJointBuffer.push_back(joint);
}

void World::BufferDestroy(std::span<Joint*> joints)
{
    for (size_t i = 0; i < joints.size(); ++i)
    {
        BufferDestroy(joints[i]);
    }
}

void World::Query(const Vec2& point, std::function<bool(Collider* collider)> callback) const
{
    struct TempCallback
    {
        Vec2 point;
        decltype(callback)& callbackFcn;

        TempCallback(Vec2 point, decltype(callback)& callback)
            : point{ point }
            , callbackFcn{ callback }
        {
        }

        bool QueryCallback(NodeProxy node, Collider* collider)
        {
            MuliNotUsed(node);

            // Body was destroyed while querying
            if (collider->body == nullptr)
            {
                return true;
            }

            if (collider->TestPoint(point))
            {
                return callbackFcn(collider);
            }

            return true;
        }
    } tempCallback(point, callback);

    tempCallback.point = point;

    contactManager.broadPhase.tree.Query(point, &tempCallback);
}

void World::Query(const AABB& aabb, std::function<bool(Collider* collider)> callback) const
{
    struct TempCallback
    {
        Polygon box;
        decltype(callback)& callbackFcn;

        TempCallback(const AABB& aabb, decltype(callback)& callback)
            : box{ { aabb.min, { aabb.max.x, aabb.min.y }, aabb.max, { aabb.min.x, aabb.max.y } }, false, 0.0f }
            , callbackFcn{ callback }
        {
        }

        bool QueryCallback(NodeProxy node, Collider* collider)
        {
            MuliNotUsed(node);

            // Body was destroyed while querying
            if (collider->body == nullptr)
            {
                return true;
            }

            if (Collide(collider->shape, collider->body->transform, &box, identity))
            {
                return callbackFcn(collider);
            }

            return true;
        }
    } tempCallback(aabb, callback);

    contactManager.broadPhase.tree.Query(aabb, &tempCallback);
}

void World::Query(const Vec2& point, WorldQueryCallback* callback)
{
    struct TempCallback
    {
        Vec2 point;
        WorldQueryCallback* callback;

        bool QueryCallback(NodeProxy node, Collider* collider)
        {
            MuliNotUsed(node);

            // Body was destroyed while querying
            if (collider->body == nullptr)
            {
                return true;
            }

            if (collider->TestPoint(point))
            {
                return callback->OnQuery(collider);
            }

            return true;
        }
    } tempCallback;

    tempCallback.point = point;
    tempCallback.callback = callback;

    contactManager.broadPhase.tree.Query(point, &tempCallback);
}

void World::Query(const AABB& aabb, WorldQueryCallback* callback)
{
    Vec2 vertices[4] = { aabb.min, { aabb.max.x, aabb.min.y }, aabb.max, { aabb.min.x, aabb.max.y } };
    Polygon box{ vertices, 4, false, 0.0f };

    struct TempCallback
    {
        Polygon region;
        WorldQueryCallback* callback;
        Transform t{ identity };

        TempCallback(const Polygon& box)
            : region{ box }
        {
        }

        bool QueryCallback(NodeProxy node, Collider* collider)
        {
            MuliNotUsed(node);

            // Body was destroyed while querying
            if (collider->body == nullptr)
            {
                return true;
            }

            if (Collide(collider->shape, collider->body->transform, &region, t))
            {
                return callback->OnQuery(collider);
            }

            return true;
        }
    } tempCallback(box);

    tempCallback.callback = callback;

    contactManager.broadPhase.tree.Query(aabb, &tempCallback);
}

void World::RayCastAny(const Vec2& from, const Vec2& to, float radius, RayCastAnyCallback* callback)
{
    AABBCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;
    input.halfExtents.Set(radius);

    struct TempCallback
    {
        RayCastAnyCallback* callback;

        float AABBCastCallback(const AABBCastInput& subInput, Collider* collider)
        {
            RayCastInput input;
            input.from = subInput.from;
            input.to = subInput.to;
            input.maxFraction = subInput.maxFraction;
            input.radius = subInput.halfExtents.x;

            RayCastOutput output;

            bool hit = collider->RayCast(input, &output);
            if (hit)
            {
                float fraction = output.fraction;
                Vec2 point = (1.0f - fraction) * input.from + fraction * input.to;

                return callback->OnHitAny(collider, point, output.normal, fraction);
            }

            return input.maxFraction;
        }
    } tempCallback;

    tempCallback.callback = callback;

    contactManager.broadPhase.tree.AABBCast(input, &tempCallback);
}

bool World::RayCastClosest(const Vec2& from, const Vec2& to, float radius, RayCastClosestCallback* callback)
{
    struct TempCallback : public RayCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider;
        Vec2 closestPoint;
        Vec2 closestNormal;
        float closestFraction;

        float OnHitAny(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestFraction = fraction;

            return fraction;
        }
    } tempCallback;

    RayCastAny(from, to, radius, &tempCallback);

    if (tempCallback.hit)
    {
        callback->OnHitClosest(
            tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal, tempCallback.closestFraction
        );
        return true;
    }

    return false;
}

void World::ShapeCastAny(const Shape* shape, const Transform& tf, const Vec2& translation, ShapeCastAnyCallback* callback)
{
    AABB aabb;
    shape->ComputeAABB(tf, &aabb);

    AABBCastInput input;
    input.from = tf.position;
    input.to = tf.position + translation;
    input.maxFraction = 1.0f;
    input.halfExtents = 0.5f * aabb.GetExtents();

    struct TempCallback
    {
        ShapeCastAnyCallback* callback;
        const Shape* shape;
        Transform tf;
        Vec2 translation;

        float AABBCastCallback(const AABBCastInput& input, Collider* collider)
        {
            ShapeCastOutput output;

            bool hit = ShapeCast(
                shape, tf, collider->GetShape(), collider->GetBody()->GetTransform(), translation * input.maxFraction, Vec2::zero,
                &output
            );
            if (hit)
            {
                return callback->OnHitAny(collider, output.point, output.normal, output.t * input.maxFraction);
            }

            return input.maxFraction;
        }
    } tempCallback;

    tempCallback.callback = callback;
    tempCallback.shape = shape;
    tempCallback.tf = tf;
    tempCallback.translation = translation;

    contactManager.broadPhase.tree.AABBCast(input, &tempCallback);
}

bool World::ShapeCastClosest(const Shape* shape, const Transform& tf, const Vec2& translation, ShapeCastClosestCallback* callback)
{
    struct TempCallback : ShapeCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider;
        Vec2 closestPoint;
        Vec2 closestNormal;
        float closestT = 1.0f;

        float OnHitAny(Collider* collider, const Vec2& point, const Vec2& normal, float t)
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestT = t;

            return t;
        }
    } tempCallback;

    ShapeCastAny(shape, tf, translation, &tempCallback);

    if (tempCallback.hit)
    {
        callback->OnHitClosest(
            tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal, tempCallback.closestT
        );
        return true;
    }

    return false;
}

void World::RayCastAny(
    const Vec2& from,
    const Vec2& to,
    float radius,
    std::function<float(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)> callback
)
{
    AABBCastInput input;
    input.from = from;
    input.to = to;
    input.maxFraction = 1.0f;
    input.halfExtents.Set(radius);

    struct TempCallback
    {
        decltype(callback)& callbackFcn;

        TempCallback(decltype(callback)& callback)
            : callbackFcn{ callback }
        {
        }

        float AABBCastCallback(const AABBCastInput& subInput, Collider* collider)
        {
            RayCastInput input;
            input.from = subInput.from;
            input.to = subInput.to;
            input.maxFraction = subInput.maxFraction;
            input.radius = subInput.halfExtents.x;

            RayCastOutput output;

            bool hit = collider->RayCast(input, &output);
            if (hit)
            {
                float fraction = output.fraction;
                Vec2 point = (1.0f - fraction) * input.from + fraction * input.to;

                return callbackFcn(collider, point, output.normal, fraction);
            }

            return input.maxFraction;
        }
    } tempCallback(callback);

    contactManager.broadPhase.tree.AABBCast(input, &tempCallback);
}

bool World::RayCastClosest(
    const Vec2& from,
    const Vec2& to,
    float radius,
    std::function<void(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)> callback
)
{
    struct TempCallback : public RayCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider;
        Vec2 closestPoint;
        Vec2 closestNormal;
        float closestFraction;

        float OnHitAny(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestFraction = fraction;

            return fraction;
        }
    } tempCallback;

    RayCastAny(from, to, radius, &tempCallback);

    if (tempCallback.hit)
    {
        callback(
            tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal, tempCallback.closestFraction
        );
        return true;
    }

    return false;
}

void World::ShapeCastAny(
    const Shape* shape,
    const Transform& tf,
    const Vec2& translation,
    std::function<float(Collider* collider, const Vec2& point, const Vec2& normal, float t)> callback
)
{
    AABB aabb;
    shape->ComputeAABB(tf, &aabb);

    AABBCastInput input;
    input.from = tf.position;
    input.to = tf.position + translation;
    input.maxFraction = 1.0f;
    input.halfExtents = 0.5f * aabb.GetExtents();

    struct TempCallback
    {
        decltype(callback)& callbackFcn;
        const Shape* shape;
        Transform tf;
        Vec2 translation;

        TempCallback(decltype(callback)& callback, const Shape* shape, Transform tf, Vec2 translation)
            : callbackFcn{ callback }
            , shape{ shape }
            , tf{ tf }
            , translation{ translation }
        {
        }

        float AABBCastCallback(const AABBCastInput& input, Collider* collider)
        {
            ShapeCastOutput output;

            bool hit = ShapeCast(
                shape, tf, collider->GetShape(), collider->GetBody()->GetTransform(), translation * input.maxFraction, Vec2::zero,
                &output
            );
            if (hit)
            {
                return callbackFcn(collider, output.point, output.normal, output.t * input.maxFraction);
            }

            return input.maxFraction;
        }
    } tempCallback(callback, shape, tf, translation);

    contactManager.broadPhase.tree.AABBCast(input, &tempCallback);
}

bool World::ShapeCastClosest(
    const Shape* shape,
    const Transform& tf,
    const Vec2& translation,
    std::function<void(Collider* collider, const Vec2& point, const Vec2& normal, float t)> callback
)
{
    struct TempCallback : ShapeCastAnyCallback
    {
        bool hit = false;
        Collider* closestCollider;
        Vec2 closestPoint;
        Vec2 closestNormal;
        float closestT = 1.0f;

        float OnHitAny(Collider* collider, const Vec2& point, const Vec2& normal, float t)
        {
            hit = true;
            closestCollider = collider;
            closestPoint = point;
            closestNormal = normal;
            closestT = t;

            return t;
        }
    } tempCallback;

    ShapeCastAny(shape, tf, translation, &tempCallback);

    if (tempCallback.hit)
    {
        callback(tempCallback.closestCollider, tempCallback.closestPoint, tempCallback.closestNormal, tempCallback.closestT);
        return true;
    }

    return false;
}

RigidBody* World::DuplicateBody(RigidBody* body)
{
    MuliAssert(body->world == this);
    if (body->world != this)
    {
        return nullptr;
    }

    RigidBody* b = CreateEmptyBody(body->GetType());

    for (Collider* collider = body->colliderList; collider; collider = collider->next)
    {
        Collider* c = b->CreateCollider(collider->GetShape(), collider->GetDensity(), collider->GetMaterial());

        c->SetFilter(collider->GetFilter());
        c->SetEnabled(collider->IsEnabled());

        c->OnDestroy = collider->OnDestroy;
        c->ContactListener = collider->ContactListener;
    }

    b->SetTransform(body->transform);

    b->SetLinearVelocity(body->linearVelocity);
    b->SetAngularDamping(body->angularVelocity);

    b->SetForce(body->force);
    b->SetTorque(body->torque);

    b->SetLinearDamping(body->linearDamping);
    b->SetAngularDamping(body->angularDamping);

    b->SetFixedRotation(body->IsRotationFixed());
    b->SetContinuous(body->IsContinuous());
    b->SetSleeping(body->IsSleeping());
    b->resting = body->resting;

    b->SetEnabled(body->IsEnabled());

    b->OnDestroy = body->OnDestroy;
    b->UserData = body->UserData;

    return b;
}

RigidBody* World::CreateEmptyBody(RigidBody::Type type)
{
    void* mem = blockAllocator.Allocate(sizeof(RigidBody));
    RigidBody* body = new (mem) RigidBody(type);

    body->world = this;

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

    ++bodyCount;

    return body;
}

RigidBody* World::CreateCircle(float radius, RigidBody::Type type, float density)
{
    RigidBody* b = CreateEmptyBody(type);

    Circle circle{ radius };
    b->CreateCollider(&circle, density);

    return b;
}

RigidBody* World::CreateCapsule(float length, float radius, bool horizontal, RigidBody::Type type, float density)
{
    RigidBody* b = CreateEmptyBody(type);

    Capsule capsule{ length, radius, horizontal };
    b->CreateCollider(&capsule, density);

    return b;
}

RigidBody* World::CreateCapsule(
    const Vec2& point1, const Vec2& point2, float radius, RigidBody::Type type, bool resetPosition, float density
)
{
    RigidBody* b = CreateEmptyBody(type);

    Vec2 center = (point1 + point2) * 0.5f;
    Capsule capsule{ point1, point2, radius, true };
    b->CreateCollider(&capsule, density);

    if (resetPosition == false)
    {
        b->Translate(center);
    }

    return b;
}

RigidBody* World::CreatePolygon(std::span<Vec2> vertices, RigidBody::Type type, bool resetPosition, float radius, float density)
{
    RigidBody* b = CreateEmptyBody(type);

    Polygon polygon(vertices.data(), int32(vertices.size()), true, radius);
    b->CreateCollider(&polygon, density);

    Vec2 center{ 0.0f };
    for (size_t i = 0; i < vertices.size(); ++i)
    {
        center += vertices[i];
    }
    center *= 1.0f / vertices.size();

    if (resetPosition == false)
    {
        b->Translate(center);
    }

    return b;
}

RigidBody* World::CreateBox(float width, float height, RigidBody::Type type, float radius, float density)
{
    RigidBody* b = CreateEmptyBody(type);

    Vec2 vertices[4] = { Vec2{ 0, 0 }, Vec2{ width, 0 }, Vec2{ width, height }, Vec2{ 0, height } };
    Polygon box{ vertices, 4, true, radius };
    b->CreateCollider(&box, density);

    return b;
}

RigidBody* World::CreateBox(float size, RigidBody::Type type, float radius, float density)
{
    return CreateBox(size, size, type, radius, density);
}

RigidBody* World::CreateRandomConvexPolygon(float length, int32 vertexCount, RigidBody::Type type, float radius, float density)
{
    if (vertexCount < 3)
    {
        vertexCount = (int32)Rand(6, 12);
    }

    std::vector<float> angles;
    angles.reserve(vertexCount);

    for (int32 i = 0; i < vertexCount; ++i)
    {
        angles.push_back(Rand(0.0f, 1.0f) * (pi * 2.0f - epsilon));
    }

    std::sort(angles.begin(), angles.end());

    std::vector<Vec2> vertices;
    vertices.reserve(vertexCount);

    for (int32 i = 0; i < vertexCount; ++i)
    {
        vertices.emplace_back(Cos(angles[i]) * length, Sin(angles[i]) * length);
    }

    RigidBody* b = CreateEmptyBody(type);

    Polygon polygon{ vertices.data(), vertexCount, true, radius };
    b->CreateCollider(&polygon, density);

    return b;
}

RigidBody* World::CreateRegularPolygon(
    float length, int32 vertexCount, float initialAngle, RigidBody::Type type, float radius, float density
)
{
    if (vertexCount < 3)
    {
        vertexCount = (int32)Rand(3, 12);
    }

    float angleStart = initialAngle - pi / 2.0f;
    float angle = pi * 2.0f / vertexCount;

    std::vector<Vec2> vertices;
    vertices.reserve(vertexCount);

    for (int32 i = 0; i < vertexCount; ++i)
    {
        float currentAngle = angleStart + angle * i;

        Vec2 vertex{ Cos(currentAngle), Sin(currentAngle) };
        vertex *= length * Sqrt(2.0f);

        vertices.push_back(vertex);
    }

    RigidBody* b = CreateEmptyBody(type);

    Polygon polygon{ vertices.data(), vertexCount, true, radius };
    b->CreateCollider(&polygon, density);

    return b;
}

GrabJoint* World::CreateGrabJoint(
    RigidBody* body, const Vec2& anchor, const Vec2& target, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    if (body->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(GrabJoint));
    GrabJoint* gj = new (mem) GrabJoint(body, anchor, target, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(gj);
    return gj;
}

RevoluteJoint* World::CreateRevoluteJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec2& anchor, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(RevoluteJoint));
    RevoluteJoint* rj = new (mem) RevoluteJoint(bodyA, bodyB, anchor, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(rj);
    return rj;
}

DistanceJoint* World::CreateDistanceJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchorA,
    const Vec2& anchorB,
    float length,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(DistanceJoint));
    DistanceJoint* dj =
        new (mem) DistanceJoint(bodyA, bodyB, anchorA, anchorB, length, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(dj);
    return dj;
}

DistanceJoint* World::CreateDistanceJoint(
    RigidBody* bodyA, RigidBody* bodyB, float length, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    return CreateDistanceJoint(
        bodyA, bodyB, bodyA->GetPosition(), bodyB->GetPosition(), length, jointFrequency, jointDampingRatio, jointMass
    );
}

AngleJoint* World::CreateAngleJoint(
    RigidBody* bodyA, RigidBody* bodyB, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(AngleJoint));
    AngleJoint* aj = new (mem) AngleJoint(bodyA, bodyB, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(aj);
    return aj;
}

WeldJoint* World::CreateWeldJoint(
    RigidBody* bodyA, RigidBody* bodyB, const Vec2& anchor, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(WeldJoint));
    WeldJoint* wj = new (mem) WeldJoint(bodyA, bodyB, anchor, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(wj);
    return wj;
}

LineJoint* World::CreateLineJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchor,
    const Vec2& dir,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(LineJoint));
    LineJoint* lj = new (mem) LineJoint(bodyA, bodyB, anchor, dir, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(lj);
    return lj;
}

LineJoint* World::CreateLineJoint(
    RigidBody* bodyA, RigidBody* bodyB, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    return CreateLineJoint(
        bodyA, bodyB, bodyA->GetPosition(), Normalize(bodyB->GetPosition() - bodyA->GetPosition()), jointFrequency,
        jointDampingRatio, jointMass
    );
}

PrismaticJoint* World::CreatePrismaticJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchor,
    const Vec2& dir,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(PrismaticJoint));
    PrismaticJoint* pj = new (mem) PrismaticJoint(bodyA, bodyB, anchor, dir, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(pj);
    return pj;
}

PrismaticJoint* World::CreatePrismaticJoint(
    RigidBody* bodyA, RigidBody* bodyB, float jointFrequency, float jointDampingRatio, float jointMass
)
{
    return CreatePrismaticJoint(
        bodyA, bodyB, bodyB->GetPosition(), Normalize(bodyB->GetPosition() - bodyA->GetPosition()), jointFrequency,
        jointDampingRatio, jointMass
    );
}

PulleyJoint* World::CreatePulleyJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchorA,
    const Vec2& anchorB,
    const Vec2& groundAnchorA,
    const Vec2& groundAnchorB,
    float ratio,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(PulleyJoint));
    PulleyJoint* pj = new (mem) PulleyJoint(
        bodyA, bodyB, anchorA, anchorB, groundAnchorA, groundAnchorB, ratio, jointFrequency, jointDampingRatio, jointMass
    );

    AddJoint(pj);
    return pj;
}

MotorJoint* World::CreateMotorJoint(
    RigidBody* bodyA,
    RigidBody* bodyB,
    const Vec2& anchor,
    float maxForce,
    float maxTorque,
    float jointFrequency,
    float jointDampingRatio,
    float jointMass
)
{
    if (bodyA->world != this || bodyB->world != this)
    {
        return nullptr;
    }

    void* mem = blockAllocator.Allocate(sizeof(MotorJoint));
    MotorJoint* mj =
        new (mem) MotorJoint(bodyA, bodyB, anchor, maxForce, maxTorque, jointFrequency, jointDampingRatio, jointMass);

    AddJoint(mj);
    return mj;
}

void World::AddJoint(Joint* joint)
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
    blockAllocator.Free(body, sizeof(RigidBody));
}

void World::FreeJoint(Joint* joint)
{
    joint->~Joint();

    switch (joint->type)
    {
    case Joint::Type::grab_joint:
        blockAllocator.Free(joint, sizeof(GrabJoint));
        break;
    case Joint::Type::revolute_joint:
        blockAllocator.Free(joint, sizeof(RevoluteJoint));
        break;
    case Joint::Type::distance_joint:
        blockAllocator.Free(joint, sizeof(DistanceJoint));
        break;
    case Joint::Type::angle_joint:
        blockAllocator.Free(joint, sizeof(AngleJoint));
        break;
    case Joint::Type::weld_joint:
        blockAllocator.Free(joint, sizeof(WeldJoint));
        break;
    case Joint::Type::line_joint:
        blockAllocator.Free(joint, sizeof(LineJoint));
        break;
    case Joint::Type::prismatic_joint:
        blockAllocator.Free(joint, sizeof(PrismaticJoint));
        break;
    case Joint::Type::pulley_joint:
        blockAllocator.Free(joint, sizeof(PulleyJoint));
        break;
    case Joint::Type::motor_joint:
        blockAllocator.Free(joint, sizeof(MotorJoint));
        break;
    default:
        MuliAssert(false);
        break;
    }
}

} // namespace muli