#pragma once

#include "world.h"

namespace muli
{

class Island
{
private:
    friend class World;

    Island(World* world, int32 bodyCapacity, int32 contactCapacity, int32 jointCapacity);
    ~Island();

    void Add(RigidBody* body);
    void Add(Contact* contact);
    void Add(Joint* joint);

    void Solve();
    void SolveTOI(float dt);
    void Clear();

    World* world;

    // Static body is not included
    RigidBody** bodies;
    Contact** contacts;
    Joint** joints;

    int32 bodyCapacity;
    int32 contactCapacity;
    int32 jointCapacity;
    int32 bodyCount;
    int32 contactCount;
    int32 jointCount;

    bool sleeping;
};

inline void Island::Add(RigidBody* body)
{
    MuliAssert(bodyCount < bodyCapacity);
    body->islandIndex = bodyCount;
    bodies[bodyCount++] = body;
}

inline void Island::Add(Contact* contact)
{
    MuliAssert(contactCount < contactCapacity);
    contacts[contactCount++] = contact;
}

inline void Island::Add(Joint* joint)
{
    MuliAssert(jointCount < jointCapacity);
    joints[jointCount++] = joint;
}

inline void Island::Clear()
{
    bodyCount = 0;
    contactCount = 0;
    jointCount = 0;

    sleeping = false;
}

} // namespace muli