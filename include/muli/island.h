#pragma once

#include "world.h"

namespace muli
{

class Island
{
private:
    friend class World;

    Island(World& _world, uint32 _bodyCapacity, uint32 _contactCapacity, uint32 _jointCapacity);
    ~Island();

    void Add(RigidBody* body);
    void Add(Contact* contact);
    void Add(Joint* joint);

    void Solve();
    void Clear();

    World& world;

    // Static body is not included
    RigidBody** bodies;
    Contact** contacts;
    Joint** joints;

    uint32 bodyCapacity;
    uint32 contactCapacity;
    uint32 jointCapacity;
    uint32 bodyCount;
    uint32 contactCount;
    uint32 jointCount;

    bool sleeping;
};

inline void Island::Add(RigidBody* body)
{
    muliAssert(bodyCount < bodyCapacity);
    bodies[bodyCount++] = body;
}

inline void Island::Add(Contact* contact)
{
    muliAssert(contactCount < contactCapacity);
    contacts[contactCount++] = contact;
}

inline void Island::Add(Joint* joint)
{
    muliAssert(jointCount < jointCapacity);
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