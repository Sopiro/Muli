#pragma once

#include "collider.h"
#include "common.h"
#include "contact.h"
#include "joint.h"

namespace muli
{

class ColliderDestroyCallback
{
public:
    virtual ~ColliderDestroyCallback() {}
    virtual void OnColliderDestroy(Collider* me) = 0;
};

class BodyDestroyCallback
{
public:
    virtual ~BodyDestroyCallback() {}
    virtual void OnBodyDestroy(RigidBody* me) = 0;
};

class JointDestroyCallback
{
public:
    virtual ~JointDestroyCallback() {}
    virtual void OnJointDestroy(Joint* me) = 0;
};

class ContactListener
{
public:
    virtual ~ContactListener() {}

    virtual void OnContactBegin(Collider* me, Collider* other, Contact* contact)
    {
        MuliNotUsed(me);
        MuliNotUsed(other);
        MuliNotUsed(contact);
    }

    virtual void OnContactTouching(Collider* me, Collider* other, Contact* contact)
    {
        MuliNotUsed(me);
        MuliNotUsed(other);
        MuliNotUsed(contact);
    }

    virtual void OnContactEnd(Collider* me, Collider* other, Contact* contact)
    {
        MuliNotUsed(me);
        MuliNotUsed(other);
        MuliNotUsed(contact);
    }

    virtual void OnPreSolve(Collider* me, Collider* other, Contact* contact)
    {
        MuliNotUsed(me);
        MuliNotUsed(other);
        MuliNotUsed(contact);
    }

    virtual void OnPostSolve(Collider* me, Collider* other, Contact* contact)
    {
        MuliNotUsed(me);
        MuliNotUsed(other);
        MuliNotUsed(contact);
    }
};

class WorldQueryCallback
{
public:
    virtual ~WorldQueryCallback() {}
    virtual bool OnQuery(Collider* collider) = 0;
};

class RayCastAnyCallback
{
public:
    virtual ~RayCastAnyCallback() {}
    virtual float OnHitAny(Collider* collider, Vec2 point, Vec2 normal, float fraction) = 0;
};

class RayCastClosestCallback
{
public:
    virtual ~RayCastClosestCallback() {}
    virtual void OnHitClosest(Collider* collider, Vec2 point, Vec2 normal, float fraction) = 0;
};

class ShapeCastAnyCallback
{
public:
    virtual ~ShapeCastAnyCallback() {}
    virtual float OnHitAny(Collider* collider, Vec2 point, Vec2 normal, float t) = 0;
};

class ShapeCastClosestCallback
{
public:
    virtual ~ShapeCastClosestCallback() {}
    virtual void OnHitClosest(Collider* collider, Vec2 point, Vec2 normal, float t) = 0;
};

} // namespace muli
