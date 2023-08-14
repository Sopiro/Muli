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
        muliNotUsed(me);
        muliNotUsed(other);
        muliNotUsed(contact);
    }
    virtual void OnContactTouching(Collider* me, Collider* other, Contact* contact)
    {
        muliNotUsed(me);
        muliNotUsed(other);
        muliNotUsed(contact);
    }
    virtual void OnContactEnd(Collider* me, Collider* other, Contact* contact)
    {
        muliNotUsed(me);
        muliNotUsed(other);
        muliNotUsed(contact);
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
    virtual float OnHitAny(Collider* collider, const Vec2& point, const Vec2& normal, float fraction) = 0;
};

class RayCastClosestCallback
{
public:
    virtual ~RayCastClosestCallback() {}
    virtual void OnHitClosest(Collider* collider, const Vec2& point, const Vec2& normal, float fraction) = 0;
};

} // namespace muli
