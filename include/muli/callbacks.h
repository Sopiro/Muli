#pragma once

#include "collider.h"
#include "common.h"
#include "contact.h"
#include "joint.h"

namespace muli
{

class ColliderDestoryCallback
{
public:
    virtual ~ColliderDestoryCallback() {}
    virtual void OnDestroy(Collider* me) = 0;
};

class BodyDestoryCallback
{
public:
    virtual ~BodyDestoryCallback() {}
    virtual void OnDestroy(RigidBody* me) = 0;
};

class JointDestoryCallback
{
public:
    virtual ~JointDestoryCallback() {}
    virtual void OnDestroy(Joint* me) = 0;
};

class ContactListener
{
public:
    virtual ~ContactListener() {}
    virtual void OnContactBegin(Collider* me, Collider* other, const Contact* contact) {}
    virtual void OnContactTouching(Collider* me, Collider* other, const Contact* contact) {}
    virtual void OnContactEnd(Collider* me, Collider* other, const Contact* contact) {}
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
    virtual float OnHit(Collider* collider, const Vec2& point, const Vec2& normal, float fraction) = 0;
};

class RayCastClosestCallback
{
public:
    virtual ~RayCastClosestCallback() {}
    virtual void OnHit(Collider* collider, const Vec2& point, const Vec2& normal, float fraction) = 0;
};

} // namespace muli
