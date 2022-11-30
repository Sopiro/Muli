#pragma once

#include "collider.h"
#include "common.h"
#include "contact.h"

namespace muli
{

struct ContactListener
{
    virtual ~ContactListener() {}
    virtual void OnContactBegin(Contact* contact, Collider* other) {}
    virtual void OnContactTouching(Contact* contact, Collider* other) {}
    virtual void OnContactEnd(Contact* contact, Collider* other) {}
};

struct WorldQueryCallback
{
    virtual ~WorldQueryCallback() {}
    virtual bool OnQuery(Collider* collider) {}
};

struct RayCastAnyCallback
{
    virtual ~RayCastAnyCallback() {}
    virtual float OnHit(Collider* collider, const Vec2& point, const Vec2& normal, float fraction)
    {
        return 1.0f;
    }
};

struct RayCastClosestCallback
{
    virtual ~RayCastClosestCallback() {}
    virtual void OnHit(Collider* collider, const Vec2& point, const Vec2& normal, float fraction) {}
};

} // namespace muli
