#pragma once

#include "broad_phase.h"
#include "contact.h"

namespace muli
{
class World;
extern void InitializeDetectionFunctionMap();

class ContactManager
{
public:
    ContactManager(World& _world);
    ~ContactManager();

    void Update(float dt);
    void Add(Collider* collider);
    void Remove(Collider* collider);
    void Reset();
    uint32 GetContactCount() const;

private:
    friend class World;

    World& world;
    BroadPhase broadPhase;
    Contact* contactList = nullptr;
    uint32 contactCount = 0;

    void Destroy(Contact* c);
};

inline ContactManager::ContactManager(World& _world)
    : world{ _world }
    , broadPhase{ _world }
{
    InitializeDetectionFunctionMap();
}

inline ContactManager::~ContactManager()
{
    Reset();
}

inline void ContactManager::Add(Collider* collider)
{
    broadPhase.Add(collider);
}

inline void ContactManager::Remove(Collider* collider)
{
    broadPhase.Remove(collider);

    RigidBody* body = collider->body;

    ContactEdge* ce = body->contactList;
    while (ce)
    {
        ContactEdge* ce0 = ce;
        ce = ce->next;
        ce0->other->Awake();
        Destroy(ce0->contact);
    }
    body->contactList = nullptr;
}

inline uint32 ContactManager::GetContactCount() const
{
    return contactCount;
}

} // namespace muli
