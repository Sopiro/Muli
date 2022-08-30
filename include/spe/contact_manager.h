#pragma once

#include "broad_phase.h"
#include "contact.h"

namespace spe
{
class World;

class ContactManager
{
    friend class World;

public:
    ContactManager(World& _world);
    ~ContactManager();

    void Update(float dt);
    void ValidateContacts();
    void Reset();
    void Add(RigidBody* body);
    void Remove(RigidBody* body);

private:
    World& world;
    BroadPhase broadPhase;
    Contact* contactList = nullptr;
    uint32_t contactCount = 0;

    void Destroy(Contact* c);
};

inline ContactManager::ContactManager(World& _world)
    : world{ _world }
    , broadPhase{ _world }
{
}

inline ContactManager::~ContactManager()
{
    Reset();
}

inline void ContactManager::Add(RigidBody* body)
{
    broadPhase.Add(body);
}

inline void ContactManager::Remove(RigidBody* body)
{
    broadPhase.Remove(body);

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

} // namespace spe
