#pragma once

#include "broad_phase.h"
#include "contact.h"

namespace muli
{
class World;

class ContactManager
{
public:
    ContactManager(World* world);
    ~ContactManager();

    void UpdateContactGraph();
    void EvaluateContacts();

    int32 GetContactCount() const;

protected:
    friend class RigidBody;

    void AddCollider(Collider* collider);
    void RemoveCollider(Collider* collider);
    void UpdateCollider(Collider* collider);

private:
    friend class World;
    friend class BroadPhase;

    World* world;

    BroadPhase broadPhase;

    Contact* contactList;
    int32 contactCount;

    void Destroy(Contact* c);
    void OnNewContact(Collider*, Collider*);
};

inline void ContactManager::UpdateContactGraph()
{
    // Find new contacts for moved objects
    // broadphase will callback OnNewContact(Collider*, Collider*)
    broadPhase.FindNewContacts();
}

inline int32 ContactManager::GetContactCount() const
{
    return contactCount;
}

} // namespace muli
