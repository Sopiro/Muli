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
    // Find contacts, insert into the contact graph
    // broadphase will callback OnNewContact()
    broadPhase.FindContacts();
}

inline void ContactManager::AddCollider(Collider* collider)
{
    broadPhase.Add(collider, collider->GetAABB());
}

inline void ContactManager::RemoveCollider(Collider* collider)
{
    broadPhase.Remove(collider);

    RigidBody* body = collider->body;

    // Destroy any contacts associated with the collider
    ContactEdge* edge = body->contactList;
    while (edge)
    {
        Contact* contact = edge->contact;
        edge = edge->next;

        Collider* colliderA = contact->GetColliderA();
        Collider* colliderB = contact->GetColliderB();

        if (collider == colliderA || collider == colliderB)
        {
            Destroy(contact);

            colliderA->body->Awake();
            colliderB->body->Awake();
        }
    }
}

inline int32 ContactManager::GetContactCount() const
{
    return contactCount;
}

} // namespace muli
