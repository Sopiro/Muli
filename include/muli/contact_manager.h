#pragma once

#include "broad_phase.h"
#include "contact.h"

namespace muli
{
class World;

class ContactGraph
{
public:
    ContactGraph(World* world);
    ~ContactGraph();

    void UpdateContactGraph();
    void EvaluateContacts();

    int32 GetContactCount() const;

protected:
    friend class RigidBody;

    void AddCollider(Collider* collider);
    void RemoveCollider(Collider* collider);
    void UpdateCollider(Collider* collider, const Transform& tf);
    void UpdateCollider(Collider* collider, const Transform& tf0, const Transform& tf1);

private:
    friend class World;
    friend class BroadPhase;

    World* world;

    // Broad phase accelerator to quickly identify potentially colliding pairs
    BroadPhase broadPhase;

    // Linked list of potential contacts (i.e. collider pairs whose AABBs overlap)
    // These pairs must be evaluated by EvaluateContacts() to determine actual collision pairs
    Contact* contactList;
    int32 contactCount;

    void Destroy(Contact* c);
    void OnNewContact(Collider*, Collider*);
};

inline void ContactGraph::UpdateContactGraph()
{
    // Find new contacts for moved objects
    // broadphase will callback OnNewContact(Collider*, Collider*)
    broadPhase.FindNewContacts();
}

inline int32 ContactGraph::GetContactCount() const
{
    return contactCount;
}

} // namespace muli
