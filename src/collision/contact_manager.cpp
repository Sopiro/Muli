#include "muli/contact_manager.h"
#include "muli/util.h"
#include "muli/world.h"

namespace muli
{

void ContactManager::Update(float dt)
{
    // Update the dynamic AABB tree node
    broadPhase.UpdateDynamicTree(dt);

    // Find contacts, insert into the contact graph
    broadPhase.FindContacts([&](RigidBody* bodyA, RigidBody* bodyB) -> void {
        // TODO: Use hash set to remove potential bottleneck
        ContactEdge* e = bodyB->contactList;
        while (e)
        {
            // This contact already exists
            if (e->other == bodyA) return;
            e = e->next;
        }

        // Create new contact
        Contact* c = new Contact(bodyA, bodyB, world.settings);

        // Insert into the world
        c->prev = nullptr;
        c->next = contactList;
        if (contactList != nullptr)
        {
            contactList->prev = c;
        }
        contactList = c;

        // Connect to island graph

        // Connect contact edge to body A
        c->nodeA.contact = c;
        c->nodeA.other = bodyB;

        c->nodeA.prev = nullptr;
        c->nodeA.next = bodyA->contactList;
        if (bodyA->contactList != nullptr)
        {
            bodyA->contactList->prev = &c->nodeA;
        }
        bodyA->contactList = &c->nodeA;

        // Connect contact edge to body B
        c->nodeB.contact = c;
        c->nodeB.other = bodyA;

        c->nodeB.prev = nullptr;
        c->nodeB.next = bodyB->contactList;
        if (bodyB->contactList != nullptr)
        {
            bodyB->contactList->prev = &c->nodeB;
        }
        bodyB->contactList = &c->nodeB;

        ++contactCount;
    });

    // Narrow phase
    // Evaluate contacts, prepare for solving step
    Contact* c = contactList;
    while (c)
    {
        RigidBody* bodyA = c->bodyA;
        RigidBody* bodyB = c->bodyB;

        bool activeA = bodyA->IsSleeping() == false && bodyA->GetType() != RigidBody::Type::Static;
        bool activeB = bodyB->IsSleeping() == false && bodyB->GetType() != RigidBody::Type::Static;

        if (!activeA && !activeB)
        {
            c = c->GetNext();
            continue;
        }

        bool overlap = broadPhase.TestOverlap(bodyA, bodyB);

        if (!overlap)
        {
            Contact* t = c;
            c = c->GetNext();
            Destroy(t);
            continue;
        }

        c->Update();
        c = c->GetNext();
    }
}

void ContactManager::Destroy(Contact* c)
{
    RigidBody* bodyA = c->bodyA;
    RigidBody* bodyB = c->bodyB;

    // Remove from the world
    if (c->prev) c->prev->next = c->next;
    if (c->next) c->next->prev = c->prev;
    if (c == contactList) contactList = c->next;

    // Remove from bodyA
    if (c->nodeA.prev) c->nodeA.prev->next = c->nodeA.next;
    if (c->nodeA.next) c->nodeA.next->prev = c->nodeA.prev;
    if (&c->nodeA == bodyA->contactList) bodyA->contactList = c->nodeA.next;

    // Remove from bodyB
    if (c->nodeB.prev) c->nodeB.prev->next = c->nodeB.next;
    if (c->nodeB.next) c->nodeB.next->prev = c->nodeB.prev;
    if (&c->nodeB == bodyB->contactList) bodyB->contactList = c->nodeB.next;

    --contactCount;
    delete c;
}

void ContactManager::Reset()
{
    broadPhase.Reset();

    Contact* c = contactList;
    while (c)
    {
        Contact* c0 = c;
        c = c->GetNext();
        delete c0; // Destroy(c0);
    }
    contactList = nullptr;

    contactCount = 0;
}

} // namespace muli
