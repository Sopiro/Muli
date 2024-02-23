#include "muli/contact_manager.h"
#include "muli/world.h"

namespace muli
{

extern void InitializeDetectionFunctionMap();

ContactManager::ContactManager(World* _world)
    : world{ _world }
    , broadPhase{ _world, this }
    , contactList{ nullptr }
    , contactCount{ 0 }
{
    InitializeDetectionFunctionMap();
}

ContactManager::~ContactManager()
{
    muliAssert(contactList == nullptr);
}

void ContactManager::EvaluateContacts()
{
    // Narrow phase
    // Evaluate contacts, prepare for solving step
    Contact* c = contactList;
    while (c)
    {
        Collider* colliderA = c->colliderA;
        Collider* colliderB = c->colliderB;

        RigidBody* bodyA = c->bodyA;
        RigidBody* bodyB = c->bodyB;

        bool activeA = bodyA->IsSleeping() == false && bodyA->GetType() != RigidBody::Type::static_body;
        bool activeB = bodyB->IsSleeping() == false && bodyB->GetType() != RigidBody::Type::static_body;

        if (activeA == false && activeB == false)
        {
            c = c->next;
            continue;
        }

        bool overlap = broadPhase.TestOverlap(colliderA, colliderB);

        // This potential contact that is configured by aabb overlap is no longer valid so destroy it
        if (overlap == false)
        {
            Contact* t = c;
            c = c->next;
            Destroy(t);
            continue;
        }

        // Evaluate the contact, prepare the solve step
        c->Update();
        c = c->next;
    }
}

void ContactManager::OnNewContact(Collider* colliderA, Collider* colliderB)
{
    RigidBody* bodyA = colliderA->body;
    RigidBody* bodyB = colliderB->body;

    muliAssert(bodyA != bodyB);
    muliAssert(colliderA->GetType() >= colliderB->GetType());

    if (bodyA->GetType() != RigidBody::Type::dynamic_body && bodyB->GetType() != RigidBody::Type::dynamic_body)
    {
        return;
    }

    if (EvaluateFilter(colliderA->GetFilter(), colliderB->GetFilter()) == false)
    {
        return;
    }

    // TODO: Use hash set to remove potential bottleneck
    ContactEdge* e = bodyB->contactList;
    while (e)
    {
        if (e->other == bodyA)
        {
            Collider* ceA = e->contact->colliderA;
            Collider* ceB = e->contact->colliderB;

            // This contact already exists
            if ((colliderA == ceA && colliderB == ceB) || (colliderA == ceB && colliderB == ceA))
            {
                return;
            }
        }

        e = e->next;
    }

    // Create new contact
    void* mem = world->blockAllocator.Allocate(sizeof(Contact));
    Contact* c = new (mem) Contact(colliderA, colliderB);

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

    c->~Contact();
    world->blockAllocator.Free(c, sizeof(Contact));
    --contactCount;
}

void ContactManager::AddCollider(Collider* collider)
{
    broadPhase.Add(collider, collider->GetAABB());
}

void ContactManager::RemoveCollider(Collider* collider)
{
    broadPhase.Remove(collider);
    collider->node = AABBTree::nullNode;

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

void ContactManager::UpdateCollider(Collider* collider, const Transform& tf)
{
    const Shape* shape = collider->GetShape();
    AABB aabb;
    shape->ComputeAABB(tf, &aabb);

    broadPhase.Update(collider, aabb, Vec2::zero);
}

void ContactManager::UpdateCollider(Collider* collider, const Transform& tf0, const Transform& tf1)
{
    const Shape* shape = collider->GetShape();

    AABB aabb0, aabb1;
    shape->ComputeAABB(tf0, &aabb0);
    shape->ComputeAABB(tf1, &aabb1);

    Vec2 prediction = aabb1.GetCenter() - aabb0.GetCenter();
    aabb1.min += prediction;
    aabb1.max += prediction;

    broadPhase.Update(collider, AABB::Union(aabb0, aabb1), prediction);
}

} // namespace muli
