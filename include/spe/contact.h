#pragma once

#include "block_solver.h"
#include "collision.h"
#include "constraint.h"
#include "contact_solver.h"

#define MAX_CONTACT_POINT 2

namespace spe
{
class RigidBody;
class Contact;

struct ContactEdge
{
    RigidBody* other;
    Contact* contact;
    ContactEdge* prev;
    ContactEdge* next;
};

class Contact : Constraint
{
    friend class World;
    friend class ContactManager;
    friend class BroadPhase;
    friend class ContactSolver;
    friend class BlockSolver;

public:
    Contact(RigidBody* _bodyA, RigidBody* _bodyB, const Settings& _settings);
    ~Contact() noexcept = default;

    void Update();
    Contact* GetNext() const;
    Contact* GetPrev() const;

    virtual void Prepare() override;
    virtual void Solve() override;

    const ContactManifold& GetContactManifold() const;

private:
    Contact* prev;
    Contact* next;

    ContactEdge nodeA;
    ContactEdge nodeB;

    bool touching = false;
    bool persistent = false;

    ContactManifold manifold;

    ContactSolver tangentContacts[MAX_CONTACT_POINT];
    ContactSolver normalContacts[MAX_CONTACT_POINT];
    BlockSolver blockSolver;
};

inline Contact::Contact(RigidBody* _bodyA, RigidBody* _bodyB, const Settings& _settings)
    : Constraint(_bodyA, _bodyB, _settings)
{
    manifold.numContacts = 0;
}

inline Contact* Contact::GetPrev() const
{
    return prev;
}

inline Contact* Contact::GetNext() const
{
    return next;
}

inline const ContactManifold& Contact::GetContactManifold() const
{
    return manifold;
}

} // namespace spe
