#pragma once

#include "block_solver.h"
#include "collision.h"
#include "constraint.h"
#include "contact_solver.h"

#define MAX_CONTACT_POINT 2

namespace spe
{

inline float mix_friction(float f1, float f2)
{
    return f1 * f2;
    // return glm::sqrt(f1, f2);
}

inline float mix_restitution(float r1, float r2)
{
    return r1 * r2;
    // return glm::max(r1 * r2);
}

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
    virtual void Prepare() override;
    virtual void Solve() override;

    Contact* GetNext() const;
    Contact* GetPrev() const;

    const ContactManifold& GetContactManifold() const;
    float GetRestitution() const;
    float GetFriction() const;

private:
    Contact* prev;
    Contact* next;

    ContactEdge nodeA;
    ContactEdge nodeB;

    bool touching = false;
    bool persistent = false;

    ContactManifold manifold;

    ContactSolver normalContacts[MAX_CONTACT_POINT];
    ContactSolver tangentContacts[MAX_CONTACT_POINT];
    BlockSolver blockSolver;

    float restitution;
    float friction;
};

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

inline float Contact::GetRestitution() const
{
    return restitution;
}

inline float Contact::GetFriction() const
{
    return friction;
}

} // namespace spe
