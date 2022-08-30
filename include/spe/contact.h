#pragma once

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

class Contact
{
    friend class BroadPhase;
    friend class ContactManager;
    friend class World;

public:
    Contact(RigidBody* _bodyA, RigidBody* _bodyB);
    ~Contact() noexcept = default;

    Contact* GetNext() const;

private:
    RigidBody* bodyA;
    RigidBody* bodyB;

    Contact* prev;
    Contact* next;

    ContactEdge nodeA;
    ContactEdge nodeB;
};

inline Contact::Contact(RigidBody* _bodyA, RigidBody* _bodyB)
    : bodyA{ _bodyA }
    , bodyB{ _bodyB }
{
}

inline Contact* Contact::GetNext() const
{
    return next;
}

} // namespace spe
