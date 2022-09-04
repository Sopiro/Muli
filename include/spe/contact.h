#pragma once

#include "block_solver.h"
#include "collision.h"
#include "constraint.h"
#include "contact_solver.h"
#include "position_solver.h"

#define MAX_CONTACT_POINT 2

namespace spe
{

inline float mix_friction(float f1, float f2)
{
    return glm::sqrt(f1 * f2);
}

inline float mix_restitution(float r1, float r2)
{
    return glm::max(r1, r2);
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
    friend class PositionSolver;

public:
    Contact(RigidBody* _bodyA, RigidBody* _bodyB, const Settings& _settings);
    ~Contact() noexcept = default;

    void Update();
    virtual void Prepare() override;
    virtual void Solve() override;
    void Solve2();

    Contact* GetNext() const;
    Contact* GetPrev() const;

    const ContactManifold& GetContactManifold() const;
    float GetRestitution() const;
    float GetFriction() const;
    bool IsTouching() const;

private:
    Contact* prev;
    Contact* next;

    ContactEdge nodeA;
    ContactEdge nodeB;

    float restitution;
    float friction;
    bool touching = false;
    bool persistent = false;

    ContactManifold manifold;

    ContactSolver normalSolvers[MAX_CONTACT_POINT];
    ContactSolver tangentSolvers[MAX_CONTACT_POINT];
    PositionSolver positionSolvers[MAX_CONTACT_POINT];
    BlockSolver blockSolver;

    glm::vec2 cPosA;
    float cRotA;
    glm::vec2 cPosB;
    float cRotB;
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

inline bool Contact::IsTouching() const
{
    return touching;
}

} // namespace spe
