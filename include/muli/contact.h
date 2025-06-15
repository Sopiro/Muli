#pragma once

#include "collision.h"
#include "constraint.h"
#include "contact_solver.h"
#include "position_solver.h"

namespace muli
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
public:
    Contact(Collider* colliderA, Collider* colliderB);
    ~Contact() = default;

    Collider* GetColliderA() const;
    Collider* GetColliderB() const;
    RigidBody* GetReferenceBody() const;
    RigidBody* GetIncidentBody() const;

    const Contact* GetNext() const;
    const Contact* GetPrev() const;

    bool IsTouching() const;

    const ContactManifold& GetContactManifold() const;
    int32 GetContactCount() const;
    float GetNormalImpulse(int32 index) const;
    float GetTangentImpulse(int32 index) const;

    float GetFriction() const;
    float GetRestitution() const;
    float GetRestitutionTreshold() const;
    float GetSurfaceSpeed() const;

    void SetEnabled(bool enabled);
    bool IsEnabled() const;

private:
    friend class World;
    friend class Island;
    friend class ContactGraph;
    friend class BroadPhase;
    friend class ContactSolverNormal;
    friend class ContactSolverTangent;
    friend class BlockSolver;
    friend class PositionSolver;

    enum
    {
        flag_enabled = 1,
        flag_touching = 1 << 1,
        flag_island = 1 << 2,
        flag_toi = 1 << 3,
    };

    virtual void Prepare(const Timestep& step) override;
    virtual void SolveVelocityConstraints(const Timestep& step) override;
    virtual bool SolvePositionConstraints(const Timestep& step) override;
    bool SolveTOIPositionConstraints();

    void Update();

    void SaveImpulses();
    void RestoreImpulses();

    CollideFunction* collideFunction;

    RigidBody* b1; // Reference body
    RigidBody* b2; // Incident body

    Collider* colliderA;
    Collider* colliderB;

    Contact* prev;
    Contact* next;

    ContactEdge nodeA;
    ContactEdge nodeB;

    float friction;
    float restitution;
    float restitutionThreshold;
    float surfaceSpeed;

    ContactManifold manifold;

    // TODO: Integrate decoupled solvers into SolveVelocityConstraints() and SolvePositionConstraints() for optimization
    ContactSolverNormal normalSolvers[max_contact_point_count];
    ContactSolverTangent tangentSolvers[max_contact_point_count];
    PositionSolver positionSolvers[max_contact_point_count];
    BlockSolver blockSolver;

    // Impulse buffer for position correction
    // prefix 'c' stands for corrective
    Vec2 cLinearImpulseA, cLinearImpulseB;
    float cAngularImpulseA, cAngularImpulseB;

    uint16 flag;

    int32 toiCount;
    float toi;
};

inline Collider* Contact::GetColliderA() const
{
    return colliderA;
}

inline Collider* Contact::GetColliderB() const
{
    return colliderB;
}

inline RigidBody* Contact::GetReferenceBody() const
{
    return b1;
}

inline RigidBody* Contact::GetIncidentBody() const
{
    return b2;
}

inline const Contact* Contact::GetPrev() const
{
    return prev;
}

inline const Contact* Contact::GetNext() const
{
    return next;
}

inline bool Contact::IsTouching() const
{
    return (flag & flag_touching) == flag_touching;
}

inline const ContactManifold& Contact::GetContactManifold() const
{
    return manifold;
}

inline int32 Contact::GetContactCount() const
{
    return manifold.contactCount;
}

inline float Contact::GetNormalImpulse(int32 index) const
{
    MuliAssert(index == 0 || index == 1);
    return normalSolvers[index].impulse;
}

inline float Contact::GetTangentImpulse(int32 index) const
{
    MuliAssert(index == 0 || index == 1);
    return tangentSolvers[index].impulse;
}

inline float Contact::GetFriction() const
{
    return friction;
}

inline float Contact::GetRestitution() const
{
    return restitution;
}

inline float Contact::GetRestitutionTreshold() const
{
    return restitutionThreshold;
}

inline float Contact::GetSurfaceSpeed() const
{
    return surfaceSpeed;
}

inline void Contact::SetEnabled(bool enabled)
{
    if (enabled)
    {
        flag |= flag_enabled;
    }
    else
    {
        flag &= ~flag_enabled;
    }
}

inline bool Contact::IsEnabled() const
{
    return (flag & flag_enabled) == flag_enabled;
}

inline void Contact::SaveImpulses()
{
    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        normalSolvers[i].impulseSave = normalSolvers[i].impulse;
        tangentSolvers[i].impulseSave = tangentSolvers[i].impulse;
        normalSolvers[i].impulse = 0.0f;
        tangentSolvers[i].impulse = 0.0f;
    }
}

inline void Contact::RestoreImpulses()
{
    for (int32 i = 0; i < manifold.contactCount; ++i)
    {
        normalSolvers[i].impulse = normalSolvers[i].impulseSave;
        tangentSolvers[i].impulse = tangentSolvers[i].impulseSave;
    }
}

} // namespace muli
