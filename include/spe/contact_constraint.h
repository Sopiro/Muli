#pragma once

#include "block_solver.h"
#include "common.h"
#include "constraint.h"
#include "contact_solver.h"
#include "detection.h"

namespace spe
{

struct ContactInfo
{
    RigidBody* other;
    uint32_t numContacts;
    ContactPoint contactPoints[2];
    glm::vec2 contactDir;
    float impulse;
};

class ContactConstraint : public Constraint
{
    friend class World;
    friend class ContactSolver;
    friend class BlockSolver;

public:
    ContactConstraint(const ContactManifold& manifold, const Settings& _settings);

    virtual void Prepare() override;
    virtual void Solve() override;
    void TryWarmStart(const ContactConstraint& oldCC);
    void GetContactInfo(ContactInfo* out) const;

private:
    ContactPoint contactPoints[2];
    float penetrationDepth;
    glm::vec2 contactNormal;
    glm::vec2 contactTangent;
    bool featureFlipped;
    uint32_t numContacts;
    bool persistent;

    // Solvers
    ContactSolver tangentContacts[2];
    ContactSolver normalContacts[2];
    BlockSolver blockSolver;
};

inline void ContactConstraint::GetContactInfo(ContactInfo* out) const
{
    float impulse = 0.0f;

    for (uint32_t i = 0; i < numContacts; i++)
    {
        impulse += normalContacts[i].impulseSum;
    }

    out->other = featureFlipped ? bodyB : bodyA;
    out->numContacts = numContacts;
    out->contactDir = featureFlipped ? -contactNormal : contactNormal;
    out->impulse = impulse;

    memcpy(out->contactPoints, contactPoints, numContacts * sizeof(ContactPoint));
}

} // namespace spe