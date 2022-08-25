#pragma once

#include "common.h"
#include "constraint.h"
#include "detection.h"
#include "contact_solver.h"
#include "block_solver.h"

namespace spe
{
struct ContactInfo
{
    const RigidBody* other;
    const size_t numContacts;
    const glm::vec2 contactDir;
    const std::array<ContactPoint, 2> contactPoints;
    const float impulse;

    ContactInfo(RigidBody* _other, size_t _numContacts, glm::vec2 _contactDir, std::array<ContactPoint, 2> _contactPoints, float _impulse) :
        other{ _other },
        numContacts{ _numContacts },
        contactDir{ _contactDir },
        contactPoints{ _contactPoints },
        impulse{ _impulse }
    {
    }
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

    bool persistent{ false };

    ContactInfo GetContactInfo() const;

private:
    std::array<ContactPoint, 2> contactPoints;
    float penetrationDepth;
    glm::vec2 contactNormal;
    glm::vec2 contactTangent;
    bool featureFlipped;
    size_t numContacts;

    // Solvers
    std::array<ContactSolver, 2> tangentContacts;
    std::array<ContactSolver, 2> normalContacts;
    BlockSolver blockSolver;
};
}