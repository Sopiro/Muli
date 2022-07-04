#pragma once

#include "../common.h"
#include "constraint.h"
#include "detection.h"
#include "contact_solver.h"

namespace spe
{
    struct Settings;

    struct ContactInfo
    {
        const RigidBody* other;
        const size_t numContacts;
        const glm::vec2 contactDir;
        const std::vector<ContactPoint> contactPoints;
        const float impulse;

        ContactInfo(RigidBody* _other, size_t _numContacts, glm::vec2 _contactDir, std::vector<ContactPoint> _contactPoints, float _impulse) :
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
        friend class ContactSolver;

    public:
        ContactConstraint(const ContactManifold& manifold, const Settings& _settings);

        virtual void Prepare() override;
        virtual void Solve() override;
        void TryWarmStart(const ContactConstraint& oldCC);

        bool persistent{ false };

        ContactInfo GetContactInfo() const;

    private:
        const Settings& settings;

        std::vector<ContactPoint> contactPoints;
        float penetrationDepth;
        glm::vec2 contactNormal;
        glm::vec2 contactTangent;
        bool featureFlipped;
        size_t numContacts;

        // Solvers
        std::vector<ContactSolver> normalContacts{};
        std::vector<ContactSolver> tangentContacts{};
    };
}