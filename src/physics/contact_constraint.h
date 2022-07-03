#pragma once

#include "../common.h"
#include "detection.h"
#include "constraint.h"
#include "contact_solver.h"

namespace spe
{
    class ContactConstraint : public Constraint
    {
        friend class ContactSolver;

    public:
        ContactConstraint(const ContactManifold& manifold);
        
        virtual void Prepare() override;
        virtual void Solve() override;
        void TryWarmStart(const ContactConstraint& oldCC);

        bool persistent{ false };

    private:
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