#include "contact_constraint.h"

using namespace spe;

ContactConstraint::ContactConstraint(const ContactManifold& manifold) :
    Constraint(manifold.bodyA, manifold.bodyB)
{
    contactPoints = manifold.contactPoints;
    penetrationDepth = manifold.penetrationDepth;
    contactNormal = manifold.contactNormal;
    contactTangent = glm::vec2(-contactNormal.y, contactNormal.x);
    featureFlipped = manifold.featureFlipped;

    numContacts = contactPoints.size();

    normalContacts.reserve(numContacts);
    tangentContacts.reserve(numContacts);

    for (size_t i = 0; i < numContacts; i++)
    {
        normalContacts.emplace_back(*this, contactPoints[i].point);
        tangentContacts.emplace_back(*this, contactPoints[i].point);
    }

    // Block solver
}

void ContactConstraint::Prepare()
{
    for (size_t i = 0; i < numContacts; i++)
    {
        normalContacts[i].Prepare(contactNormal, Normal);
        tangentContacts[i].Prepare(contactTangent, Tangent);
    }
}

void ContactConstraint::Solve()
{
    // Solve tangential constraint first
    for (size_t i = 0; i < numContacts; i++)
    {
        tangentContacts[i].Solve(&normalContacts[i]);
    }

    for (size_t i = 0; i < numContacts; i++)
    {
        normalContacts[i].Solve();
    }
}

void ContactConstraint::TryWarmStart(const ContactConstraint& oldCC)
{
    for (size_t n = 0; n < numContacts; n++)
    {
        size_t o = 0;
        for (; o < oldCC.numContacts; o++)
        {
            if (contactPoints[n].id == oldCC.contactPoints[o].id)
            {
                if (APPLY_WARM_STARTING_THRESHOLD)
                {
                    float dist = glm::distance2(contactPoints[n].point, oldCC.contactPoints[o].point);
                    // If contact points are close enough, warm start.
                    // Otherwise, it means it's penetrating too deeply, skip the warm starting to prevent the overshoot
                    if (dist < WARM_STARTING_THRESHOLD)
                        break;
                }
                else
                {
                    break;
                }
            }
        }

        if (o < oldCC.numContacts)
        {
            normalContacts[n].impulseSum = oldCC.normalContacts[o].impulseSum;
            tangentContacts[n].impulseSum = oldCC.tangentContacts[o].impulseSum;

            persistent = true;
        }
    }
}

ContactInfo ContactConstraint::GetContactInfo() const
{
    float impulse = 0.0f;

    for (size_t i = 0; i < numContacts; i++)
    {
        impulse += normalContacts[i].impulseSum;
    }

    return ContactInfo
    {
        featureFlipped ? bodyB : bodyA,
        numContacts,
        featureFlipped ? -contactNormal : contactNormal,
        contactPoints,
        impulse
    };
}
