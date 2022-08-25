#include "spe/contact_constraint.h"
#include "spe/world.h"

namespace spe
{

ContactConstraint::ContactConstraint(const ContactManifold& manifold, const Settings& _settings) :
    Constraint(manifold.bodyA, manifold.bodyB, _settings)
{
    contactPoints = manifold.contactPoints;
    numContacts = manifold.numContacts;
    penetrationDepth = manifold.penetrationDepth;
    contactNormal = manifold.contactNormal;
    contactTangent = glm::vec2(-contactNormal.y, contactNormal.x);
    featureFlipped = manifold.featureFlipped;

    for (size_t i = 0; i < numContacts; i++)
    {
        normalContacts[i].cc = this;
        normalContacts[i].contactPoint = contactPoints[i].point;
        tangentContacts[i].cc = this;
        tangentContacts[i].contactPoint = contactPoints[i].point;
    }

    if (numContacts == 2 && settings.BLOCK_SOLVE)
    {
        blockSolver.cc = this;
    }
}

void ContactConstraint::Prepare()
{
    for (size_t i = 0; i < numContacts; i++)
    {
        normalContacts[i].Prepare(contactNormal, ContactType::Normal);
        tangentContacts[i].Prepare(contactTangent, ContactType::Tangent);
    }

    if (numContacts == 2 && settings.BLOCK_SOLVE)
    {
        blockSolver.Prepare();
    }
}

void ContactConstraint::Solve()
{
    // Solve tangential constraint first
    for (size_t i = 0; i < numContacts; i++)
    {
        tangentContacts[i].Solve(&normalContacts[i]);
    }

    if (numContacts == 1 || !settings.BLOCK_SOLVE)
    {
        for (size_t i = 0; i < numContacts; i++)
        {
            normalContacts[i].Solve();
        }
    }
    else // Solve two contact constraint in one shot using block solver
    {
        blockSolver.Solve();
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
                if (settings.APPLY_WARM_STARTING_THRESHOLD)
                {
                    float dist = glm::distance2(contactPoints[n].point, oldCC.contactPoints[o].point);
                    // If contact points are close enough, warm start.
                    // Otherwise, it means it's penetrating too deeply, skip the warm starting to prevent the overshoot
                    if (dist < settings.WARM_STARTING_THRESHOLD)
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

}