#pragma once

#include "common.h"

namespace spe
{
class ContactConstraint;

enum ContactType : uint8_t
{
    Normal = 0,
    Tangent
};

struct Jacobian
{
    glm::vec2 va;
    float wa;
    glm::vec2 vb;
    float wb;
};

class ContactSolver
{
    friend class ContactConstraint;
    friend class BlockSolver;

public:
    ContactSolver(ContactConstraint& _cc, const glm::vec2& _contactPoint);

    void Prepare(const glm::vec2& dir, ContactType contactType);
    void Solve(const ContactSolver* normalContact = nullptr);

private:
    ContactConstraint& cc;
    glm::vec2 contactPoint;
    ContactType contactType;

    glm::vec2 ra;
    glm::vec2 rb;

    Jacobian jacobian;
    float bias = 0.0f;
    float effectiveMass = 0.0f;
    float impulseSum = 0.0f;

    float beta;
    float restitution;
    float friction;

    void ApplyImpulse(float lambda);
};
}