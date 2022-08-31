#pragma once

#include "common.h"

namespace spe
{

class Contact;

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
    friend class Contact;
    friend class BlockSolver;

public:
    void Prepare(const glm::vec2& dir, ContactType contactType);
    void Solve(const ContactSolver* normalContact = nullptr);

private:
    Contact* contact;
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

} // namespace spe