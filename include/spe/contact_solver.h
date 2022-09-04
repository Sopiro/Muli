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
    enum Type : uint8_t
    {
        Normal = 0,
        Tangent
    };

    void Prepare(Contact* contact, uint32_t index, const glm::vec2& dir, Type contactType);
    void Solve(const ContactSolver* normalContact = nullptr);

private:
    Contact* c;
    ContactSolver::Type type;

    glm::vec2 p;
    glm::vec2 ra;
    glm::vec2 rb;
    Jacobian j;

    float bias = 0.0f;
    float effectiveMass = 0.0f;
    float impulseSum = 0.0f;

    void ApplyImpulse(float lambda);
};

} // namespace spe