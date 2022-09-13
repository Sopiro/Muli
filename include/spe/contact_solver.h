#pragma once

#include "common.h"

namespace spe
{

class Contact;

enum ContactType : uint8
{
    Normal = 0,
    Tangent
};

struct Jacobian
{
    Vec2 va;
    float wa;
    Vec2 vb;
    float wb;
};

class ContactSolver
{
    friend class Contact;
    friend class BlockSolver;

public:
    enum Type : uint8
    {
        Normal = 0,
        Tangent
    };

    void Prepare(Contact* contact, uint32 index, const Vec2& dir, Type contactType);
    void Solve(const ContactSolver* normalContact = nullptr);

private:
    Contact* c;
    ContactSolver::Type type;

    Vec2 p;
    Vec2 ra;
    Vec2 rb;
    Jacobian j;

    float bias = 0.0f;
    float effectiveMass = 0.0f;
    float impulseSum = 0.0f;

    void ApplyImpulse(float lambda);
};

} // namespace spe