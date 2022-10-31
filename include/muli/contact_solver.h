#pragma once

#include "common.h"

namespace muli
{

class Contact;

enum ContactType : uint8
{
    Normal = 0,
    Tangent
};

struct Jacobian
{
    Vec2 va;  // -normal
    float wa; // -ra × normal
    Vec2 vb;  //  normal
    float wb; //  rb × normal
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

    float bias;
    float m; // effective mass
    float impulseSum = 0.0f;

    void ApplyImpulse(float lambda);
};

} // namespace muli