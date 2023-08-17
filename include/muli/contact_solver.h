#pragma once

#include "common.h"

namespace muli
{

class Contact;

struct Jacobian
{
    Vec2 va;  // -normal
    float wa; // -ra × normal
    Vec2 vb;  //  normal
    float wb; //  rb × normal
};

class ContactSolver
{
public:
    enum Type
    {
        normal,
        tangent,
    };

    void Prepare(Contact* contact, int32 index, const Vec2& dir, Type contactType);
    void Solve(const ContactSolver* normalContact = nullptr);

private:
    friend class Contact;
    friend class BlockSolver;

    ContactSolver() = default;

    Contact* c;
    Type type;

    Vec2 p;
    Jacobian j;

    float bias;
    float m;              // effective mass

    float impulse = 0.0f; // impluse sum
    float impulseSave = 0.0f;
};

} // namespace muli