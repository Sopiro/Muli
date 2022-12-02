#pragma once

#include "common.h"

namespace muli
{

class Contact;

enum ContactType
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
public:
    enum Type
    {
        normal = 0,
        tangent
    };

    void Prepare(Contact* contact, uint32 index, const Vec2& dir, Type contactType);
    void Solve(const ContactSolver* normalContact = nullptr);

private:
    friend class Contact;
    friend class BlockSolver;

    Contact* c;
    Type type;

    Vec2 p;
    Jacobian j;

    float bias;
    float m; // effective mass
    float impulseSum = 0.0f;

    void ApplyImpulse(float lambda);
};

} // namespace muli