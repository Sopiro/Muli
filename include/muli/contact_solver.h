#pragma once

#include "common.h"

namespace muli
{

class Contact;
struct Timestep;

class ContactSolver
{
public:
    struct Jacobian
    {
        Vec2 va;  // -normal
        float wa; // -ra × normal
        Vec2 vb;  //  normal
        float wb; //  rb × normal
    };

    enum Type
    {
        normal,
        tangent,
    };

    void Prepare(Contact* contact, Type contactType, const Vec2& dir, int32 index, const Timestep& step);
    void Solve(const ContactSolver* normalContact = nullptr);

private:
    friend class Contact;
    friend class BlockSolver;

    Contact* c;
    Type type;

    Jacobian j;

    float bias;
    float m;              // effective mass

    float impulse = 0.0f; // impluse sum
    float impulseSave = 0.0f;
};

} // namespace muli