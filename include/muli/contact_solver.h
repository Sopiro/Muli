#pragma once

#include "common.h"

namespace muli
{

class Contact;
struct Timestep;

struct ContactJacobian
{
    Vec2 va;  // -dir
    float wa; // -ra × dir
    Vec2 vb;  //  dir
    float wb; //  rb × dir
};

class ContactSolverNormal
{
public:
    void Prepare(const Contact* contact, const Vec2& normal, int32 index, const Timestep& step);
    void Solve(Contact* contact);

private:
    friend class Contact;
    friend class ContactSolverTangent;
    friend class BlockSolver;

    ContactJacobian j;

    float m;              // effective mass
    float bias;

    float impulse = 0.0f; // impulse sum
    float impulseSave = 0.0f;
};

class ContactSolverTangent
{
public:
    void Prepare(const Contact* contact, const Vec2& tangent, int32 index, const Timestep& step);
    void Solve(Contact* contact, const ContactSolverNormal* normalContact);

private:
    friend class Contact;

    ContactJacobian j;

    float m;              // effective mass
    float bias;

    float impulse = 0.0f; // impulse sum
    float impulseSave = 0.0f;
};

class BlockSolver
{
public:
    void Prepare(const Contact* contact);
    void Solve(Contact* contact);

    bool enabled;

private:
    Mat2 k;
    Mat2 m;
};

} // namespace muli