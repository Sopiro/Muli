#pragma once

#include "common.h"

namespace muli
{

class Contact;
class ContactSolver;
struct Jacobian;

class BlockSolver
{
public:
    void Prepare(Contact* contact);
    void Solve();

private:
    friend class Contact;

    BlockSolver() = default;

    Contact* c;

    Mat2 k;
    Mat2 m;

    bool enabled;
};

} // namespace muli