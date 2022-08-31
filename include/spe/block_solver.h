#pragma once

#include "common.h"

namespace spe
{

class Contact;
class ContactSolver;
struct Jacobian;

class BlockSolver
{
    friend class Contact;

public:
    void Prepare();
    void Solve();

private:
    Contact* c;

    ContactSolver* nc1;
    ContactSolver* nc2;

    Jacobian* j1;
    Jacobian* j2;

    glm::mat2 k;
    glm::mat2 m;

    void ApplyImpulse(const glm::vec2& lambda);
};

} // namespace spe