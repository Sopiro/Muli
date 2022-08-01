#pragma once

#include "common.h"

namespace spe
{
class ContactConstraint;
class ContactSolver;
struct Jacobian;

class BlockSolver
{
public:
    BlockSolver(ContactConstraint& _cc);

    void Prepare();
    void Solve();

private:
    ContactConstraint& cc;

    ContactSolver* nc1;
    ContactSolver* nc2;

    Jacobian* j1;
    Jacobian* j2;

    glm::mat2 k;
    glm::mat2 m;

    void ApplyImpulse(const glm::vec2& lambda);
};
}