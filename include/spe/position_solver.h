#pragma once

#include "common.h"

namespace spe
{

class Contact;

class PositionSolver
{
    friend class Contact;
    friend class BlockSolver;

public:
    void Prepare(Contact* contact, uint32_t index);
    void Solve();

private:
    Contact* c;

    glm::vec2 localPA;
    glm::vec2 localPB;
    glm::vec2 localNormal;
};

} // namespace spe
