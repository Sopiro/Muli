#pragma once

#include "common.h"

namespace muli
{

class Contact;

class PositionSolver
{
public:
    void Prepare(Contact* contact, uint32 index);
    bool Solve();

private:
    friend class Contact;
    friend class BlockSolver;

    PositionSolver() = default;

    Contact* contact;

    Vec2 localPlainPoint;
    Vec2 localClipPoint;
    Vec2 localNormal;
};

} // namespace muli
