#pragma once

#include "math.h"

namespace muli
{

class Contact;

class PositionSolver
{
public:
    void Prepare(Contact* contact, int32 index);
    bool Solve(Contact* contact);
    bool SolveTOI(Contact* contact);

private:
    friend class Contact;
    friend class BlockSolver;

    Vec2 localPlainPoint;
    Vec2 localClipPoint;
    Vec2 localNormal;
};

} // namespace muli
