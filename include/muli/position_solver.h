#pragma once

#include "common.h"

namespace muli
{

class Contact;

class PositionSolver
{
public:
    void Prepare(Contact* contact, int32 index);
    bool Solve();
    bool SolveTOI();

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
