#pragma once

#include "common.h"

namespace muli
{

class Contact;

class PositionSolver
{
public:
    void Prepare(Contact* _contact, uint32 _index);
    bool Solve();

private:
    friend class Contact;
    friend class BlockSolver;

    Contact* contact;

    Vec2 localPlainPoint;
    Vec2 localClipPoint;
    Vec2 localNormal;
};

} // namespace muli
