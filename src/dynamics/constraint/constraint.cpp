#include "muli/constraint.h"
#include "muli/world.h"

namespace muli
{

Constraint::Constraint(RigidBody* bodyA, RigidBody* bodyB)
    : bodyA{ bodyA }
    , bodyB{ bodyB }
    , beta{ 0.0f }
    , gamma{ 0.0f }
{
    assert(bodyA->GetWorld() == bodyB->GetWorld());
}

} // namespace muli
