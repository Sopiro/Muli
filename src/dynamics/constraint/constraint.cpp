#include "muli/constraint.h"
#include "muli/world.h"

namespace muli
{

Constraint::Constraint(RigidBody* _bodyA, RigidBody* _bodyB)
    : bodyA{ _bodyA }
    , bodyB{ _bodyB }
    , settings{ _bodyA->GetWorld()->GetWorldSettings() }
    , beta{ 0.0f }
    , gamma{ 0.0f }
{
    assert(bodyA->GetWorld() == bodyB->GetWorld());
}

} // namespace muli
