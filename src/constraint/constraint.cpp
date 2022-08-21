#include "spe/constraint.h"
#include "spe/world.h"

namespace spe
{

Constraint::Constraint(RigidBody* _bodyA, RigidBody* _bodyB, const Settings& _settings) :
    bodyA{ _bodyA },
    bodyB{ _bodyB },
    settings{ _settings }
{

}

}