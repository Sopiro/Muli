#include "muli/collider.h"

namespace muli
{

Collider::Collider(RigidBody* _body, Shape* _shape, float _density, float _friction, float _restitution, float _surfaceSpeed)
    : body{ _body }
    , shape{ _shape }
    , density{ _density }
    , friction{ _friction }
    , restitution{ _restitution }
    , surfaceSpeed{ _surfaceSpeed }
    , next{ nullptr }
{
}

} // namespace muli