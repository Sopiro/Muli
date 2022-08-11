#include "spe/entity.h"

namespace spe
{

Entity::Entity(glm::vec2 _position, float _rotation, glm::vec2 _scale) :
    position{ std::move(_position) },
    rotation{ std::move(_rotation) },
    scale{ std::move(_scale) }
{
}

}