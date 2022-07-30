#include "spe/entity.h"

namespace spe
{

Entity::Entity(glm::vec2 _position, float _rotation, glm::vec2 _scale) :
    position{ std::move(_position) },
    rotation{ std::move(_rotation) },
    scale{ std::move(_scale) }
{

}

void Entity::ResetTransform()
{
    position.x = 0.0f;
    position.y = 0.0f;
    rotation = 0.0f;
    scale.x = 1.0f;
    scale.y = 1.0f;
}

void Entity::Translate(const glm::vec2& t)
{
    position.x += t.x;
    position.y += t.y;
}

void Entity::Rotate(float r)
{
    rotation += r;
}

void Entity::Scale(const glm::vec2& s)
{
    scale.x *= s.x;
    scale.y *= s.y;
}

glm::mat3 Entity::LocalToGlobal() const
{
    return glm::scale(glm::rotate(glm::translate(glm::mat3{ 1.0f }, position), rotation), scale);
}

glm::mat3 Entity::GlobalToLocal() const
{
    return glm::translate(glm::rotate(glm::scale(glm::mat3{ 1.0f }, 1.0f / scale), -rotation), -position);
}

}