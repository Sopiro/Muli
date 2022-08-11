#pragma once

#include "common.h"

namespace spe
{
class Entity
{
public:
    glm::vec2 position;
    float rotation;
    glm::vec2 scale;

    Entity(glm::vec2 _position = glm::vec2{ 0.0f }, float _rotation = 0.0f, glm::vec2 _scale = glm::vec2{ 1.0f });
    virtual ~Entity() noexcept = default;

    // Non-copyable object
    Entity(const Entity&) noexcept = delete;
    Entity& operator=(const Entity&) noexcept = delete;

    Entity(Entity&&) noexcept = default;
    Entity& operator=(Entity&&) noexcept = default;

    inline void ResetTransform()
    {
        position.x = 0.0f;
        position.y = 0.0f;
        rotation = 0.0f;
        scale.x = 1.0f;
        scale.y = 1.0f;
    }

    inline void Translate(const glm::vec2& t)
    {
        position.x += t.x;
        position.y += t.y;
    }

    inline void Rotate(float r)
    {
        rotation += r;
    }

    inline void Scale(const glm::vec2& s)
    {
        scale.x *= s.x;
        scale.y *= s.y;
    }

    // Returns local to global transform
    inline glm::mat3 LocalToGlobal() const
    {
        return glm::scale(glm::rotate(glm::translate(glm::mat3{ 1.0f }, position), rotation), scale);
    }

    // Returns global to local transform
    inline glm::mat3 GlobalToLocal() const
    {
        return glm::translate(glm::rotate(glm::scale(glm::mat3{ 1.0f }, 1.0f / scale), -rotation), -position);
    }
};
}