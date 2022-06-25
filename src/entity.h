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

        void ResetTransform();

        void Translate(const glm::vec2& t);
        void Rotate(float r);
        void Scale(const glm::vec2& s);

        // Returns local to global transform
        glm::mat3 LocalToGlobal();

        // Returns global to local transform
        glm::mat3 GlobalToLocal();
    };
}