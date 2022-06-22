#pragma once

#include "common.h"

namespace spe
{
    class Entity
    {
    public:
        glm::vec2 position{ 0.0f };
        float rotation{ 0.0f };
        glm::vec2 scale{ 1.0f };

        Entity();
        Entity(const Entity& other) = delete;
        virtual ~Entity();

        void ResetTransform();

        void Translate(const glm::vec2& t);
        void Rotate(const float& r);
        void Scale(const glm::vec2& s);

        // Returns local to global transform
        glm::mat3 LocalToGlobal();

        // Returns global to local transform
        glm::mat3 GlobalToLocal();
    };
}