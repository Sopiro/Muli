#ifndef __ENTITY_H__
#define __ENTITY_H__

#include "common.h"

class Entity
{
public:
    glm::vec2 position{ 0.0f };
    float rotation{ 0.0f };
    glm::vec2 scale{ 1.0f };

    Entity();
    virtual ~Entity();

    void ResetTransform();

    void Translate(glm::vec2 t);
    void Rotate(float r);
    void Scale(glm::vec2 s);

    // Returns local to global transform
    glm::mat3 LocalToGlobal();

    // Returns global to local transform
    glm::mat3 GlobalToLocal();
};

#endif