#include "spe/aabb.h"

namespace spe
{

AABB::AABB(glm::vec2 _min, glm::vec2 _max) :
    min{ std::move(_min) },
    max{ std::move(_max) }
{
    //fix(*this);
}

AABB spe::create_AABB(RigidBody* body, float margin)
{
    auto& bodyType = typeid(*body);

    if (bodyType == typeid(Circle))
    {
        Circle* c = static_cast<Circle*>(body);

        float radius = c->GetRadius();

        return AABB
        {
            glm::vec2(body->position.x - radius - margin, body->position.y - radius - margin),
            glm::vec2(body->position.x + radius + margin, body->position.y + radius + margin)
        };
    }
    else if (bodyType == typeid(Polygon) || bodyType == typeid(Box))
    {
        Polygon* p = static_cast<Polygon*>(body);

        glm::mat3 localToGlobal = p->LocalToGlobal();
        const std::vector<glm::vec2>& vertices = p->GetVertices();

        AABB res{ localToGlobal * vertices[0], localToGlobal * vertices[1] };
        fix(res);

        for (size_t i = 1; i < p->VertexCount(); i++)
        {
            glm::vec2 gv = localToGlobal * vertices[i];

            if (gv.x < res.min.x)
                res.min.x = gv.x;
            else if (gv.x > res.max.x)
                res.max.x = gv.x;
            if (gv.y < res.min.y)
                res.min.y = gv.y;
            else if (gv.y > res.max.y)
                res.max.y = gv.y;
        }

        res.min.x -= margin;
        res.min.y -= margin;
        res.max.x += margin;
        res.max.y += margin;

        return res;
    }
    else
    {
        throw std::exception("Not a supported shape");
    }
}

}