#pragma once

#include "rigidbody.h"
#include "util.h"

namespace spe
{

// Children: Box
class Polygon : public RigidBody
{
public:
    Polygon(std::vector<glm::vec2> _vertices,
            RigidBody::Type _type = Dynamic,
            bool _resetPosition = true,
            float _density = DEFAULT_DENSITY);

    virtual void SetDensity(float d) override;
    virtual void SetMass(float m) override;

    float GetRadius() const;
    virtual float GetArea() const override final;
    virtual AABB GetAABB() const override;

    const std::vector<glm::vec2>& GetVertices() const;
    size_t VertexCount() const;

protected:
    std::vector<glm::vec2> vertices;
    float area;
    float radius;
};

inline float Polygon::GetRadius() const
{
    return radius;
}

inline float Polygon::GetArea() const
{
    return area;
}

inline const std::vector<glm::vec2>& Polygon::GetVertices() const
{
    return vertices;
}

inline size_t Polygon::VertexCount() const
{
    return vertices.size();
}

inline AABB Polygon::GetAABB() const
{
    const Transform& localToGlobal = GetTransform();

    glm::vec2 min = localToGlobal * vertices[0];
    glm::vec2 max = min;

    for (size_t i = 1; i < vertices.size(); i++)
    {
        glm::vec2 v = localToGlobal * vertices[i];

        min = glm::min(min, v);
        max = glm::max(max, v);
    }

    return AABB{ min, max };
}

} // namespace spe