#pragma once

#include "util.h"
#include "rigidbody.h"

namespace spe
{
    // Children: Box
    class Polygon : public RigidBody
    {
    public:
        Polygon(std::vector<glm::vec2> _vertices, BodyType _type = Dynamic, bool _resetPosition = true, float _density = DEFAULT_DENSITY);

        virtual void SetDensity(float d) override;
        virtual void SetMass(float m) override;

        float GetRadius() const;
        virtual float GetArea() const override final;

        const std::vector<glm::vec2>& GetVertices() const;
        size_t VertexCount() const;

    protected:
        std::vector<glm::vec2> vertices;
        float area;
        float radius;
    };
}