#pragma once

#include "rigidbody.h"

namespace spe
{
    // Children: Box
    class Polygon : public RigidBody
    {
    public:
        Polygon(std::vector<glm::vec2> _vertices, Type _type = Dynamic, bool _resetPosition = true, float _density = DEFAULT_DENSITY);
        virtual ~Polygon();

        virtual void SetDensity(float d) override;
        virtual void SetMass(float m) override;

        virtual float GetArea() override final;

        const std::vector<glm::vec2>& GetVertices();
        size_t VertexCount();

    protected:
        std::vector<glm::vec2> vertices;
        float area;
    };
}