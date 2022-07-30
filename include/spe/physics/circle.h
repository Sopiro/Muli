#pragma once

#include "util.h"
#include "rigidbody.h"

namespace spe
{
class Circle : public RigidBody
{
public:
    Circle(float radius, BodyType _type = Dynamic, float _density = DEFAULT_DENSITY);

    virtual void SetMass(float m) override;
    virtual void SetDensity(float d) override;

    float GetRadius() const;
    virtual float GetArea() const override final;

protected:
    float radius;
    float area;
};
}