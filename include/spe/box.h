#pragma once

#include "polygon.h"
#include "util.h"

namespace spe
{

class Box : public Polygon
{
public:
    Box(float _width,
        float _height,
        RigidBody::Type _type = Dynamic,
        float _radius = DEFAULT_RADIUS,
        float _density = DEFAULT_DENSITY);
    Box(float _size, RigidBody::Type _type = Dynamic, float _radius = DEFAULT_RADIUS, float _density = DEFAULT_DENSITY);

    virtual void SetMass(float _mass) override;
    virtual void SetDensity(float _density) override;

    float GetWidth() const;
    float GetHeight() const;

protected:
    float width;
    float height;
};

inline float Box::GetWidth() const
{
    return width;
}

inline float Box::GetHeight() const
{
    return height;
}

} // namespace spe