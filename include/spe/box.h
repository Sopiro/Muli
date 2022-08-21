#pragma once

#include "util.h"
#include "polygon.h"

namespace spe
{
class Box : public Polygon
{
public:
    Box(float _width, float _height, BodyType _type = Dynamic, float _density = DEFAULT_DENSITY);
    Box(float _width, BodyType _type = Dynamic, float _density = DEFAULT_DENSITY);

    inline virtual void SetMass(float _mass) override;
    inline virtual void SetDensity(float _density) override;

    inline float GetWidth() const;
    inline float GetHeight() const;

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

// This will automatically set the inertia
inline void Box::SetMass(float _mass)
{
    assert(_mass > 0);

    density = _mass / area;
    mass = _mass;
    invMass = 1.0f / mass;
    inertia = calculate_box_inertia(width, height, mass);
    invInertia = 1.0f / inertia;
}

// This will automatically set the mass and inertia
inline void Box::SetDensity(float _density)
{
    assert(_density > 0);

    density = _density;
    mass = _density * area;
    invMass = 1.0f / mass;
    inertia = calculate_box_inertia(width, height, mass);
    invInertia = 1.0f / inertia;
}

}