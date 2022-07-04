#pragma once

#include "util.h"
#include "polygon.h"

namespace spe
{
    class Box : public Polygon
    {
    public:
        Box(float _width, float _height, BodyType _type = Dynamic, float _density = DEFAULT_DENSITY);

        virtual void SetMass(float _mass) override;
        virtual void SetDensity(float _density) override;

        float GetWidth() const;
        float GetHeight() const;

    protected:
        float width;
        float height;
    };
}