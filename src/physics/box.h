#ifndef __BOX_H__
#define __BOX_H__

#include "polygon.h"

namespace spe
{
    class Box : public Polygon
    {
    public:
        Box(float _width, float _height, Type _type = Dynamic, float _density = DEFAULT_DENSITY);
        virtual ~Box();

        virtual void SetMass(float _mass) override;
        virtual void SetDensity(float _density) override;

        float GetWidth();
        float GetHeight();

    protected:
        float width;
        float height;
    };
}
#endif