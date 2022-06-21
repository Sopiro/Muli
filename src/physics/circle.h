#ifndef __CIRCLE_H__
#define __CIRCLE_H__

#include "rigidbody.h"

namespace spe
{
    class Circle : public RigidBody
    {
    public:
        Circle(float radius, Type _type = Dynamic, float _density = DEFAULT_DENSITY);
        virtual ~Circle();

        virtual void SetMass(float m) override;
        virtual void SetDensity(float d) override;

        float GetRadius();
        virtual float GetArea() override final;

    protected:
        float radius;
        float area;
    };
}

#endif