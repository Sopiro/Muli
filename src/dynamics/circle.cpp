#include "muli/circle.h"

namespace muli
{

Circle::Circle(float _radius, Type _type, float _density)
    : RigidBody(_type, Shape::ShapeCircle)
{
    radius = _radius;
    area = MULI_PI * radius * radius;

    if (type == RigidBody::Type::Dynamic)
    {
        muliAssert(_density > 0);

        density = _density;
        mass = _density * area;
        invMass = 1.0f / mass;
        inertia = ComputeCircleInertia(radius, mass);
        invInertia = 1.0f / inertia;
    }
}

void Circle::SetMass(float _mass)
{
    muliAssert(_mass > 0);

    density = _mass / area;
    mass = _mass;
    invMass = 1.0f / mass;
    inertia = ComputeCircleInertia(radius, mass);
    invInertia = 1.0f / inertia;
}

void Circle::SetDensity(float _density)
{
    muliAssert(density > 0);

    density = _density;
    mass = density * area;
    invMass = 1.0f / mass;
    inertia = ComputeCircleInertia(radius, mass);
    invInertia = 1.0f / inertia;
}

bool Circle::RayCast(const RayCastInput& input, RayCastOutput* output) const
{
    // https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm

    Vec2 d = input.to - input.from;
    Vec2 f = input.from - transform.position;
    float r2 = radius * radius;

    float a = Dot(d, d);
    float b = 2.0f * Dot(f, d);
    float c = Dot(f, f) - r2;

    // Quadratic equation discriminant
    float discriminant = b * b - 4 * a * c;

    if (discriminant < 0.0f)
    {
        return false;
    }

    discriminant = Sqrt(discriminant);

    float t = (-b - discriminant) / (2.0f * a);
    if (t >= 0.0f && t <= 1.0f)
    {
        output->fraction = t;
        output->normal = (f + d * t).Normalized();

        return true;
    }

    return false;
}

} // namespace muli