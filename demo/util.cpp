#include "util.h"
#include "common.h"
#include "options.h"

namespace muli
{

/*
 * https://gist.github.com/ciembor/1494530
 *
 * Converts an RGB color value to HSL. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSL_color_space.
 * Assumes r, g, and b are contained in the set [0, 255] and
 * returns HSL in the set [0, 1].
 */
Vec3 RGBtoHSL(Vec3 rgb)
{
    rgb /= 255;

    float max = std::max({ rgb.x, rgb.y, rgb.z });
    float min = std::min({ rgb.x, rgb.y, rgb.z });

    Vec3 res{ (max + min) / 2 };

    if (max == min)
    {
        // achromatic
        res.x = 0;
        res.y = 0;
    }
    else
    {
        float d = max - min;
        res.x = (res.z > 0.5f) ? d / (2 - max - min) : d / (max + min);

        if (max == rgb.x)
            res.x = (rgb.y - rgb.z) / d + (rgb.y < rgb.z ? 6 : 0);
        else if (max == rgb.y)
            res.x = (rgb.z - rgb.x) / d + 2;
        else if (max == rgb.z)
            res.x = (rgb.x - rgb.y) / d + 4;

        res.x /= 6;
    }

    return res;
}

/*
 * Converts an HUE to r, g or b.
 * returns float in the set [0, 1].
 */
static float hue2rgb(float p, float q, float t)
{
    if (t < 0)
        t += 1;
    else if (t > 1)
        t -= 1;

    if (t < 1.0f / 6.0f)
        return p + (q - p) * 6 * t;
    else if (t < 1.0f / 2.0f)
        return q;
    else if (t < 2.0f / 3.0f)
        return p + (q - p) * (2.0f / 3.0f - t) * 6;

    return p;
}

/*
 * Converts an HSL color value to RGB. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSL_color_space.
 * Assumes h, s, and l are contained in the set [0, 1] and
 * returns RGB in the set [0, 1].
 */
Vec3 HSLtoRGB(Vec3 hsl)
{
    Vec3 res;

    if (hsl.y == 0)
    {
        res.x = res.y = res.z = hsl.z; // achromatic
    }
    else
    {
        float q = hsl.z < 0.5f ? hsl.z * (1 + hsl.y) : hsl.z + hsl.y - hsl.z * hsl.y;
        float p = 2 * hsl.z - q;
        res.x = hue2rgb(p, q, hsl.x + 1.0f / 3.0f);
        res.y = hue2rgb(p, q, hsl.x);
        res.z = hue2rgb(p, q, hsl.x - 1.0f / 3.0f);
    }

    return res;
}

void PrintSizes()
{
    std::cout << "Circle: " << sizeof(Circle) << '\n';
    std::cout << "Capsule: " << sizeof(Capsule) << '\n';
    std::cout << "Polygon: " << sizeof(Polygon) << '\n';
    std::cout << "Collider: " << sizeof(Collider) << '\n';
    std::cout << "RigidBody: " << sizeof(RigidBody) << '\n';
    std::cout << "Contact: " << sizeof(Contact) << '\n';
    std::cout << "Manifold: " << sizeof(ContactManifold) << '\n';
    std::cout << '\n';

    std::cout << "Angle joint: " << sizeof(AngleJoint) << '\n';
    std::cout << "Distance joint: " << sizeof(DistanceJoint) << '\n';
    std::cout << "Grab joint: " << sizeof(GrabJoint) << '\n';
    std::cout << "Line joint: " << sizeof(LineJoint) << '\n';
    std::cout << "Motor joint: " << sizeof(MotorJoint) << '\n';
    std::cout << "Prismatic joint: " << sizeof(PrismaticJoint) << '\n';
    std::cout << "Pulley joint: " << sizeof(PulleyJoint) << '\n';
    std::cout << "Revolute joint: " << sizeof(RevoluteJoint) << '\n';
    std::cout << "Weld joint: " << sizeof(WeldJoint) << '\n';
}

} // namespace muli