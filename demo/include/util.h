#include "common.h"
#include "mesh.h"

namespace muli
{

std::unique_ptr<Mesh> GenerateMesh(const Collider* body, int32 circlePolygonCount = 13);

// Modified Ear-clipping algorithm, only works for convex geometry
std::vector<int32> Triangulate(const std::vector<Vec2>& vertices);

/*
 * https://gist.github.com/ciembor/1494530
 *
 * Converts an RGB color value to HSL. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSL_color_space.
 * Assumes r, g, and b are contained in the set [0, 255] and
 * returns HSL in the set [0, 1].
 */
Vec3 rgb2hsl(float r, float g, float b);

/*
 * Converts an HUE to r, g or b.
 * returns float in the set [0, 1].
 */
float hue2rgb(float p, float q, float t);

/*
 * Converts an HSL color value to RGB. Conversion formula
 * adapted from http://en.wikipedia.org/wiki/HSL_color_space.
 * Assumes h, s, and l are contained in the set [0, 1] and
 * returns RGB in the set [0, 1].
 */
Vec3 hsl2rgb(float h, float s, float l);

} // namespace muli