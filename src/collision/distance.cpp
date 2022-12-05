#include "muli/distance.h"
#include "muli/util.h"

namespace muli
{

struct GJKResult
{
    Simplex simplex;
    Vec2 direction;
    float distance;
    bool collide;
};

extern GJKResult GJK(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, bool earlyReturn);

float ComputeDistance(const Shape* a, const Transform& tfA, const Shape* b, const Transform& tfB, ClosestFeatures* features)
{
    GJKResult gr = GJK(a, tfA, b, tfB, false);

    if (gr.collide)
    {
        return 0.0f;
    }

    float r2 = a->GetRadius() + b->GetRadius();
    if (gr.distance < r2)
    {
        return 0.0f;
    }

    Simplex& simplex = gr.simplex;

    // Discard collinear case
    features->count = Min(simplex.VertexCount(), MAX_SIMPLEX_VERTEX_COUNT - 1);
    for (int32 i = 0; i < features->count; ++i)
    {
        features->fA[i] = simplex.vertices[i].pointA;
        features->fB[i] = simplex.vertices[i].pointB;
        features->fA[i].position += gr.direction * a->GetRadius();
        features->fB[i].position -= gr.direction * b->GetRadius();
    }

    return gr.distance - r2;
}

} // namespace muli
