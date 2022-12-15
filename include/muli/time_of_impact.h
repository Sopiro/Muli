#pragma once

#include "distance.h"

namespace muli
{
struct TOIInput
{
    Shape* shapeA;
    Shape* shapeB;
    Sweep sweepA;
    Sweep sweepB;
    float tMax; // defines sweep interval [0, tMax]
};

struct TOIOutput
{
    enum State
    {
        unknown,
        failed,
        overlapped,
        touching,
        separated,
    };

    State state;
    float t;
};

// Bilateral advancement method by Erin Catto, the author of Box2d(https://box2d.org/)
// https://www.youtube.com/watch?v=7_nKOET6zwI
void ComputeTimeOfImpact(const TOIInput& input, TOIOutput* output);

} // namespace muli
