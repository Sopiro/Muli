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
        separated
    };

    State state;
    float t;
};

void FindTimeOfImpact(const TOIInput& input, TOIOutput* output);

} // namespace muli
