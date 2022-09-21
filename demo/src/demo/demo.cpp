#include "demo.h"

namespace muli
{

uint32 demo_count = 0;
DemoFrame demos[MAX_DEMOS];

extern DemoFrame single_box;

static int32 init_demos()
{
    demos[demo_count++] = single_box;

    return demo_count;
}

static int32 _ = init_demos();

} // namespace muli
