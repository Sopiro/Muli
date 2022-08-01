#include "spe/edge.h"

namespace spe
{

Edge::Edge(glm::vec2 _p1, glm::vec2 _p2, int32_t _id1, int32_t _id2) :
    p1{ std::move(_p1) },
    p2{ std::move(_p2) },
    id1{ std::move(_id1) },
    id2{ std::move(_id2) }
{
    if (p1 == p2)
    {
        dir = glm::vec2{ 0.0f };
    }
    else
    {
        dir = glm::normalize(p2 - p1);
    }
}

}