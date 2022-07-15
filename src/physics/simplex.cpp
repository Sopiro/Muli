#include "spe/physics/simplex.h"
#include "spe/physics/util.h"

using namespace spe;

// Returns the closest point to the input q
ClosestResult Simplex::GetClosest(const glm::vec2& q) const
{
    std::vector<uint32_t> contributors{};
    contributors.reserve(3);

    switch (vertices.size())
    {
    case 1: // 0-Simplex: Point
    {
        contributors.push_back(0);

        return { vertices[0], contributors };
    }
    case 2: // 1-Simplex: Line segment
    {
        const glm::vec2 a = vertices[0];
        const glm::vec2 b = vertices[1];
        const uv w = get_uv(a, b, q);

        if (w.v <= 0)
        {
            contributors.push_back(0);
            return { a, contributors };
        }
        else if (w.v >= 1)
        {
            contributors.push_back(1);
            return { b, contributors };
        }
        else
        {
            contributors.push_back(0);
            contributors.push_back(1);

            return { lerp_vector(a, b, w), contributors };
        }
    }
    case 3: // 2-Simplex: Triangle
    {
        const glm::vec2 a = vertices[0];
        const glm::vec2 b = vertices[1];
        const glm::vec2 c = vertices[2];

        const uv wab = get_uv(a, b, q);
        const uv wbc = get_uv(b, c, q);
        const uv wca = get_uv(c, a, q);

        if (wca.u <= 0 && wab.v <= 0) // A area
        {
            contributors.push_back(0);
            return { a, contributors };
        }
        else if (wab.u <= 0 && wbc.v <= 0) // B area
        {
            contributors.push_back(1);
            return { b, contributors };
        }
        else if (wbc.u <= 0 && wca.v <= 0) // C area
        {
            contributors.push_back(2);
            return { c, contributors };
        }

        const float area = glm::cross(b - a, c - a);

        // If area == 0, 3 vertices are in collinear position, which means all aligned in a line

        const float u = glm::cross(b - q, c - q);
        const float v = glm::cross(c - q, a - q);
        const float w = glm::cross(a - q, b - q);

        if (wab.u > 0 && wab.v > 0 && w * area <= 0) // On the AB edge
        {
            if (area != 0.0f)
            {
                contributors.push_back(0);
                contributors.push_back(1);
            }
            else
            {
                contributors.push_back(0);
                contributors.push_back(1);
                contributors.push_back(2);
            }

            return { lerp_vector(a, b, wab), contributors };
        }
        else if (wbc.u > 0 && wbc.v > 0 && u * area <= 0) // On the BC edge
        {
            if (area != 0.0f)
            {
                contributors.push_back(1);
                contributors.push_back(2);
            }
            else
            {
                contributors.push_back(0);
                contributors.push_back(1);
                contributors.push_back(2);
            }

            return { lerp_vector(b, c, wbc), contributors };
        }
        else if (wca.u > 0 && wca.v > 0 && v * area <= 0) // On the CA edge
        {
            if (area != 0.0f)
            {
                contributors.push_back(2);
                contributors.push_back(0);
            }
            else
            {
                contributors.push_back(0);
                contributors.push_back(1);
                contributors.push_back(2);
            }

            return { lerp_vector(c, a, wca), contributors };
        }
        else // Inside the triangle
        {
            return { q, contributors };
        }
    }
    default:
    {
        throw std::exception("Simplex constains vertices more than 3");
    }
    };
}
