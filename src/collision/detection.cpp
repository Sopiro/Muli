#include "spe/detection.h"
#include "spe/rigidbody.h"
#include "spe/polygon.h"
#include "spe/box.h"
#include "spe/circle.h"
#include "spe/simplex.h"
#include "spe/polytope.h"
#include "spe/edge.h"

namespace spe
{
struct SupportResult
{
    glm::vec2 vertex;
    int32_t index;
};

// Returns the fardest vertex in the 'dir' direction
static SupportResult support(RigidBody* b, glm::vec2 dir)
{
    dir = glm::normalize(dir);

    auto& type = typeid(*b);
    if (type == typeid(Polygon) || type == typeid(Box))
    {
        Polygon* p = static_cast<Polygon*>(b);
        const std::vector<glm::vec2>& vertices = p->GetVertices();

        int32_t idx = 0;
        float maxValue = glm::dot(dir, vertices[idx]);

        for (int32_t i = 1; i < vertices.size(); i++)
        {
            float value = glm::dot(dir, vertices[i]);
            if (value > maxValue)
            {
                idx = i;
                maxValue = value;
            }
        }

        return { vertices[idx], idx };
    }
    else if (type == typeid(Circle))
    {
        Circle* c = static_cast<Circle*>(b);

        return { dir * c->GetRadius(), -1 };
    }
    else
    {
        throw std::exception("Not a supported shape");
    }
}

/*
* Returns support point in 'Minkowski Difference' set
* Minkowski Sum: A ⊕ B = {Pa + Pb| Pa ∈ A, Pb ∈ B}
* Minkowski Difference : A ⊖ B = {Pa - Pb| Pa ∈ A, Pb ∈ B}
* CSO stands for Configuration Space Object
*/
static glm::vec2 cso_support(RigidBody* b1, RigidBody* b2, glm::vec2 dir)
{
    const glm::vec2 localDirP1 = glm::mul(b1->GlobalToLocal(), dir, 0);
    const glm::vec2 localDirP2 = glm::mul(b2->GlobalToLocal(), -dir, 0);

    const glm::vec2 supportP1 = b1->LocalToGlobal() * support(b1, localDirP1).vertex;
    const glm::vec2 supportP2 = b2->LocalToGlobal() * support(b2, localDirP2).vertex;

    return supportP1 - supportP2;
}

struct GJKResult
{
    bool collide;
    Simplex simplex;
};

static GJKResult gjk(RigidBody* b1, RigidBody* b2)
{
    constexpr glm::vec2 origin{ 0.0f };
    glm::vec2 dir{ 1.0f, 0.0f }; // Random initial direction

    bool collide = false;
    Simplex simplex{};

    glm::vec2 supportPoint = cso_support(b1, b2, dir);
    simplex.AddVertex(supportPoint);

    for (size_t k = 0; k < GJK_MAX_ITERATION; k++)
    {
        ClosestResult closest = simplex.GetClosest(origin);

        if (glm::distance2(closest.result, origin) < GJK_TOLERANCE)
        {
            collide = true;
            break;
        }

        if (simplex.Count() != 1)
        {
            // Rebuild the simplex with vertices that are used(involved) to calculate closest distance
            simplex.Shrink(closest.contributors);
        }

        dir = origin - closest.result;
        supportPoint = cso_support(b1, b2, dir);

        // If the new support point is not further along the search direction than the closest point,
        // two objects are not colliding so you can early return here.
        if (glm::length(dir) > glm::dot(glm::normalize(dir), supportPoint - closest.result))
        {
            collide = false;
            break;
        }

        if (simplex.ContainsVertex(supportPoint))
        {
            collide = false;
            break;
        }
        else
        {
            simplex.AddVertex(supportPoint);
        }
    }

    return { collide, simplex };
}

struct EPAResult
{
    float penetrationDepth;
    glm::vec2 contactNormal;
};

static EPAResult epa(RigidBody* b1, RigidBody* b2, Simplex& gjkResult)
{
    Polytope polytope{ gjkResult };

    ClosestEdgeInfo closestEdge{ 0, FLT_MAX, glm::vec2{0.0f} };

    for (size_t i = 0; i < EPA_MAX_ITERATION; i++)
    {
        closestEdge = polytope.GetClosestEdge();
        const glm::vec2 supportPoint = cso_support(b1, b2, closestEdge.normal);
        const float newDistance = glm::dot(closestEdge.normal, supportPoint);

        if (glm::abs(closestEdge.distance - newDistance) > EPA_TOLERANCE)
        {
            // Insert the support vertex so that it expands our polytope
            polytope.vertices.insert(polytope.vertices.begin() + closestEdge.index + 1, supportPoint);
        }
        else
        {
            // If you didn't expand edge, it means you reached the closest outer edge
            break;
        }
    }

    return { closestEdge.distance, closestEdge.normal };
}

static Edge find_farthest_edge(RigidBody* b, const glm::vec2& dir)
{
    const glm::vec2 localDir = glm::mul(b->GlobalToLocal(), dir, 0);
    const SupportResult farthest = support(b, localDir);

    glm::vec2 curr = farthest.vertex;
    int32_t idx = farthest.index;

    const glm::mat3 localToGlobal = b->LocalToGlobal();

    auto& typeID = typeid(*b);
    if (typeID == typeid(Circle))
    {
        curr = localToGlobal * curr;
        const glm::vec2 tangent = glm::cross(1.0f, dir) * TANGENT_MIN_LENGTH;

        return Edge{ curr, curr + tangent };
    }
    else if (typeID == typeid(Polygon) || typeID == typeid(Box))
    {
        Polygon* p = static_cast<Polygon*>(b);

        const std::vector<glm::vec2>& vertices = p->GetVertices();
        int32_t vertexCount = static_cast<int32_t>(p->VertexCount());

        const glm::vec2& prev = vertices[(idx - 1 + vertexCount) % vertexCount];
        const glm::vec2& next = vertices[(idx + 1) % vertexCount];

        const glm::vec2 e1 = glm::normalize(curr - prev);
        const glm::vec2 e2 = glm::normalize(curr - next);

        const bool w = glm::abs(glm::dot(e1, localDir)) <= glm::abs(glm::dot(e2, localDir));

        curr = localToGlobal * curr;

        return w ?
            Edge{ localToGlobal * prev, curr, (idx - 1 + vertexCount) % vertexCount, idx } :
            Edge{ curr, localToGlobal * next, idx, (idx + 1) % vertexCount };
    }
    else
    {
        throw std::exception("Not a supported shape");
    }
}

static void clip_edge(Edge* edge, const glm::vec2& p, const glm::vec2& dir, bool remove = false)
{
    const float d1 = glm::dot(edge->p1 - p, dir);
    const float d2 = glm::dot(edge->p2 - p, dir);

    if (d1 >= 0 && d2 >= 0) return;

    const float per = glm::abs(d1) + glm::abs(d2);

    if (d1 < 0)
    {
        if (remove)
        {
            edge->p1 = edge->p2;
            edge->id1 = edge->id2;
        }
        else
        {
            edge->p1 = edge->p1 + (edge->p2 - edge->p1) * (-d1 / per);
        }
    }
    else if (d2 < 0)
    {
        if (remove)
        {
            edge->p2 = edge->p1;
            edge->id2 = edge->id1;
        }
        else
        {
            edge->p2 = edge->p2 + (edge->p1 - edge->p2) * (-d2 / per);
        }
    }
}

static std::vector<ContactPoint> find_contact_points(const glm::vec2& n, RigidBody* a, RigidBody* b)
{
    Edge edgeA = find_farthest_edge(a, n);
    Edge edgeB = find_farthest_edge(b, -n);

    Edge* ref = &edgeA; // Reference edge
    Edge* inc = &edgeB; // Incidence edge
    bool flip = false;

    float aPerpendicularness = glm::abs(glm::dot(edgeA.dir, n));
    float bPerpendicularness = glm::abs(glm::dot(edgeB.dir, n));

    if (aPerpendicularness >= bPerpendicularness)
    {
        ref = &edgeB;
        inc = &edgeA;
        flip = true;
    }

    clip_edge(inc, ref->p1, ref->dir);
    clip_edge(inc, ref->p2, -(ref->dir));
    clip_edge(inc, ref->p1, flip ? n : -n, true);

    std::vector<ContactPoint> contactPoints{};

    // If two points are closer than threshold, merge them into one point
    if (inc->Length() <= CONTACT_MERGE_THRESHOLD)
    {
        contactPoints.emplace_back(inc->p1, inc->id1);
    }
    else
    {
        contactPoints.emplace_back(inc->p1, inc->id1);
        contactPoints.emplace_back(inc->p2, inc->id2);
    }

    return contactPoints;
}

std::optional<ContactManifold> spe::detect_collision(RigidBody* a, RigidBody* b)
{
    ContactManifold res{};

    // Circle vs. Circle collision
    if (typeid(*a) == typeid(Circle) && typeid(*b) == typeid(Circle))
    {
        float d = glm::distance2(a->position, b->position);
        const float r2 = static_cast<Circle*>(a)->GetRadius() + static_cast<Circle*>(b)->GetRadius();

        if (d > r2 * r2)
        {
            return {};
        }
        else
        {
            d = glm::sqrt(d);

            res.bodyA = a;
            res.bodyB = b;
            res.contactNormal = glm::normalize(b->position - a->position);
            res.contactPoints.push_back({ a->position + (res.contactNormal * static_cast<Circle*>(a)->GetRadius()), -1 });
            res.penetrationDepth = r2 - d;
            res.featureFlipped = false;

            if (glm::dot(res.contactNormal, glm::vec2{ 0.0f, -1.0f }) < 0.0f)
            {
                res.bodyA = b;
                res.bodyB = a;
                res.contactNormal *= -1;
                res.featureFlipped = true;
            }

            return res;
        }
    }

    GJKResult gjkResult = gjk(a, b);

    if (!gjkResult.collide)
    {
        return {};
    }
    else
    {
        // If the gjk termination simplex has vertices less than 3, expand to full simplex
        // Because EPA needs a full n-simplex to get started
        Simplex& simplex = gjkResult.simplex;
        switch (simplex.Count())
        {
        case 1:
        {
            const glm::vec2& v = simplex.vertices[0];
            glm::vec2 randomSupport = cso_support(a, b, glm::vec2{ 1, 0 });

            if (randomSupport == v)
                randomSupport = cso_support(a, b, glm::vec2{ -1, 0 });

            simplex.AddVertex(randomSupport);
        }
        case 2:
        {
            Edge e{ simplex.vertices[0], simplex.vertices[1] };
            glm::vec2 normalSupport = cso_support(a, b, e.Normal());

            if (simplex.ContainsVertex(normalSupport))
                simplex.AddVertex(cso_support(a, b, -e.Normal()));
            else
                simplex.AddVertex(normalSupport);
        }
        }

        assert(simplex.Count() == 3);

        EPAResult epaResult = epa(a, b, gjkResult.simplex);

        res.bodyA = a;
        res.bodyB = b;
        res.contactNormal = epaResult.contactNormal;
        res.penetrationDepth = epaResult.penetrationDepth;
        res.featureFlipped = false;

        // Apply axis weight to improve coherence
        if (glm::dot(epaResult.contactNormal, glm::vec2{ 0.0f, -1.0f }) < 0.0f)
        {
            res.bodyA = b;
            res.bodyB = a;
            res.contactNormal *= -1;
            res.featureFlipped = true;
        }

        // Remove floating point error
        res.contactNormal.x = glm::round((res.contactNormal.x / EPA_TOLERANCE)) * EPA_TOLERANCE;
        res.contactNormal.y = glm::round((res.contactNormal.y / EPA_TOLERANCE)) * EPA_TOLERANCE;

        res.contactPoints = find_contact_points(res.contactNormal, res.bodyA, res.bodyB);

        return res;
    }
}

bool test_point_inside(RigidBody* body, const glm::vec2& point)
{
    glm::vec2 localP = body->GlobalToLocal() * point;

    auto& type = typeid(*body);

    if (type == typeid(Circle))
    {
        return glm::length(localP) <= static_cast<Circle*>(body)->GetRadius();
    }
    else if (type == typeid(Box) || type == typeid(Polygon))
    {
        Polygon* p = static_cast<Polygon*>(body);
        const std::vector<glm::vec2>& vertices = p->GetVertices();

        float dir = glm::cross(vertices[0] - localP, vertices[1] - localP);

        for (size_t i = 1; i < vertices.size(); i++)
        {
            float nDir = glm::cross(vertices[i] - localP, vertices[(i + 1) % vertices.size()] - localP);

            if (dir * nDir < 0)
                return false;
        }

        return true;
    }
    else
    {
        throw std::exception("Not a supported shape");
    }
}
}
