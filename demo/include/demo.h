#pragma once

#include "common.h"

namespace spe
{
static void demo1(World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, BodyType::Static);

    RigidBody* box = world.CreateBox(0.4f);
    box->position = { 0.0f, 5.0f };
    box->SetAngularVelocity(glm::linearRand(-12.0f, 12.0f));
}

static void demo2(World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, BodyType::Static);
    ground->SetRestitution(0.45f);

    float start = 0.5f;
    float size = 0.3f;
    float gap = 0.25f;

    // float error = 0.015f;
    float error = 0.0f;

    for (size_t i = 0; i < 20; i++)
    {
        RigidBody* b = world.CreateBox(size);
        b->position = glm::vec2{ glm::linearRand(-error, error), start + i * (size + gap) };
    }
}

static void demo3(World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, BodyType::Static);
    ground->SetRestitution(0.45f);

    int32_t rows = 15;
    float boxSize = 0.35f;
    float xGap = 0.05f;
    float yGap = 0.15f;
    float xStart = -(rows - 1.0f) * (boxSize + xGap) / 2.0f;
    float yStart = 0.55f;

    for (size_t y = 0; y < rows; y++)
    {
        for (size_t x = 0; x < rows - y; x++)
        {
            RigidBody* b = world.CreateBox(boxSize);
            b->position = glm::vec2{ xStart + y * (boxSize + xGap) / 2.0f + x * (boxSize + xGap), yStart + y * (boxSize + yGap) };
        }
    }
}

static void demo4(World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, BodyType::Static);
    ground->SetRestitution(0.45f);

    Box* b = world.CreateBox(0.3f);
    b->position = glm::vec2(-3.0f, 5.0f);

    world.CreateRevoluteJoint(ground, b, glm::vec2(0.0f, 5.0f), -1.0f);
}

static void demo5(World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = false;

    Box* g = world.CreateBox(0.3f, 6, BodyType::Static);
    g->position.y = 3.6f;

    Box* b = world.CreateBox(0.3f);
    b->position.x = 3.0f;
    b->position.y = 3.6f + 2.0f;
    world.CreateDistanceJoint(g, b, { 0.0f, b->position.y }, b->position, 2.0f, 1.0f, 0.05f, b->GetMass());

    b = world.CreateBox(0.3f);
    b->position.x = 3.0f;
    b->position.y = 3.6f;
    world.CreateDistanceJoint(g, b, { 0.0f, b->position.y }, b->position, 2.0f, 1.0f, 0.2f, b->GetMass());

    b = world.CreateBox(0.3f);
    b->position.x = 3.0f;
    b->position.y = 3.6f - 2.0f;
    world.CreateDistanceJoint(g, b, { 0.0f, b->position.y }, b->position, 2.0f, 1.0f, 0.7f, b->GetMass());

    b = world.CreateBox(0.3f);
    b->position.x = -3.0f;
    b->position.y = 3.6f + 2.0f;
    world.CreateDistanceJoint(g, b, { 0.0f, b->position.y }, b->position, 2.0f, 0.5f, 0.2f, b->GetMass());

    // Reduce the amplitude by half every second
    float halfLife = 1.0f;
    float frequency = -glm::log(0.5f) / (halfLife * glm::pi<float>() * 2.0f);

    b = world.CreateBox(0.3f);
    b->position.x = -3.0f;
    b->position.y = 3.6f;
    world.CreateDistanceJoint(g, b, { 0.0f, b->position.y }, b->position, 2.0f, frequency, 1.0f, b->GetMass());

    b = world.CreateBox(0.3f);
    b->position.x = -3.0f;
    b->position.y = 3.6f - 2.0f;
    world.CreateDistanceJoint(g, b, { 0.0f, b->position.y }, b->position, 2.0f, 2.0f, 0.01f, b->GetMass());
}

static void demo6(World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, BodyType::Static);
    ground->SetRestitution(0.45f);

    int rows = 12;
    float size = 0.3f;
    float xGap = 0.2f;
    float yGap = 0.15f;
    float xStart = -(rows - 1) * (size + xGap) / 2.0f;
    float yStart = 1.0f;

    for (int y = 0; y < rows; y++)
    {
        for (int x = 0; x < rows - y; x++)
        {
            Polygon* b = world.CreateRandomConvexPolygon(size, 4, 10.0f);
            b->position = glm::vec2(xStart + y * (size + xGap) / 2 + x * (size + xGap), yStart + y * (size + yGap));
            b->SetLinearVelocity(b->position * glm::linearRand(0.5f, 0.7f));
            b->SetFriction(glm::linearRand(0.2f, 1.0f));
        }
    }

    Box* pillar = world.CreateBox(0.25f, 4.0f, BodyType::Static);
    pillar->position = { xStart - 0.2f, 3.0f };

    pillar = world.CreateBox(0.25f, 4.0f, BodyType::Static);
    pillar->position = { -(xStart - 0.2f), 3.0f };
}

static void demo7(World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, BodyType::Static);
    ground->SetRestitution(0.45f);

    Box* seesaw = world.CreateBox(6.0f, 0.1f);
    seesaw->position = { 0.0f, 0.45f };
    seesaw->SetMass(10.0f);

    world.CreateRevoluteJoint(ground, seesaw, seesaw->position, -1);

    RigidBody* b = world.CreateCircle(0.2f);
    b->position = {-2.5f, 1.0f};

    b = world.CreateBox(0.2f);
    b->position = {-2.8f, 1.0f};
    b->SetMass(1.0f);

    b = world.CreateBox(0.5f);
    b->position = {2.5f, 5.0f};
    b->SetMass(30.0f);
}

std::vector<std::pair<std::string, std::function<void(World&, Settings&)>>> get_demos()
{
    decltype(get_demos()) demos;

    demos.reserve(3);
    demos.push_back({ "Single Box", demo1 });
    demos.push_back({ "Box stacking", demo2 });
    demos.push_back({ "Pyramid", demo3 });
    demos.push_back({ "Single pendulum", demo4 });
    demos.push_back({ "Springs", demo5 });
    demos.push_back({ "Random convex shapes", demo6 });
    demos.push_back({ "Seesaw", demo7 });

    return demos;
}
}