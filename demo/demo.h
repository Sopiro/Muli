#pragma once

#include "common.h"
#include "game.h"

namespace spe
{
static void demo1(Game& game, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = new Box{ 12.8f * 10.0f, 0.4f, Static };
    game.AddBody(ground);

    RigidBody* box = new Box{ 0.4f };
    box->position = { 0.0f, 5.0f };
    box->SetAngularVelocity(glm::linearRand(-12.0f, 12.0f));
    game.AddBody(box);
}

static void demo2(Game& game, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = new Box{ 12.8f * 10.0f, 0.4f, Static };
    ground->SetRestitution(0.45f);
    game.AddBody(ground);

    float start = 0.5f;
    float size = 0.3f;
    float gap = 0.25f;

    // float error = 0.015f;
    float error = 0.0f;

    for (size_t i = 0; i < 20; i++)
    {
        RigidBody* b = new Box{ size };
        b->position = glm::vec2{ glm::linearRand(-error, error), start + i * (size + gap) };
        game.AddBody(b);
    }
}

static void demo3(Game& game, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = new Box{ 12.8f * 10.0f, 0.4f, Static };
    ground->SetRestitution(0.45f);
    game.AddBody(ground);

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
            RigidBody* b = new Box{ boxSize };
            b->position = glm::vec2{ xStart + y * (boxSize + xGap) / 2.0f + x * (boxSize + xGap), yStart + y * (boxSize + yGap) };
            game.AddBody(b);
        }
    }
}

std::vector<std::pair<std::string, std::function<void(Game&, Settings&)>>> get_demos()
{
    decltype(get_demos()) demos;

    demos.reserve(3);
    demos.push_back({ "Single Box", demo1 });
    demos.push_back({ "Box stacking", demo2 });
    demos.push_back({ "Pyramid", demo3 });

    return demos;
}
}