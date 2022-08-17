#pragma once

#include "common.h"

namespace spe
{
static void demo1(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, BodyType::Static);

    RigidBody* box = world.CreateBox(0.4f);
    box->position = { 0.0f, 5.0f };
    box->SetAngularVelocity(glm::linearRand(-12.0f, 12.0f));
}

static void demo2(Game& game, World& world, Settings& settings)
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

static void demo3(Game& game, World& world, Settings& settings)
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

static void demo4(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, BodyType::Static);
    ground->SetRestitution(0.45f);

    Box* b = world.CreateBox(0.3f);
    b->position = glm::vec2(-3.0f, 5.0f);

    world.CreateRevoluteJoint(ground, b, glm::vec2(0.0f, 5.0f), -1.0f);
}

static void demo5(Game& game, World& world, Settings& settings)
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

static void demo6(Game& game, World& world, Settings& settings)
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

static void demo7(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, BodyType::Static);
    ground->SetRestitution(0.45f);

    Box* seesaw = world.CreateBox(6.0f, 0.1f);
    seesaw->position = { 0.0f, 0.45f };
    seesaw->SetMass(10.0f);

    world.CreateRevoluteJoint(ground, seesaw, seesaw->position, -1);

    RigidBody* b = world.CreateCircle(0.2f);
    b->position = { -2.5f, 1.0f };

    b = world.CreateBox(0.2f);
    b->position = { -2.8f, 1.0f };
    b->SetMass(1.0f);

    b = world.CreateBox(0.5f);
    b->position = { 2.5f, 5.0f };
    b->SetMass(30.0f);
}

static void demo8(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, BodyType::Static);
    ground->SetRestitution(0.45f);

    Box* b = world.CreateBox(6.0f, 0.1f, BodyType::Static);
    b->position = { -0.6f, 5.0f };
    b->rotation = -0.15f;
    b->SetFriction(1.0f);

    b = world.CreateBox(6.0f, 0.1f, BodyType::Static);
    b->position = { 0.0f, 3.0f };
    b->rotation = 0.15f;
    b->SetFriction(1.0f);

    b = world.CreateBox(6.0f, 0.1f, BodyType::Static);
    b->position = { -0.6f, 1.0f };
    b->rotation = -0.15f;
    b->SetFriction(1.0f);


    b = world.CreateBox(0.1f, 1.1f, BodyType::Static);
    b->position = { 3.1f, 4.3f };
    b = world.CreateBox(0.1f, 1.1f, BodyType::Static);
    b->position = { -3.7f, 2.3f };

    float xStart = -4.5f;
    float yStart = 7.0f;
    float gap = 0.30f;
    float size = 0.30f;

    std::array<float, 5> frictions = { 0.51f, 0.31f, 0.21f, 0.11f, 0.0f };

    for (size_t i = 0; i < frictions.size(); i++)
    {
        b = world.CreateBox(size, size);
        b->position = { xStart + (size + gap) * i, yStart };
        b->SetFriction(frictions[i]);
        b->SetLinearVelocity({ 2.0f, 0.0f });
    }
}

static void demo9(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, BodyType::Static);
    ground->SetRestitution(0.7f);

    int count = 7;
    float gap = 1.0f;
    float size = 0.3f;

    float xStart = -(count - 1) / 2 * gap;
    float yStart = 6.5f;

    for (int i = 0; i < count; i++)
    {
        Box* b = world.CreateBox(size);
        b->position = { xStart + gap * i, yStart };
        float attenuation = (count - i) / (float)count;
        b->SetRestitution(1.0f - attenuation * attenuation);
    }
}

static void demo10(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, BodyType::Static);
    ground->SetRestitution(0.45f);

    float xStart = 0.0f;
    float yStart = 5.0f;
    float sizeW = 0.3f;
    float sizeH = 0.15f;
    float gap = 0.1f;

    RigidBody* b1 = world.CreateBox(sizeW, sizeH);
    b1->SetMass(1.0);
    b1->position = { xStart - (gap + sizeW), yStart };

    Joint* j = world.CreateRevoluteJoint(ground, b1, { xStart, yStart }, -1.0f);

    bool t = glm::linearRand<float>(0.0f, 1.0f) > 0.5;

    for (int i = 1; i < 12; i++)
    {
        RigidBody* b2 = world.CreateBox(sizeW, sizeH);
        b2->SetMass(1.0f);
        b2->position = { xStart - (gap + sizeW) * (i + 1), yStart };

        if (t)
            j = world.CreateRevoluteJoint(b1, b2, { xStart - (sizeW + gap) / 2 - (gap + sizeW) * i, yStart }, 15.0f, 0.5f);
        else
            j = world.CreateDistanceJoint(b1, b2, b1->position - glm::vec2{ sizeW / 2, 0 }, b2->position + glm::vec2{ sizeW / 2, 0 });

        b1 = b2;
    }
}

static void demo11(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, BodyType::Static);
    ground->SetRestitution(0.45f);

    float groundStart = 0.2f;

    float xStart = -5.0f;
    float yStart = 4.0f;
    float gap = 0.1f;

    float pillarWidth = 0.3f;
    float sizeX = 0.5f;
    float sizeY = sizeX * 0.25f;

    Box* pillar = world.CreateBox(pillarWidth, yStart, BodyType::Static);
    pillar->position = glm::vec2(xStart, yStart / 2 + 0.2);

    Box* b1 = world.CreateBox(sizeX, sizeY);
    b1->SetMass(10.0f);
    b1->position = glm::vec2(xStart + sizeX / 2 + pillarWidth / 2 + gap, yStart + groundStart);

    Joint* j;

    bool revoluteBridge = glm::linearRand<float>(0.0f, 1.0f) > 0.5;
    float frequency = 30.0f;

    if (revoluteBridge)
    {
        j = world.CreateRevoluteJoint(pillar, b1, pillar->position + glm::vec2(pillarWidth, yStart) / 2.0f, frequency, 1.0f);
    }
    else
    {
        j = world.CreateDistanceJoint(pillar, b1, pillar->position + glm::vec2(pillarWidth / 2.0f, yStart / 2.0f), b1->position + glm::vec2(-sizeX / 2, 0.03), -1.0f, frequency, 1.0f);
        j = world.CreateDistanceJoint(pillar, b1, pillar->position + glm::vec2(pillarWidth / 2.0f, yStart / 2.0f), b1->position + glm::vec2(-sizeX / 2, -0.03), -1.0f, frequency, 1.0f);
    }

    for (int i = 1; i + 1 < xStart * -2 / (sizeX + gap); i++)
    {
        Box* b2 = world.CreateBox(sizeX, sizeY);
        b2->SetMass(10.0f);
        b2->position = glm::vec2(xStart + sizeX / 2.0f + pillarWidth / 2.0f + gap + (gap + sizeX) * i, yStart + groundStart);

        if (revoluteBridge)
        {
            j = world.CreateRevoluteJoint(b1, b2, (b1->position + b2->position) / 2.0f, frequency, 1.0f);
        }
        else
        {
            j = world.CreateDistanceJoint(b1, b2, b1->position + glm::vec2(sizeX / 2.0f, 0.03f), b2->position + glm::vec2(-sizeX / 2.0f, 0.03f), -1.0f, frequency, 1.0f);
            j = world.CreateDistanceJoint(b1, b2, b1->position + glm::vec2(sizeX / 2.0f, -0.03f), b2->position + glm::vec2(-sizeX / 2.0f, -0.03f), -1.0f, frequency, 1.0f);
        }

        b1 = b2;
    }

    pillar = world.CreateBox(pillarWidth, yStart, BodyType::Static);
    pillar->position = glm::vec2(-xStart, yStart / 2.0f + 0.2f);

    if (revoluteBridge)
    {
        j = world.CreateRevoluteJoint(pillar, b1, pillar->position + glm::vec2(-pillarWidth, yStart) / 2.0f, frequency, 1.0f);
    }
    else
    {
        j = world.CreateDistanceJoint(pillar, b1, pillar->position + glm::vec2(-pillarWidth / 2.0f, yStart / 2.0f), b1->position + glm::vec2(sizeX / 2, 0.03), -1, frequency, 1.0f);
        j = world.CreateDistanceJoint(pillar, b1, pillar->position + glm::vec2(-pillarWidth / 2.0f, yStart / 2.0f), b1->position + glm::vec2(sizeX / 2, -0.03), -1, frequency, 1.0f);
    }

    Camera& camera = game.GetCamera();
    camera.position = glm::vec2{ 0, 3.6f + 1.8f };
    camera.scale = glm::vec2{ 1.5f, 1.5f };
}

static void demo12(Game& game, World& world, Settings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(12.8f * 10.0f, 0.4f, BodyType::Static);
    ground->SetRestitution(0.45f);

    float xStart = -3.0f;
    float yStart = 1.0f;
    float size = 0.3f;
    float gap = 0.3f;

    int rows = 10;

    for (int i = 0; i < rows; i++)
    {
        for (int j = i; j < rows; j++)
        {
            Circle* c = world.CreateCircle(size);
            c->SetMass((1 + i) + (1 + i) * j + 2.0f);
            c->position.x = xStart + (gap + size * 2) * i;
            c->position.y = yStart + (gap + size * 2) * j;
        }
    }
}

std::vector<std::pair<std::string, std::function<void(Game&, World&, Settings&)>>> get_demos()
{
    decltype(get_demos()) demos;
    demos.reserve(12);

    demos.push_back({ "Single Box", demo1 });
    demos.push_back({ "Box stacking", demo2 });
    demos.push_back({ "Pyramid", demo3 });
    demos.push_back({ "Single pendulum", demo4 });
    demos.push_back({ "Springs", demo5 });
    demos.push_back({ "Random convex shapes", demo6 });
    demos.push_back({ "Seesaw", demo7 });
    demos.push_back({ "Friction test", demo8 });
    demos.push_back({ "Restitution test", demo9 });
    demos.push_back({ "Multi pendulum", demo10 });
    demos.push_back({ "Suspension bridge", demo11 });
    demos.push_back({ "Circle stacking", demo12 });

    return demos;
}

}