#pragma once

#include "common.h"

namespace spe
{

static void single_box(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

    RigidBody* box = world.CreateBox(0.4f);
    box->SetPosition(0.0f, 5.0f);
    box->SetAngularVelocity(LinearRand(-12.0f, 12.0f));
}

static void box_stacking(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

    float start = 0.5f;
    float size = 0.3f;
    float gap = 0.25f;

    // float error = 0.015f;
    float error = 0.0f;

    for (uint32 i = 0; i < 20; i++)
    {
        RigidBody* b = world.CreateBox(size);
        b->SetPosition(LinearRand(-error, error), start + i * (size + gap));
    }
}

static void pyramid(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

    int32 rows = 15;
    float boxSize = 0.4f;
    float xGap = 0.0625f * boxSize / 0.5f;
    float yGap = 0.125f * boxSize / 0.5f;
    float xStart = -(rows - 1.0f) * (boxSize + xGap) / 2.0f;
    float yStart = 0.2f + boxSize / 2.0f + yGap;

    for (int y = 0; y < rows; y++)
    {
        for (int x = 0; x < rows - y; x++)
        {
            RigidBody* b = world.CreateBox(boxSize);
            b->SetPosition(xStart + y * (boxSize + xGap) / 2.0f + x * (boxSize + xGap), yStart + y * (boxSize + yGap));
        }
    }
}

static void single_pendulum(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

    Box* b = world.CreateBox(0.3f);
    b->SetPosition(-3.0f, 5.0f);

    world.CreateRevoluteJoint(ground, b, Vec2{ 0.0f, 5.0f }, -1.0f);
}

static void springs(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = false;

    Box* g = world.CreateBox(0.3f, 6, RigidBody::Type::Static);
    g->SetPosition(0.0f, 3.6f);

    Box* b = world.CreateBox(0.3f);
    b->SetPosition(3.0f, 3.6f + 2.0f);
    world.CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, 1.0f, 0.05f, b->GetMass());

    b = world.CreateBox(0.3f);
    b->SetPosition(3.0f, 3.6f);
    world.CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, 1.0f, 0.2f, b->GetMass());

    b = world.CreateBox(0.3f);
    b->SetPosition(3.0f, 3.6f - 2.0f);
    world.CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, 1.0f, 0.7f, b->GetMass());

    b = world.CreateBox(0.3f);
    b->SetPosition(-3.0f, 3.6f + 2.0f);
    world.CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, 0.5f, 0.2f, b->GetMass());

    // Reduce the amplitude by half every second
    float halfLife = 1.0f;
    float frequency = -Log(0.5f) / (halfLife * SPE_PI * 2.0f);

    b = world.CreateBox(0.3f);
    b->SetPosition(-3.0f, 3.6f);
    world.CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, frequency, 1.0f, b->GetMass());

    b = world.CreateBox(0.3f);
    b->SetPosition(-3.0f, 3.6f - 2.0f);
    world.CreateDistanceJoint(g, b, { 0.0f, b->GetPosition().y }, b->GetPosition(), 2.0f, 2.0f, 0.01f, b->GetMass());
}

static void random_convex_polygons(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

    int rows = 12;
    float size = 0.25f;
    float xGap = 0.2f;
    float yGap = 0.15f;
    float xStart = -(rows - 1) * (size + xGap) / 2.0f;
    float yStart = 1.0f;

    for (int y = 0; y < rows; y++)
    {
        for (int x = 0; x < rows - y; x++)
        {
            Polygon* b = world.CreateRandomConvexPolygon(size, 6);
            b->SetPosition(xStart + y * (size + xGap) / 2 + x * (size + xGap), yStart + y * (size + yGap));
            b->SetLinearVelocity(b->GetPosition() * LinearRand(0.5f, 0.7f));
            b->SetFriction(LinearRand(0.2f, 1.0f));
        }
    }

    Capsule* pillar = world.CreateCapsule(4.0f, 0.1f, false, RigidBody::Type::Static);
    pillar->SetPosition(xStart - 0.2f, 3.0f);

    pillar = world.CreateCapsule(4.0f, 0.1f, false, RigidBody::Type::Static);
    pillar->SetPosition(-(xStart - 0.2f), 3.0f);

    // Capsule* c = world.CreateCapsule(4.0f, 0.1f, true, RigidBody::Type::Dynamic);
    // c->SetPosition(0.0f, 2.0f);

    // world.CreateGrabJoint(c, c->GetPosition(), c->GetPosition(), -1.0f);
}

static void seesaw(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

    RigidBody* seesaw = world.CreateCapsule(6.0f, 0.05f, true);
    seesaw->SetPosition(0.0f, 0.45f);
    seesaw->SetMass(10.0f);

    world.CreateRevoluteJoint(ground, seesaw, seesaw->GetPosition(), -1);

    RigidBody* b = world.CreateCircle(0.2f);
    b->SetPosition(-2.5f, 1.0f);

    b = world.CreateBox(0.2f);
    b->SetPosition(-2.8f, 1.0f);
    b->SetMass(1.0f);

    b = world.CreateBox(0.5f);
    b->SetPosition(2.5f, 5.0f);
    b->SetMass(30.0f);

    b = world.CreateCapsule(1.0f, 0.5f, false, RigidBody::Type::Dynamic);
    b->SetPosition(-2.5, 250.0f);
}

static void frictions(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = true;
    float groundFriction = 0.5f;

    RigidBody* ground = world.CreateBox(100.0f, 0.4f, RigidBody::Type::Static);
    ground->SetFriction(groundFriction);

    RigidBody* b = world.CreateCapsule(5.9f, 0.05f, true, RigidBody::Type::Static);
    b->SetPosition(-0.6f, 5.0f);
    b->SetRotation(-0.15f);
    b->SetFriction(groundFriction);

    b = world.CreateCapsule(5.9f, 0.05f, true, RigidBody::Type::Static);
    b->SetPosition(0.0f, 3.0f);
    b->SetRotation(0.15f);
    b->SetFriction(groundFriction);

    b = world.CreateCapsule(5.9f, 0.05f, true, RigidBody::Type::Static);
    b->SetPosition(-0.6f, 1.0f);
    b->SetRotation(-0.15f);
    b->SetFriction(groundFriction);

    b = world.CreateBox(0.1f, 1.1f, RigidBody::Type::Static);
    b->SetPosition(3.1f, 4.3f);
    b = world.CreateBox(0.1f, 1.1f, RigidBody::Type::Static);
    b->SetPosition(-3.7f, 2.3f);

    float xStart = -4.5f;
    float yStart = 7.0f;
    float gap = 0.30f;
    float size = 0.30f;

    std::array<float, 5> frictions = { 0.4f, 0.3f, 0.2f, 0.1f, 0.0f };

    for (uint32 i = 0; i < frictions.size(); i++)
    {
        b = world.CreateBox(size, size);
        b->SetPosition(xStart + (size + gap) * i, yStart);
        b->SetFriction(frictions[i]);
        b->SetLinearVelocity({ 2.3f, 0.0f });
    }
}

static void restitutions(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

    int count = 11;
    float gap = 0.5f;
    float size = 0.3f;

    float xStart = -(count - 1) / 2 * gap;
    float yStart = 6.0f;

    bool r = LinearRand(0.0f, 1.0f) > 0.5f;

    RigidBody* b;
    for (int i = 0; i < count; i++)
    {
        if (r)
            b = world.CreateBox(size);
        else
            b = world.CreateCircle(size / 2.0f);
        b->SetPosition(xStart + gap * i, yStart);
        float attenuation = (count - i) / (float)count;
        b->SetRestitution(1.0f - attenuation * attenuation);
    }
}

static void multi_pendulum(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

    float xStart = 0.0f;
    float yStart = 5.0f;
    float sizeW = 0.3f;
    float sizeH = 0.15f;
    float gap = 0.1f;

    RigidBody* b1 = world.CreateBox(sizeW, sizeH);
    b1->SetMass(1.0);
    b1->SetPosition(xStart - (gap + sizeW), yStart);

    Joint* j = world.CreateRevoluteJoint(ground, b1, { xStart, yStart }, -1.0f);

    bool t = LinearRand(0.0f, 1.0f) > 0.5;

    int count = 12;
    for (int i = 1; i < count; i++)
    {
        RigidBody* b2 = world.CreateBox(sizeW, sizeH);
        b2->SetMass(1.0f);
        b2->SetPosition(xStart - (gap + sizeW) * (i + 1), yStart);

        if (t)
        {
            j = world.CreateRevoluteJoint(b1, b2, { xStart - (sizeW + gap) / 2 - (gap + sizeW) * i, yStart }, 15.0f, 0.5f);
        }
        else
        {
            j = world.CreateDistanceJoint(b1, b2, b1->GetPosition() - Vec2{ sizeW / 2, 0 },
                                          b2->GetPosition() + Vec2{ sizeW / 2, 0 });
        }

        b1 = b2;
    }
}

static void suspension_bridge(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

    float groundStart = 0.2f;

    float xStart = -5.0f;
    float yStart = 4.0f;
    float gap = 0.1f;

    float pillarWidth = 0.3f;
    float sizeX = 0.5f;
    float sizeY = sizeX * 0.25f;

    Box* pillar = world.CreateBox(pillarWidth, yStart, RigidBody::Type::Static);
    pillar->SetPosition(xStart, yStart / 2 + 0.2f);

    Box* b1 = world.CreateBox(sizeX, sizeY);
    b1->SetMass(10.0f);
    b1->SetPosition(xStart + sizeX / 2 + pillarWidth / 2 + gap, yStart + groundStart);

    Joint* j;

    bool revoluteBridge = LinearRand(0.0f, 1.0f) > 0.5;
    float frequency = 30.0f;

    if (revoluteBridge)
    {
        j = world.CreateRevoluteJoint(pillar, b1, pillar->GetPosition() + Vec2{ pillarWidth, yStart } / 2.0f, frequency, 1.0f);
    }
    else
    {
        j = world.CreateDistanceJoint(pillar, b1, pillar->GetPosition() + Vec2{ pillarWidth / 2.0f, yStart / 2.0f },
                                      b1->GetPosition() + Vec2{ -sizeX / 2.0f, 0.03f }, -1.0f, frequency, 1.0f);
        j = world.CreateDistanceJoint(pillar, b1, pillar->GetPosition() + Vec2{ pillarWidth / 2.0f, yStart / 2.0f },
                                      b1->GetPosition() + Vec2{ -sizeX / 2.0f, -0.03f }, -1.0f, frequency, 1.0f);
    }

    for (int i = 1; i + 1 < xStart * -2 / (sizeX + gap); i++)
    {
        Box* b2 = world.CreateBox(sizeX, sizeY);
        b2->SetMass(10.0f);
        b2->SetPosition(xStart + sizeX / 2.0f + pillarWidth / 2.0f + gap + (gap + sizeX) * i, yStart + groundStart);

        if (revoluteBridge)
        {
            j = world.CreateRevoluteJoint(b1, b2, (b1->GetPosition() + b2->GetPosition()) / 2.0f, frequency, 1.0f);
        }
        else
        {
            j = world.CreateDistanceJoint(b1, b2, b1->GetPosition() + Vec2{ sizeX / 2.0f, 0.03f },
                                          b2->GetPosition() + Vec2{ -sizeX / 2.0f, 0.03f }, -1.0f, frequency, 1.0f);
            j = world.CreateDistanceJoint(b1, b2, b1->GetPosition() + Vec2{ sizeX / 2.0f, -0.03f },
                                          b2->GetPosition() + Vec2{ -sizeX / 2.0f, -0.03f }, -1.0f, frequency, 1.0f);
        }

        b1 = b2;
    }

    pillar = world.CreateBox(pillarWidth, yStart, RigidBody::Type::Static);
    pillar->SetPosition(-xStart, yStart / 2.0f + 0.2f);

    if (revoluteBridge)
    {
        j = world.CreateRevoluteJoint(pillar, b1, pillar->GetPosition() + Vec2{ -pillarWidth, yStart } / 2.0f, frequency, 1.0f);
    }
    else
    {
        j = world.CreateDistanceJoint(pillar, b1, pillar->GetPosition() + Vec2{ -pillarWidth / 2.0f, yStart / 2.0f },
                                      b1->GetPosition() + Vec2{ sizeX / 2.0f, 0.03f }, -1, frequency, 1.0f);
        j = world.CreateDistanceJoint(pillar, b1, pillar->GetPosition() + Vec2{ -pillarWidth / 2.0f, yStart / 2.0f },
                                      b1->GetPosition() + Vec2{ sizeX / 2.0f, -0.03f }, -1, frequency, 1.0f);
    }

    Camera& camera = game.GetCamera();
    camera.position = Vec2{ 0, 3.6f + 1.8f };
    camera.scale = Vec2{ 1.5f, 1.5f };
}

static void circle_stacking(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = true;
    RigidBody* ground = world.CreateBox(100.0f, 0.4f, RigidBody::Type::Static);

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
            c->SetPosition(xStart + (gap + size * 2) * i, yStart + (gap + size * 2) * j);
        }
    }
}

static void circles_1000(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = true;

    float size = 15.0f;
    float halfSize = size / 2.0f;
    float wallWidth = 0.4f;
    float wallRadius = wallWidth / 2.0f;

    world.CreateCapsule(Vec2{ -halfSize, -halfSize }, Vec2{ halfSize, -halfSize }, wallRadius, false, RigidBody::Type::Static);
    world.CreateCapsule(Vec2{ halfSize, -halfSize }, Vec2{ halfSize, halfSize }, wallRadius, false, RigidBody::Type::Static);
    world.CreateCapsule(Vec2{ halfSize, halfSize }, Vec2{ -halfSize, halfSize }, wallRadius, false, RigidBody::Type::Static);
    world.CreateCapsule(Vec2{ -halfSize, halfSize }, Vec2{ -halfSize, -halfSize }, wallRadius, false, RigidBody::Type::Static);

    float r = 0.22f;

    for (int i = 0; i < 1000; i++)
    {
        RigidBody* b = world.CreateCircle(r);
        b->SetPosition(LinearRand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f,
                       LinearRand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f);
        b->SetRotation(LinearRand(0.0f, SPE_PI * 2.0f));
    }

    Camera& c = game.GetCamera();
    c.position = { 0.0f, 0.0f };
    c.scale = { 3.f, 3.f };
}

static void boxes_1000(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = true;

    float size = 15.0f;
    float halfSize = size / 2.0f;
    float wallWidth = 0.4f;
    float wallRadius = wallWidth / 2.0f;

    world.CreateCapsule(Vec2{ -halfSize, -halfSize }, Vec2{ halfSize, -halfSize }, wallRadius, false, RigidBody::Type::Static);
    world.CreateCapsule(Vec2{ halfSize, -halfSize }, Vec2{ halfSize, halfSize }, wallRadius, false, RigidBody::Type::Static);
    world.CreateCapsule(Vec2{ halfSize, halfSize }, Vec2{ -halfSize, halfSize }, wallRadius, false, RigidBody::Type::Static);
    world.CreateCapsule(Vec2{ -halfSize, halfSize }, Vec2{ -halfSize, -halfSize }, wallRadius, false, RigidBody::Type::Static);

    float r = 0.38f;

    for (int i = 0; i < 1000; i++)
    {
        RigidBody* b = world.CreateBox(r);
        b->SetPosition(LinearRand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f,
                       LinearRand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f);
    }

    Camera& c = game.GetCamera();
    c.position = { 0.0f, 0.0f };
    c.scale = { 3.f, 3.f };
}

static void capsules_1000(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = true;

    float size = 15.0f;
    float halfSize = size / 2.0f;
    float wallWidth = 0.4f;
    float wallRadius = wallWidth / 2.0f;

    world.CreateCapsule(Vec2{ -halfSize, -halfSize }, Vec2{ halfSize, -halfSize }, wallRadius, false, RigidBody::Type::Static);
    world.CreateCapsule(Vec2{ halfSize, -halfSize }, Vec2{ halfSize, halfSize }, wallRadius, false, RigidBody::Type::Static);
    world.CreateCapsule(Vec2{ halfSize, halfSize }, Vec2{ -halfSize, halfSize }, wallRadius, false, RigidBody::Type::Static);
    world.CreateCapsule(Vec2{ -halfSize, halfSize }, Vec2{ -halfSize, -halfSize }, wallRadius, false, RigidBody::Type::Static);

    float r = 0.3f;

    for (int i = 0; i < 1000; i++)
    {
        Capsule* c = world.CreateCapsule(r, r / 2.0f);
        c->SetPosition(LinearRand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f,
                       LinearRand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f);
        c->SetRotation(LinearRand(0.0f, SPE_PI * 2.0f));
    }

    Camera& c = game.GetCamera();
    c.position = { 0.0f, 0.0f };
    c.scale = { 3.f, 3.f };
}

static void convex_polygons_1000(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = true;

    float size = 15.0f;
    float halfSize = size / 2.0f;
    float wallWidth = 0.4f;
    float wallRadius = wallWidth / 2.0f;

    world.CreateCapsule(Vec2{ -halfSize, -halfSize }, Vec2{ halfSize, -halfSize }, wallRadius, false, RigidBody::Type::Static);
    world.CreateCapsule(Vec2{ halfSize, -halfSize }, Vec2{ halfSize, halfSize }, wallRadius, false, RigidBody::Type::Static);
    world.CreateCapsule(Vec2{ halfSize, halfSize }, Vec2{ -halfSize, halfSize }, wallRadius, false, RigidBody::Type::Static);
    world.CreateCapsule(Vec2{ -halfSize, halfSize }, Vec2{ -halfSize, -halfSize }, wallRadius, false, RigidBody::Type::Static);

    float r = 0.27f;
    for (int i = 0; i < 1000; i++)
    {
        RigidBody* b = world.CreateRandomConvexPolygon(r, 7);
        b->SetPosition(LinearRand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f,
                       LinearRand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f);
    }

    Camera& c = game.GetCamera();
    c.position = { 0.0f, 0.0f };
    c.scale = { 3.f, 3.f };
}

static void mix_1000(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = true;

    float size = 15.0f;
    float halfSize = size / 2.0f;
    float wallWidth = 0.4f;
    float wallRadius = wallWidth / 2.0f;

    world.CreateCapsule(Vec2{ -halfSize, -halfSize }, Vec2{ halfSize, -halfSize }, wallRadius, false, RigidBody::Type::Static);
    world.CreateCapsule(Vec2{ halfSize, -halfSize }, Vec2{ halfSize, halfSize }, wallRadius, false, RigidBody::Type::Static);
    world.CreateCapsule(Vec2{ halfSize, halfSize }, Vec2{ -halfSize, halfSize }, wallRadius, false, RigidBody::Type::Static);
    world.CreateCapsule(Vec2{ -halfSize, halfSize }, Vec2{ -halfSize, -halfSize }, wallRadius, false, RigidBody::Type::Static);

    float r = 0.24f;

    RigidBody* b;
    for (int i = 0; i < 1000; i++)
    {
        float random = LinearRand(0.0f, 3.0f);
        if (random < 1.0f)
        {
            b = world.CreateRandomConvexPolygon(r, 7);
        }
        else if (random < 2.0f)
        {
            b = world.CreateCircle(r);
        }
        else
        {
            b = world.CreateCapsule(r * 1.2f, r * 1.2f / 2.0f);
        }

        b->SetPosition(LinearRand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f,
                       LinearRand(0.0f, size - wallWidth) - (size - wallWidth) / 2.0f);
    }

    Camera& c = game.GetCamera();
    c.position = { 0.0f, 0.0f };
    c.scale = { 3.f, 3.f };
}

static void dense_collision(Game& game, World& world, WorldSettings& settings)
{
    settings.APPLY_GRAVITY = false;

    float size = 15.0f;
    float r = 0.25f;
    float spread = 10.0f;

    Polygon* b = world.CreateRandomConvexPolygon(spread / 2.0f, 7);
    b->SetPosition(-25.0, 0.0f);
    b->SetLinearVelocity(12.0f, 0.0f);
    b->SetAngularVelocity(1.0f);

    for (int i = 0; i < 500; i++)
    {
        Circle* c = world.CreateCircle(r);
        c->SetPosition(LinearRand(0.0f, spread * 1.414f), LinearRand(0.0f, spread * 0.9f) - spread / 2.0f);
    }

    Camera& c = game.GetCamera();
    c.position = { -5.0f, 0.0f };
    c.scale = { 6.0f, 6.0f };
}

std::vector<std::pair<std::string, std::function<void(Game&, World&, WorldSettings&)>>> GetDemos()
{
    decltype(GetDemos()) demos;
    demos.reserve(12);

    demos.push_back({ "Single Box", single_box });
    demos.push_back({ "Box stacking", box_stacking });
    demos.push_back({ "Pyramid", pyramid });
    demos.push_back({ "Single pendulum", single_pendulum });
    demos.push_back({ "Springs", springs });
    demos.push_back({ "Random convex polygons", random_convex_polygons });
    demos.push_back({ "Seesaw", seesaw });
    demos.push_back({ "Friction test", frictions });
    demos.push_back({ "Restitution test", restitutions });
    demos.push_back({ "Multi pendulum", multi_pendulum });
    demos.push_back({ "Suspension bridge", suspension_bridge });
    demos.push_back({ "Circle stacking", circle_stacking });
    demos.push_back({ "1000 circles", circles_1000 });
    demos.push_back({ "1000 boxes", boxes_1000 });
    demos.push_back({ "1000 capsules", capsules_1000 });
    demos.push_back({ "1000 convex polygons", convex_polygons_1000 });
    demos.push_back({ "1000 random shapes", mix_1000 });
    demos.push_back({ "Dense collision", dense_collision });

    return demos;
}

} // namespace spe