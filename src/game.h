#pragma once

#include "common.h"
#include "entity.h"
#include "rendering/rigidbody_renderer.h"
#include "physics/polygon.h"
#include "physics/circle.h"
#include "physics/box.h"
#include "physics/aabbtree.h"
#include "input.h"
#include "physics/world.h"
#include "camera.h"

namespace spe
{
    class Application;

    class Game final
    {
    public:
        Game(Application& app);
        ~Game() noexcept;

        Game(const Game&) noexcept = delete;
        Game& operator=(const Game&) noexcept = delete;

        Game(Game&&) noexcept = delete;
        Game& operator=(Game&&) noexcept = delete;

        void Update(float dt);
        void HandleInput();
        void Render();

        void UpdateProjectionMatrix();

    private:
        Application& app;

        bool drawOutlineOnly = false;

        std::set<RigidBody*> bodies{};
        Camera camera{};
        World world{};
        RigidBodyRenderer renderer{};

        float time{ 0.0f };
    };
}