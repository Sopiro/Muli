#pragma once

#include "common.h"
#include "entity.h"
#include "rendering/rigidbody_renderer.h"
#include "rendering/dynamic_renderer.h"
#include "physics/rigidbody.h"
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

    private:
        Application& app;

        Settings settings{};
        std::unique_ptr<World> world;

        std::vector<RigidBody*> bodies{};
        Camera camera{};
        RigidBodyRenderer rRenderer{};
        DynamicRenderer dRenderer{};

        float time{ 0.0f };
        glm::vec2 mpos{ 0.0f };

        bool pause = false;
        bool step = false;
        bool drawOutlineOnly = false;
        bool showBVH = false;
        bool showCP = false;

        void AddBody(std::vector<RigidBody*> bodies);
        void AddBody(RigidBody* body);
        void RemoveBody(std::vector<RigidBody*> bodies);
        void RemoveBody(RigidBody* body);
        
        void UpdateProjectionMatrix();
    };
}