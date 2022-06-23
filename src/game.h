#pragma once

#include "common.h"
#include "entity.h"
#include "rendering/shader.h"
#include "rendering/myshader.h"
#include "rendering/mesh.h"
#include "physics/polygon.h"
#include "physics/circle.h"
#include "physics/box.h"
#include "physics/aabbtree.h"

namespace spe
{
    class Engine;

    class Game
    {
    public:
        Game(Engine& engine);
        ~Game();

        void Update(float dt);
        void Render();

        void UpdateProjectionMatrix();

    private:
        glm::vec2 viewportSize{ 0.0f, 0.0f };
        float zoom = 100.0f;

        Engine& engine;
        std::unique_ptr<MyShader> s;
        std::unique_ptr<Mesh> m;
        float time{ 0.0f };
    };
}