#pragma once

#include "common.h"

namespace spe
{
    class Window;

    class Input
    {
        friend class Window;

    public:
        inline static void Init()
        {
            lastKeys.fill(false);
            currKeys.fill(false);

            lastBtns.fill(false);
            currBtns.fill(false);
        }

        inline static void Update()
        {
            lastKeys = currKeys;
            lastBtns = currBtns;

            mouseAcceleration = currMousePos - lastMousePos;
            lastMousePos = currMousePos;

            mouseScroll.x = 0;
            mouseScroll.y = 0;
        }

        inline static bool IsKeyDown(int key)
        {
            return currKeys[key];
        }

        inline static bool IsKeyPressed(int key)
        {
            return currKeys[key] && !lastKeys[key];
        }

        inline static bool IsKeyReleased(int key)
        {
            return !currKeys[key] && lastKeys[key];
        }

        inline static glm::vec2 GetMousePosition()
        {
            return currMousePos;
        }

        inline static glm::vec2 GetMouseAcceleration()
        {
            return mouseAcceleration;
        }

        inline static glm::vec2 GetMouseScroll()
        {
            return mouseScroll;
        }

    private:
        Input();

        static std::array<bool, GLFW_KEY_LAST + 1> lastKeys;
        static std::array<bool, GLFW_KEY_LAST + 1> currKeys;

        static std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> lastBtns;
        static std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> currBtns;

        static glm::vec2 currMousePos;
        static glm::vec2 lastMousePos;
        static glm::vec2 mouseAcceleration;

        static glm::vec2 mouseScroll;

        // static glm::vec2 currMousePos;
        // static glm::vec2 lastMousePos;
    };
}
