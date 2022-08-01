#pragma once

#include "common.h"

namespace spe
{
class Window;

class Input final
{
    friend class Window;

public:
    Input() = delete;

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

        glm::clear(mouseScroll);
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

    inline static bool IsMouseDown(int button)
    {
        return currBtns[button];
    }

    inline static bool IsMousePressed(int button)
    {
        return currBtns[button] && !lastBtns[button];
    }

    inline static bool IsMouseReleased(int button)
    {
        return !currBtns[button] && lastBtns[button];
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

    static std::array<bool, GLFW_KEY_LAST + 1> lastKeys;
    static std::array<bool, GLFW_KEY_LAST + 1> currKeys;

    static std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> lastBtns;
    static std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> currBtns;

    static glm::vec2 currMousePos;
    static glm::vec2 lastMousePos;
    static glm::vec2 mouseAcceleration;

    static glm::vec2 mouseScroll;
};
}
