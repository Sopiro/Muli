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

    static void Init();
    static void Update();
    static bool IsKeyDown(int key);
    static bool IsKeyPressed(int key);
    static bool IsKeyReleased(int key);
    static bool IsMouseDown(int button);
    static bool IsMousePressed(int button);
    static bool IsMouseReleased(int button);
    static Vec2 GetMousePosition();
    static Vec2 GetMouseAcceleration();
    static Vec2 GetMouseScroll();

private:
    static std::array<bool, GLFW_KEY_LAST + 1> lastKeys;
    static std::array<bool, GLFW_KEY_LAST + 1> currKeys;

    static std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> lastBtns;
    static std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> currBtns;

    static Vec2 currMousePos;
    static Vec2 lastMousePos;
    static Vec2 mouseAcceleration;

    static Vec2 mouseScroll;
};

inline void Input::Init()
{
    lastKeys.fill(false);
    currKeys.fill(false);

    lastBtns.fill(false);
    currBtns.fill(false);
}

inline void Input::Update()
{
    lastKeys = currKeys;
    lastBtns = currBtns;

    mouseAcceleration = currMousePos - lastMousePos;
    lastMousePos = currMousePos;

    mouseScroll.SetZero();
}

inline bool Input::IsKeyDown(int key)
{
    return currKeys[key];
}

inline bool Input::IsKeyPressed(int key)
{
    return currKeys[key] && !lastKeys[key];
}

inline bool Input::IsKeyReleased(int key)
{
    return !currKeys[key] && lastKeys[key];
}

inline bool Input::IsMouseDown(int button)
{
    return currBtns[button];
}

inline bool Input::IsMousePressed(int button)
{
    return currBtns[button] && !lastBtns[button];
}

inline bool Input::IsMouseReleased(int button)
{
    return !currBtns[button] && lastBtns[button];
}

inline Vec2 Input::GetMousePosition()
{
    return currMousePos;
}

inline Vec2 Input::GetMouseAcceleration()
{
    return mouseAcceleration;
}

inline Vec2 Input::GetMouseScroll()
{
    return mouseScroll;
}

} // namespace spe
