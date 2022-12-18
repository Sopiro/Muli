#pragma once

#include "common.h"

namespace muli
{

class Window;

class Input final
{
public:
    Input() = delete;

    static void Init();
    static void Update();
    static bool IsKeyDown(int32 key);
    static bool IsKeyPressed(int32 key);
    static bool IsKeyReleased(int32 key);
    static bool IsMouseDown(int32 button);
    static bool IsMousePressed(int32 button);
    static bool IsMouseReleased(int32 button);
    static Vec2 GetMousePosition();
    static Vec2 GetMouseAcceleration();
    static Vec2 GetMouseScroll();

private:
    friend class Window;

    inline static std::array<bool, GLFW_KEY_LAST + 1> lastKeys;
    inline static std::array<bool, GLFW_KEY_LAST + 1> currKeys;

    inline static std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> lastBtns;
    inline static std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> currBtns;

    inline static Vec2 currMousePos{ 0.0f };
    inline static Vec2 lastMousePos{ 0.0f };
    inline static Vec2 mouseAcceleration{ 0.0f };

    inline static Vec2 mouseScroll{ 0.0f };
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

inline bool Input::IsKeyDown(int32 key)
{
    return currKeys[key];
}

inline bool Input::IsKeyPressed(int32 key)
{
    return currKeys[key] && !lastKeys[key];
}

inline bool Input::IsKeyReleased(int32 key)
{
    return !currKeys[key] && lastKeys[key];
}

inline bool Input::IsMouseDown(int32 button)
{
    return currBtns[button];
}

inline bool Input::IsMousePressed(int32 button)
{
    return currBtns[button] && !lastBtns[button];
}

inline bool Input::IsMouseReleased(int32 button)
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

} // namespace muli
