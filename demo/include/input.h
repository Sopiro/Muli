#pragma once

#include "common.h"

namespace muli
{

class Window;

class Input final
{
public:
    Input() = delete;

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

    inline static std::array<bool, GLFW_KEY_LAST + 1> last_keys = { 0 };
    inline static std::array<bool, GLFW_KEY_LAST + 1> curr_keys = { 0 };

    inline static std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> last_btns = { 0 };
    inline static std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> curr_btns = { 0 };

    inline static Vec2 curr_mouse_pos{ 0 };
    inline static Vec2 last_mouse_pos{ 0 };
    inline static Vec2 mouse_acceleration{ 0 };

    inline static Vec2 mouse_scroll{ 0 };
};

inline void Input::Update()
{
    last_keys = curr_keys;
    last_btns = curr_btns;

    mouse_acceleration = curr_mouse_pos - last_mouse_pos;
    last_mouse_pos = curr_mouse_pos;

    mouse_scroll.SetZero();
}

inline bool Input::IsKeyDown(int32 key)
{
    return curr_keys[key];
}

inline bool Input::IsKeyPressed(int32 key)
{
    return curr_keys[key] && !last_keys[key];
}

inline bool Input::IsKeyReleased(int32 key)
{
    return !curr_keys[key] && last_keys[key];
}

inline bool Input::IsMouseDown(int32 button)
{
    return curr_btns[button];
}

inline bool Input::IsMousePressed(int32 button)
{
    return curr_btns[button] && !last_btns[button];
}

inline bool Input::IsMouseReleased(int32 button)
{
    return !curr_btns[button] && last_btns[button];
}

inline Vec2 Input::GetMousePosition()
{
    return curr_mouse_pos;
}

inline Vec2 Input::GetMouseAcceleration()
{
    return mouse_acceleration;
}

inline Vec2 Input::GetMouseScroll()
{
    return mouse_scroll;
}
} // namespace muli
