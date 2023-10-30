#pragma once

// clang-format off
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
// clang-format on

#include <iostream>
#include <muli/muli.h>

class NonCopyable
{
public:
    NonCopyable() = default;
    NonCopyable(const NonCopyable&) = delete;
    void operator=(const NonCopyable&) = delete;
};
