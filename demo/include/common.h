#pragma once

#ifdef __EMSCRIPTEN__
#include <GL/gl.h>
#include <GLES3/gl3.h>
#include <emscripten/emscripten.h>
#else
#include <glad/glad.h>
#endif

#include <GLFW/glfw3.h>

// clang-format off
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
