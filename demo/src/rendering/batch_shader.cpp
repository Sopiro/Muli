#include "batch_shader.h"

namespace muli
{

const char* vertexShader =
#ifndef __EMSCRIPTEN__
    R"(
    #version 330 core

    layout (location = 0) in vec2 pos;
    layout (location = 1) in vec4 color;

    out vec4 out_color;

    uniform mat4 view;
    uniform mat4 proj;

    void main()
    {
        mat4 mvp = proj * view;

        out_color = color;
        gl_Position = mvp * vec4(pos, 0.0f, 1.0f);
    }
)";
#else
    R"(
    #version 100

    attribute vec2 pos;
    attribute vec4 color;

    varying vec4 out_color;

    uniform mat4 view;
    uniform mat4 proj;

    void main()
    {
        mat4 mvp = proj * view;

        out_color = color;
        gl_PointSize = 5.0;
        gl_Position = mvp * vec4(pos, 0.0, 1.0);
    }
)";
#endif

const char* fragmentShader =
#ifndef __EMSCRIPTEN__
    R"(
    #version 330 core

    in vec4 out_color;

    out vec4 fragColor;

    void main()
    {
        fragColor = out_color;
    }
)";
#else
    R"(
    #version 100
    precision mediump float;

    varying vec4 out_color;

    void main()
    {
        gl_FragColor = out_color;
    }
)";
#endif

BatchShader::BatchShader()
    : Shader(vertexShader, fragmentShader)
{
    uniformMap.insert({ "color", glGetUniformLocation(shaderHandle, "color") });
    uniformMap.insert({ "model", glGetUniformLocation(shaderHandle, "model") });
    uniformMap.insert({ "view", glGetUniformLocation(shaderHandle, "view") });
    uniformMap.insert({ "proj", glGetUniformLocation(shaderHandle, "proj") });
}

} // namespace muli