#include "dynamic_shader.h"

namespace muli
{

DynamicShader::DynamicShader()
    : Shader(
          // Vertex shader
          R"(
        #version 330 core

        layout (location = 0) in vec2 pos;
        layout (location = 1) in vec4 color;

        out vec4 f_color;

        uniform mat4 view;
        uniform mat4 proj;

        void main()
        {
           mat4 mvp = proj * view;

           f_color = color;
           gl_Position = mvp * vec4(pos, 0.0f, 1.0f);
        }
    )",
          // Fragment shader
          R"(
        #version 330 core
        
        in vec4 f_color;

        out vec4 fragColor;

        void main()
        {
            fragColor = f_color;
        }
    )")
{
    uniformMap.insert({ "color", glGetUniformLocation(shaderHandle, "color") });
    uniformMap.insert({ "model", glGetUniformLocation(shaderHandle, "model") });
    uniformMap.insert({ "view", glGetUniformLocation(shaderHandle, "view") });
    uniformMap.insert({ "proj", glGetUniformLocation(shaderHandle, "proj") });
}

} // namespace muli