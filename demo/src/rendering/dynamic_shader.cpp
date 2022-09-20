#include "dynamic_shader.h"

namespace muli
{

std::unique_ptr<DynamicShader> DynamicShader::Create()
{
    return std::unique_ptr<DynamicShader>(new DynamicShader);
}

DynamicShader::DynamicShader()
    : Shader(
          // Vertex shader
          R"(
        #version 330 core

        layout (location = 0) in vec2 pos;

        uniform mat4 view;
        uniform mat4 proj;
        uniform mat4 model;

        void main()
        {
           mat4 mvp = proj * view * model;
           gl_Position = mvp * vec4(pos, 0.0, 1.0);
        }
    )",
          // Fragment shader
          R"(
        #version 330 core
        
        out vec4 fragColor;

        uniform vec3 color;

        void main()
        {
            fragColor = vec4(color, 1.0f);
        }
    )")
{
    uniformMap.insert({ "color", glGetUniformLocation(shaderHandle, "color") });
    uniformMap.insert({ "model", glGetUniformLocation(shaderHandle, "model") });
    uniformMap.insert({ "view", glGetUniformLocation(shaderHandle, "view") });
    uniformMap.insert({ "proj", glGetUniformLocation(shaderHandle, "proj") });
}

} // namespace muli