#include "rigidbody_shader.h"

namespace muli
{

RigidBodyShader::RigidBodyShader()
    : Shader(
          // Vertex shader
          R"(
        #version 330 core

        layout (location = 0) in vec3 pos;
        layout (location = 1) in vec2 _texCoord;

        out vec2 texCoord;

        uniform mat4 view;
        uniform mat4 proj;
        uniform mat4 model;

        void main()
        {
           mat4 mvp = proj * view * model;
           gl_Position = mvp * vec4(pos, 1.0);

           texCoord = _texCoord;
        }
    )",
          // Fragment shader
          R"(
        #version 330 core
        
        out vec4 fragColor;

        in vec2 texCoord;

        uniform vec4 color;

        void main()
        {
            // vec2 uv = texCoord; // barycentric coordinate
            
            fragColor = color;
        }
    )")
{
    uniformMap.insert({ "color", glGetUniformLocation(shaderHandle, "color") });
    uniformMap.insert({ "model", glGetUniformLocation(shaderHandle, "model") });
    uniformMap.insert({ "view", glGetUniformLocation(shaderHandle, "view") });
    uniformMap.insert({ "proj", glGetUniformLocation(shaderHandle, "proj") });
}

} // namespace muli