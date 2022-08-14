#include "rigidbody_shader.h"    

namespace spe
{

std::unique_ptr<RigidBodyShader> RigidBodyShader::Create()
{
    return std::unique_ptr<RigidBodyShader>(new RigidBodyShader);
}

RigidBodyShader::RigidBodyShader() : Shader(
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

        uniform vec3 color;

        void main()
        {
            vec2 uv = texCoord; // barycentric coordinate
            
            fragColor = vec4(color, 1.0f);
        }
    )"
)
{
    uniformMap.insert({"color", glGetUniformLocation(shaderHandle, "color")});
    uniformMap.insert({"model", glGetUniformLocation(shaderHandle, "model")});
    uniformMap.insert({"view", glGetUniformLocation(shaderHandle, "view")});
    uniformMap.insert({"proj", glGetUniformLocation(shaderHandle, "proj")});
}

void RigidBodyShader::SetColor(glm::vec3 _color)
{
    color = std::move(_color);
    glUniform3fv(uniformMap["color"], 1, &color.r);
}

void RigidBodyShader::SetModelMatrix(glm::mat4 _modelMatrix)
{
    modelMatrix = std::move(_modelMatrix);
    glUniformMatrix4fv(uniformMap["model"], 1, GL_FALSE, glm::value_ptr(modelMatrix));
}

void RigidBodyShader::SetViewMatrix(glm::mat4 _viewMatrix)
{
    viewMatrix = std::move(_viewMatrix);
    glUniformMatrix4fv(uniformMap["view"], 1, GL_FALSE, glm::value_ptr(viewMatrix));
}

void RigidBodyShader::SetProjectionMatrix(glm::mat4 _projMatrix)
{
    projMatrix = std::move(_projMatrix);
    glUniformMatrix4fv(uniformMap["proj"], 1, GL_FALSE, glm::value_ptr(projMatrix));
}

}