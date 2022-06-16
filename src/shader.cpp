#include "shader.h"    

Shader::Shader(uint32_t _vertexShader, uint32_t _fragmentShader, uint32_t _shaderProgram) :
    vertexShader{ std::move(_vertexShader) },
    fragmentShader{ std::move(_fragmentShader) },
    shaderProgram{ std::move(_shaderProgram) }
{

}

Shader::~Shader()
{
    SPDLOG_INFO("Terminate shader");

    if (vertexShader)
        glDeleteShader(vertexShader);
    if (fragmentShader)
        glDeleteShader(fragmentShader);
    if (shaderProgram)
        glDeleteProgram(shaderProgram);
}

std::unique_ptr<Shader> Shader::create(const char* vsCode, const char* fsCode)
{
    // vertex shader
    int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vsCode, NULL);
    glCompileShader(vertexShader);

    int success = 0;
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char infoLog[1024];
        glGetShaderInfoLog(vertexShader, 1024, nullptr, infoLog);
        SPDLOG_ERROR("failed to compile vertex shader: {}", infoLog);
        return nullptr;
    }

    // fragment shader
    int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fsCode, NULL);
    glCompileShader(fragmentShader);

    success = 0;
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char infoLog[1024];
        glGetShaderInfoLog(fragmentShader, 1024, nullptr, infoLog);
        SPDLOG_ERROR("failed to compile fragment shader: {}", infoLog);
        return nullptr;
    }

    // pipeline (or program)
    int shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    success = 0;
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success)
    {
        char infoLog[1024];
        glGetProgramInfoLog(shaderProgram, 1024, nullptr, infoLog);
        SPDLOG_ERROR("failed to link program: {}", infoLog);
        return nullptr;
    }

    // SPDLOG_INFO("{} {} {}", vertexShader, fragmentShader, shaderProgram);

    // Here's where the RVO comes into play
    return std::unique_ptr<Shader>(new Shader(vertexShader, fragmentShader, shaderProgram));
}

void Shader::setColor(glm::vec3 _color)
{
    color = std::move(_color);
}

void Shader::use()
{
    glUniform3fv(glGetUniformLocation(shaderProgram, "color"), 1, &color.r);
    glUseProgram(shaderProgram);
}