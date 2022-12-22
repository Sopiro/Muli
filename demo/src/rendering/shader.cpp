#include "shader.h"

namespace muli
{

Shader::Shader(const char* vsCode, const char* fsCode)
{
    // vertex shader
    int32 vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vsCode, NULL);
    glCompileShader(vertexShader);

    int32 success = 0;
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char infoLog[1024];
        glGetShaderInfoLog(vertexShader, 1024, nullptr, infoLog);
        std::printf("failed to compile vertex shader: %s\n", infoLog);
        exit(1);
    }

    // fragment shader
    int32 fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fsCode, NULL);
    glCompileShader(fragmentShader);

    success = 0;
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        char infoLog[1024];
        glGetShaderInfoLog(fragmentShader, 1024, nullptr, infoLog);
        std::printf("failed to compile fragment shader: %s\n", infoLog);
        exit(1);
    }

    // pipeline (or program)
    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);

    success = 0;
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success)
    {
        char infoLog[1024];
        glGetProgramInfoLog(shaderProgram, 1024, nullptr, infoLog);
        std::printf("failed to link program: %s\n", infoLog);
        exit(1);
    }

    glDetachShader(shaderProgram, vertexShader);
    glDetachShader(shaderProgram, fragmentShader);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    shaderHandle = shaderProgram;
}

Shader::~Shader() noexcept
{
    if (shaderHandle)
    {
        glDeleteProgram(shaderHandle);
    }
}

} // namespace muli