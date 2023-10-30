#pragma once

#include "common.h"
#include "util.h"

#include <unordered_map>

namespace muli
{

class Shader : NonCopyable
{
public:
    virtual ~Shader() noexcept;

    void Use() const;

protected:
    Shader(const char* vsCode, const char* fsCode);

    std::unordered_map<std::string, GLuint> uniformMap;
    GLuint shaderHandle;
};

inline void Shader::Use() const
{
    glUseProgram(shaderHandle);
}

} // namespace muli