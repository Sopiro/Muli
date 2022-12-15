#pragma once

#include "common.h"

namespace muli
{

class Shader
{
public:
    virtual ~Shader() noexcept;

    Shader(const Shader&) noexcept = delete;
    Shader& operator=(const Shader&) noexcept = delete;

    Shader(Shader&&) noexcept = delete;
    Shader& operator=(Shader&&) noexcept = delete;

    void Use() const;

protected:
    Shader(const char* vsCode, const char* fsCode);

    std::unordered_map<std::string, uint32> uniformMap;
    uint32 shaderHandle{ 0 };
};

inline void Shader::Use() const
{
    glUseProgram(shaderHandle);
}

} // namespace muli