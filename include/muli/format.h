#pragma once

#include "common.h"

namespace muli
{

template <typename... Args>
inline std::string FormatString(const char* format, Args... args)
{
    int size = 1 + std::snprintf(nullptr, 0, format, args...);

    std::string str(size, '\0');
    std::snprintf(&str[0], size, format, args...);

    str.pop_back(); // remove trailing null charactor

    return str;
}

} // namespace muli
