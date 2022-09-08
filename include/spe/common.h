#pragma once

// glm
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/matrix_transform_2d.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/string_cast.hpp>

// stds
#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <queue>
#include <set>
#include <stack>
#include <string>
#include <typeinfo>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "growable_array.h"
#include "math.h"

// Some useful operator overloading for glm containers
namespace glm
{

// 3x3 matrix * 2d vector multiplication
template <typename T, qualifier Q>
GLM_FUNC_QUALIFIER typename mat<2, 2, T, Q>::col_type mul(mat<3, 3, T, Q> const& m,
                                                          typename mat<2, 2, T, Q>::row_type const& v,
                                                          float z = 1.0f)
{
    // clang-format off
    return typename mat<2, 2, T, Q>::col_type
    (
        m[0][0] * v.x + m[1][0] * v.y + m[2][0] * z,
        m[0][1] * v.x + m[1][1] * v.y + m[2][1] * z
    );
    // clang-format on
}

template <typename T, qualifier Q>
GLM_FUNC_QUALIFIER typename mat<2, 2, T, Q>::col_type operator*(mat<3, 3, T, Q> const& m,
                                                                typename mat<2, 2, T, Q>::row_type const& v)
{
    return mul(m, v, 1);
}

// 2d vector cross function returing scalar
template <typename T, qualifier Q>
GLM_FUNC_QUALIFIER float cross(vec<2, T, Q> const& v1, vec<2, T, Q> const& v2)
{
    return v1.x * v2.y - v1.y * v2.x;
}

// 1d cross returning vec2
template <typename T, qualifier Q>
GLM_FUNC_QUALIFIER vec<2, T, Q> cross(float s, vec<2, T, Q> const& v)
{
    return vec<2, T, Q>{ -s * v.y, s * v.x };
}

// clear vec2
template <typename T, qualifier Q>
GLM_FUNC_QUALIFIER void clear(vec<2, T, Q>& v, float value = 0.0f)
{
    v.x = value;
    v.y = value;
}

// to string
template <typename T>
GLM_FUNC_QUALIFIER std::string operator*(T& v)
{
    return glm::to_string(v);
}

} // namespace glm