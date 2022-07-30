#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_transform_2d.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/norm.hpp>
#include <imgui.h>

// stds
#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <typeinfo>
#include <queue>
#include <stack>
#include <set>
#include <unordered_set>
#include <map>
#include <unordered_map>
#include <numeric>
#include <algorithm>
#include <limits>
#include <optional>

// Some useful operator overloading for glm containers
namespace glm
{
// 3x3 matrix * 2d vector multiplication
template<typename T, qualifier Q>
GLM_FUNC_QUALIFIER typename mat<2, 2, T, Q>::col_type mul(mat<3, 3, T, Q> const& m, typename mat<2, 2, T, Q>::row_type const& v, float z = 1.0f)
{
    return typename mat<2, 2, T, Q>::col_type
    (
        m[0][0] * v.x + m[1][0] * v.y + m[2][0] * z,
        m[0][1] * v.x + m[1][1] * v.y + m[2][1] * z
    );
}

template<typename T, qualifier Q>
GLM_FUNC_QUALIFIER typename mat<2, 2, T, Q>::col_type operator*(mat<3, 3, T, Q> const& m, typename mat<2, 2, T, Q>::row_type const& v)
{
    return mul(m, v, 1);
}

// 2d vector cross function returing scalar
template<typename T, qualifier Q>
GLM_FUNC_QUALIFIER float cross(vec<2, T, Q> const& v1, vec<2, T, Q> const& v2)
{
    return v1.x * v2.y - v1.y * v2.x;
}

// 1d cross returning vec2
template<typename T, qualifier Q>
GLM_FUNC_QUALIFIER vec<2, T, Q> cross(float s, vec<2, T, Q> const& v)
{
    return typename vec<2, T, Q>{ -s * v.y, s* v.x };
}

// clear vec2
template<typename T, qualifier Q>
GLM_FUNC_QUALIFIER void clear(vec<2, T, Q>& v, float value = 0.0f)
{
    v.x = value;
    v.y = value;
}

// to string
template<typename T, qualifier Q>
GLM_FUNC_QUALIFIER std::string operator*(vec<2, T, Q>& v)
{
    return glm::to_string(v);
}
}