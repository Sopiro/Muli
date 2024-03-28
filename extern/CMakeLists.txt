# external dependencies

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# glad
add_subdirectory(glad)
set_target_properties(glad glad-generate-files PROPERTIES FOLDER "GLAD")

# glfw
option(GLFW_BUILD_EXAMPLES OFF)
option(GLFW_BUILD_TESTS OFF)
option(GLFW_BUILD_DOCS OFF)
add_subdirectory(glfw)

# imgui 
file(GLOB IMGUI_HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/imgui/*.h)
file(GLOB IMGUI_SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/imgui/*.cpp)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "include" FILES ${IMGUI_HEADER_FILES})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "src" FILES ${IMGUI_SOURCE_FILES})

add_library(imgui ${IMGUI_HEADER_FILES} ${IMGUI_SOURCE_FILES}
    imgui/backends/imgui_impl_opengl3.cpp
    imgui/backends/imgui_impl_glfw.cpp
)

target_include_directories(imgui
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui
)

target_link_libraries(imgui glfw)
