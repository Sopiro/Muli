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
add_subdirectory(imgui)