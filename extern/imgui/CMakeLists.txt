add_library(imgui
    imgui_draw.cpp
    imgui_tables.cpp
    imgui_widgets.cpp
    imgui.cpp
    imgui_impl_glfw.cpp
    imgui_impl_opengl3.cpp
    imgui_demo.cpp
)

target_include_directories(imgui
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(imgui glfw)
