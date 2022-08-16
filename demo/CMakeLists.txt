set(WINDOW_TITLE "SPE Demo")
set(WINDOW_WIDTH 1280)
set(WINDOW_HEIGHT 720)

set(DEMO_SOURCE_FILES
    src/main.cpp
    src/window.cpp
    src/application.cpp
    src/input.cpp
    src/game.cpp
    src/camera.cpp
    src/util.cpp

    src/rendering/shader.cpp
    src/rendering/rigidbody_shader.cpp
    src/rendering/rigidbody_renderer.cpp
    src/rendering/mesh.cpp
    src/rendering/dynamic_shader.cpp
    src/rendering/dynamic_renderer.cpp
)

add_executable(demo ${DEMO_SOURCE_FILES})
target_compile_definitions(demo PUBLIC
    WINDOW_TITLE="${WINDOW_TITLE}"
    WINDOW_WIDTH=${WINDOW_WIDTH}
    WINDOW_HEIGHT=${WINDOW_HEIGHT}
)
target_compile_options(demo PUBLIC /wd4566 /wd4819 /wd4251 /wd4244 /wd4267)

target_include_directories(demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../extern/imgui
    ${DEP_INCLUDE_DIR}
)

target_link_directories(demo PRIVATE ${DEP_LIB_DIR})
target_link_libraries(demo PRIVATE ${DEP_LIBS_DEMO} spe)

add_dependencies(demo ${DEP_LIST_DEMO} spe)
