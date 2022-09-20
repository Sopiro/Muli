set(WINDOW_TITLE "Muli Engine Demo")
set(WINDOW_WIDTH 1280)
set(WINDOW_HEIGHT 720)

set(DEMO_SOURCE_FILES
    src/main.cpp
    src/window.cpp
    src/application.cpp
    src/game.cpp
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

target_include_directories(demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../extern/imgui
    ${DEP_INCLUDE_DIR}
)

target_link_directories(demo PRIVATE ${DEP_LIB_DIR})
target_link_libraries(demo PRIVATE ${DEP_LIBS_DEMO} muli)

add_dependencies(demo ${DEP_LIST_DEMO} muli)
