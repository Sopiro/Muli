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

target_include_directories(demo PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_directories(demo PUBLIC ${DEP_LIB_DIR})
target_link_libraries(demo PUBLIC ${DEP_LIBS_DEMO} spe)

add_dependencies(demo ${DEP_LIST_DEMO} spe)
