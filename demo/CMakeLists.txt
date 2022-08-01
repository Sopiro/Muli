set(DEMO_SOURCE_FILES
    main.cpp
    window.cpp
    application.cpp
    input.cpp
    game.cpp
    camera.cpp
    util.cpp

    rendering/shader.cpp
    rendering/rigidbody_shader.cpp
    rendering/rigidbody_renderer.cpp
    rendering/mesh.cpp
    rendering/dynamic_shader.cpp
    rendering/dynamic_renderer.cpp
)

add_executable(demo ${DEMO_SOURCE_FILES})

target_include_directories(demo PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

target_link_directories(demo PUBLIC ${DEP_LIB_DIR})
target_link_libraries(demo PUBLIC ${DEP_LIBS_DEMO} spe)

add_dependencies(demo ${DEP_LIST_DEMO} spe)
