set(SOURCE_FILES
    main.cpp
    window.cpp
    application.cpp
    input.cpp
    game.cpp
    entity.cpp
    camera.cpp

    rendering/shader.cpp
    rendering/rigidbody_shader.cpp
    rendering/rigidbody_renderer.cpp
    rendering/mesh.cpp
    rendering/dynamic_shader.cpp
    rendering/dynamic_renderer.cpp

    physics/world.cpp

    physics/rigidbody.cpp
    physics/polygon.cpp
    physics/circle.cpp
    physics/box.cpp
    physics/util.cpp
    physics/aabb.cpp
    physics/aabbtree.cpp
    physics/detection.cpp
    physics/simplex.cpp
    physics/polytope.cpp
    physics/edge.cpp

    physics/constraint.cpp
    physics/contact_constraint.cpp
    physics/contact_solver.cpp
    physics/block_solver.cpp
    physics/island.cpp
    physics/joint.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC ${DEP_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/include)
target_link_directories(${PROJECT_NAME} PUBLIC ${DEP_LIB_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC ${DEP_LIBS})
