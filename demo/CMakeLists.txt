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

    src/demo/demo.cpp
    src/demo/single_box.cpp
    src/demo/box_stacking.cpp
    src/demo/pyramid.cpp
    src/demo/single_pendulum.cpp
    src/demo/springs.cpp
    src/demo/random_convex_polygons.cpp
    src/demo/seesaw.cpp
    src/demo/frictions.cpp
    src/demo/restitutions.cpp
    src/demo/multi_pendulum.cpp
    src/demo/suspension_bridge.cpp
    src/demo/circle_stacking.cpp
    src/demo/circles_1000.cpp
    src/demo/boxes_1000.cpp
    src/demo/capsules_1000.cpp
    src/demo/convex_polygons_1000.cpp
    src/demo/mix_1000.cpp
    src/demo/dense_collision.cpp
    src/demo/kinematic_body.cpp
    src/demo/collision_detection.cpp
    src/demo/dynamic_aabb_tree.cpp
    src/demo/convex_hull.cpp
    src/demo/compute_distance_point.cpp
    src/demo/cloth.cpp
    src/demo/collision_filtering.cpp
    src/demo/angle_joint_test.cpp
    src/demo/weld_joint_test.cpp
    src/demo/prismatic_joint_test.cpp
    src/demo/conveyor_belt.cpp
    src/demo/pulley.cpp
    src/demo/windmill.cpp
    src/demo/crank.cpp
    src/demo/terrain.cpp
    src/demo/rounded_polygon.cpp
    src/demo/ray_casting.cpp
    src/demo/line_of_sight.cpp
)

set(DEMO_HEADER_FILES
    include/mesh.h
    include/renderer.h
    include/shader.h
    include/dynamic_renderer.h
    include/dynamic_shader.h
    include/rigidbody_renderer.h
    include/rigidbody_shader.h

    include/application.h
    include/window.h
    include/camera.h
    include/common.h
    include/demo.h
    include/game.h
    include/input.h
    include/util.h
    include/options.h
)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/src" PREFIX "src" FILES ${DEMO_SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/include" PREFIX "include" FILES ${DEMO_HEADER_FILES})

add_executable(demo ${DEMO_SOURCE_FILES} ${DEMO_HEADER_FILES})
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
