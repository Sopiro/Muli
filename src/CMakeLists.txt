set(SOURCE_FILES
    dynamics/entity.cpp
    dynamics/world.cpp
    dynamics/island.cpp

    dynamics/rigidbody.cpp
    dynamics/polygon.cpp
    dynamics/circle.cpp
    dynamics/box.cpp
    dynamics/util.cpp

    collision/aabb.cpp
    collision/aabbtree.cpp
    collision/detection.cpp
    collision/simplex.cpp
    collision/polytope.cpp
    collision/edge.cpp

    constraint/constraint.cpp
    constraint/contact/contact_constraint.cpp
    constraint/contact/contact_solver.cpp
    constraint/contact/block_solver.cpp

    constraint/joint/joint.cpp
    constraint/joint/grab_joint.cpp
    constraint/joint/revolute_joint.cpp
    constraint/joint/distance_joint.cpp
)

add_library(spe ${SOURCE_FILES})

target_include_directories(spe PUBLIC
    ${DEP_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/include
)

target_link_directories(spe PUBLIC ${DEP_LIB_DIR})
target_link_libraries(spe PUBLIC ${DEP_LIBS})

add_dependencies(spe ${DEP_LIST})