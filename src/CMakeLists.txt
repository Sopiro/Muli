set(SOURCE_FILES
    entity.cpp

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

add_library(spe ${SOURCE_FILES})

target_include_directories(spe PUBLIC
    ${DEP_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/include
)

target_link_directories(spe PUBLIC ${DEP_LIB_DIR})
target_link_libraries(spe PUBLIC ${DEP_LIBS})

add_dependencies(spe ${DEP_LIST})