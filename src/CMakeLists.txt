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
target_compile_options(spe PUBLIC /wd4566 /wd4819 /wd4251 /wd4244 /wd4267)

target_include_directories(spe
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${DEP_INCLUDE_DIR}
)

install(
    TARGETS spe
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
