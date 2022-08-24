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
    collision/broad_phase.cpp

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

target_include_directories(spe
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
    ${DEP_INCLUDE_DIR}
)

install(
    TARGETS spe
    EXPORT speConfig
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
    EXPORT speConfig
    NAMESPACE spe::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spe
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/speConfigVersion.cmake
    COMPATIBILITY SameMajorVersion
)

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/speConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/spe
)